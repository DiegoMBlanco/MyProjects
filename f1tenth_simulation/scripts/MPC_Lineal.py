#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import osqp
import time
from scipy import sparse
from scipy.signal import cont2discrete
from std_srvs.srv import Empty
from std_msgs.msg import Float32

class AckermannMPC(Node):
    def __init__(self):
        super().__init__('ackermann_mpc_speed_node')

        # Parámetros del Vehículo 
        self.L = 0.33        # Wheelbase
        self.Lr = 0.165
        self.v_fixed = 0.3
        self.Ts = 0.1        
        self.N = 15          
        
        # AHORA: 2 estados [ey, epsi] y 1 entrada [delta]
        self.nx = 2          
        self.nu = 1          

        # Pesos del MPC (Ajustados para 3x2)
        # Penalizamos: [Error lateral, Error ángulo]
        self.Q = sparse.diags([50.0, 40.0])   
        self.QN = self.Q # Costo terminal
        # Penalizamos: [Uso de volante]
        self.R = sparse.diags([0.2])    

        # Restricciones
        self.delta_max = 0.6  
        self.umin = np.array([-self.delta_max])
        self.umax = np.array([ self.delta_max])
        
        self.xmin = np.array([-np.inf, -np.inf]) 
        self.xmax = np.array([ np.inf,  np.inf]) 

        # Variables de estado actual
        self.x_robot = 0.0
        self.y_robot = 0.0
        self.yaw_robot = 0.0
        self.x_robot_ground_truth = 0.0
        self.y_robot_ground_truth = 0.0
        self.yaw_robot_ground_truth = 0.0
        self.v_current = 0.0
        self.path = []
        self.target_idx = 0
        self.executing = False

        # Comunicación
        self.sub_path = self.create_subscription(Path, "/drawn_plan", self.path_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/r1/odom", self.odom_cb, 10)
        self.sub_odom = self.create_subscription(PoseStamped, "/r1/ground_truth", self.ground_truth_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, "/r1/cmd_vel", 10)
        self.pub_CTE = self.create_publisher(Float32, "/CTE", 10)
        self.pub_e_psi = self.create_publisher(Float32, "/e_psi", 10)
        self.pub_CTE_gt = self.create_publisher(Float32, "/CTE_gt", 10)
        self.pub_e_psi_gt = self.create_publisher(Float32, "/e_psi_gt", 10)
        self.pub_solve_time = self.create_publisher(Float32, "/solve_time_ms", 10)
        self.srv_start = self.create_service(Empty, '/start_execution', self.start_cb)
        
        self.timer = self.create_timer(self.Ts, self.control_loop)
        
        # El setup de OSQP se llamará la primera vez en el loop porque depende de v
        self.prob = None

    def get_model_matrices(self, v):
        """Linealiza el sistema basado en la velocidad actual"""
        v_lineal = self.v_fixed
        
        # Matriz Ac de 2x2
        Ac = np.array([[0.0, v_lineal],
                    [0.0, 0.0]])
        
        # Matriz Bc de 2x1 (Tu ecuación exacta)
        Bc = np.array([[v_lineal * (self.Lr / self.L)],
                    [v_lineal / self.L]])
        
        # Discretización limpia para 2 estados y 1 entrada
        Ad, Bd, _, _, _ = cont2discrete((Ac, Bc, np.eye(2), np.zeros((2, 1))), self.Ts)
        return sparse.csc_matrix(Ad), sparse.csc_matrix(Bd)

    def setup_osqp(self, Ad, Bd):
        # (La lógica de construcción de P, A, l, u es igual pero con nx=3, nu=2)
        nx, nu, N = self.nx, self.nu, self.N
        
        P = sparse.block_diag([
            sparse.kron(sparse.eye(N), self.Q), self.QN,
            sparse.kron(sparse.eye(N), self.R)
        ], format='csc')

        q_vec = np.zeros((N + 1) * nx + N * nu)

        Ax = sparse.kron(sparse.eye(N + 1), -sparse.eye(nx)) + \
             sparse.kron(sparse.eye(N + 1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])
        
        leq = np.hstack([-np.zeros(nx), np.zeros(N * nx)])
        ueq = leq.copy()

        Aineq = sparse.eye((N + 1) * nx + N * nu)
        lineq = np.hstack([np.tile(self.xmin, N+1), np.tile(self.umin, N)])
        uineq = np.hstack([np.tile(self.xmax, N+1), np.tile(self.umax, N)])

        A = sparse.vstack([Aeq, Aineq], format='csc')
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])

        if self.prob is None:
            self.prob = osqp.OSQP()
            self.prob.setup(P, q_vec, A, l, u, warm_start=True, verbose=False)
        else:
            self.prob.update(Ax=A.data, l=l, u=u)

    def odom_cb(self, msg: Odometry):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y
        self.v_current = msg.twist.twist.linear.x
        q = msg.pose.pose.orientation
        self.yaw_robot = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def path_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.target_idx = 0

    def start_cb(self, req, res):
        if self.path: self.executing = True
        return res
    
    def ground_truth_cb(self, msg):
        self.x_robot_ground_truth = msg.pose.position.x
        self.y_robot_ground_truth = msg.pose.position.y
        q = msg.pose.orientation
        self.yaw_robot_ground_truth = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


    def get_errors(self):
        if not self.path: return 0.0, 0.0, 0.0

        lookahead = 20
        max_idx = len(self.path) - 1

        
        # 1. CÁLCULO CON ODOMETRÍA 
        dx_odom = [self.x_robot - p[0] for p in self.path[self.target_idx : self.target_idx + lookahead]]
        dy_odom = [self.y_robot - p[1] for p in self.path[self.target_idx : self.target_idx + lookahead]]
        dist_odom = np.hypot(dx_odom, dy_odom)
        self.target_idx += np.argmin(dist_odom)
        self.target_idx = min(self.target_idx, max_idx)
        
        ref_x_odom, ref_y_odom = self.path[self.target_idx]
        
        if self.target_idx < max_idx:
            ref_yaw_odom = math.atan2(self.path[self.target_idx + 1][1] - ref_y_odom, 
                                      self.path[self.target_idx + 1][0] - ref_x_odom)
        else:
            ref_yaw_odom = self.yaw_robot

        e_psi = math.atan2(math.sin(self.yaw_robot - ref_yaw_odom), math.cos(self.yaw_robot - ref_yaw_odom))
        cte = (self.y_robot - ref_y_odom) * math.cos(ref_yaw_odom) - (self.x_robot - ref_x_odom) * math.sin(ref_yaw_odom)


        # 2. CÁLCULO CON GROUND TRUTH (Para Análisis)
        if not hasattr(self, 'target_idx_gt'):
            self.target_idx_gt = 0

        dx_gt = [self.x_robot_ground_truth - p[0] for p in self.path[self.target_idx_gt : self.target_idx_gt + lookahead]]
        dy_gt = [self.y_robot_ground_truth - p[1] for p in self.path[self.target_idx_gt : self.target_idx_gt + lookahead]]
        dist_gt = np.hypot(dx_gt, dy_gt)
        self.target_idx_gt += np.argmin(dist_gt)
        self.target_idx_gt = min(self.target_idx_gt, max_idx)

        ref_x_gt, ref_y_gt = self.path[self.target_idx_gt]

        if self.target_idx_gt < max_idx:
            ref_yaw_gt = math.atan2(self.path[self.target_idx_gt + 1][1] - ref_y_gt, 
                                    self.path[self.target_idx_gt + 1][0] - ref_x_gt)
        else:
            ref_yaw_gt = self.yaw_robot_ground_truth

        e_psi_gt = math.atan2(math.sin(self.yaw_robot_ground_truth - ref_yaw_gt), math.cos(self.yaw_robot_ground_truth - ref_yaw_gt))
        cte_gt = (self.y_robot_ground_truth - ref_y_gt) * math.cos(ref_yaw_gt) - (self.x_robot_ground_truth - ref_x_gt) * math.sin(ref_yaw_gt)


        # Publicación Errores Odometría
        send_cte = Float32(data=cte)
        self.pub_CTE.publish(send_cte)

        send_e_psi = Float32(data=e_psi)
        self.pub_e_psi.publish(send_e_psi)

        # Publicación Errores Ground Truth 
        send_cte_gt = Float32(data=cte_gt)
        self.pub_CTE_gt.publish(send_cte_gt)

        send_e_psi_gt = Float32(data=e_psi_gt)
        self.pub_e_psi_gt.publish(send_e_psi_gt)

        return cte, e_psi
    

    def control_loop(self):
        if not self.executing or not self.path:
            return

        # 1. Obtener errores
        cte, e_psi = self.get_errors()
        current_state = np.array([cte, e_psi])

        # 2. Actualizar modelo con la v_actual y resolver
        Ad, Bd = self.get_model_matrices(self.v_current)
        self.setup_osqp(Ad, Bd)

        self.prob.update(l=np.hstack([-current_state, np.zeros(self.N * self.nx), np.tile(self.xmin, self.N+1), np.tile(self.umin, self.N)]),
                         u=np.hstack([-current_state, np.zeros(self.N * self.nx), np.tile(self.xmax, self.N+1), np.tile(self.umax, self.N)]))
        
        # ── Medir tiempo de solución ──────────────────────────────────────────
        t0 = time.perf_counter()
        res = self.prob.solve() # Resuelve
        solve_ms = (time.perf_counter() - t0) * 1000.0
        msg_time = Float32()
        msg_time.data = float(solve_ms)
        self.pub_solve_time.publish(msg_time)

        if res.info.status != 'solved':
            return

        # 3. Extraer controles
        start_u = (self.N + 1) * self.nx
        delta = res.x[start_u]

        # 4. Enviar comandos (Integrando la aceleración)
        t = Twist()
        # Aplicamos la aceleración a la velocidad actual para el siguiente comando
        t.linear.x = self.v_fixed
        t.angular.z = float(delta) 
        self.pub_cmd.publish(t)

        if self.target_idx >= len(self.path) - 1:
            self.executing = False
            self.pub_cmd.publish(Twist())

def main():
    rclpy.init()
    node = AckermannMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()