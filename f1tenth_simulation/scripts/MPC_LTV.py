#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        self.v_target = 0.5  # Velocidad crucero deseada
        self.Ts = 0.1        
        self.N = 15          
        
        # AHORA: 3 estados [ey, epsi, ev] y 2 entradas [delta, acc]
        self.nx = 3          
        self.nu = 2          

        # Pesos del MPC (Ajustados para 3x2)
        # Penalizamos: [Error lateral, Error ángulo, Error velocidad]
        self.Q = sparse.diags([50.0, 40.0, 50.0])   
        self.QN = self.Q # Costo terminal
        # Penalizamos: [Uso de volante, Aceleración brusca]
        self.R = sparse.diags([0.2, 1.0])    

        # Restricciones
        self.delta_max = 0.6  
        self.acc_max = 1.0
        self.umin = np.array([-self.delta_max, -self.acc_max])
        self.umax = np.array([ self.delta_max,  self.acc_max])
        
        self.xmin = np.array([-np.inf, -np.inf, -2.0]) # ev min
        self.xmax = np.array([ np.inf,  np.inf,  2.0]) # ev max

        # Variables de estado actual
        self.x_robot = 0.0
        self.y_robot = 0.0
        self.yaw_robot = 0.0
        self.v_current = 0.0
        self.path = []
        self.target_idx = 0
        self.executing = False

        # Comunicación
        self.sub_path = self.create_subscription(Path, "/drawn_plan", self.path_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/r1/odom", self.odom_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, "/r1/cmd_vel", 10)
        self.pub_CTE = self.create_publisher(Float32, "/CTE", 10)
        self.pub_e_psi = self.create_publisher(Float32, "/e_psi", 10)
        self.pub_solve_time = self.create_publisher(Float32, "/solve_time_ms", 10)
        self.srv_start = self.create_service(Empty, '/start_execution', self.start_cb)
        
        self.timer = self.create_timer(self.Ts, self.control_loop)
        
        # El setup de OSQP se llamará la primera vez en el loop porque depende de v
        self.prob = None

    def get_model_matrices(self, v):
        """Linealiza el sistema basado en la velocidad actual"""
        v = max(v, 0.1) # Evitar división por cero o velocidades nulas
        Ac = np.array([[0.0, v,   0.0],
                       [0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0]])
        
        Bc = np.array([[v * (self.Lr / self.L), 0.0],
                       [v / self.L,             0.0],
                    [0.0,                    1.0]])
        
        Ad, Bd, _, _, _ = cont2discrete((Ac, Bc, np.eye(3), np.zeros((3, 2))), self.Ts)
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

    def get_path_curvature(self, idx):
        """Estima la curvatura máxima en el horizonte de predicción"""
        curvatures = []
        lookahead_curve = min(idx + self.N, len(self.path) - 2)
        
        for i in range(max(0, idx - 1), lookahead_curve):
            if i + 2 >= len(self.path):
                break
            p0 = self.path[i]
            p1 = self.path[i + 1]
            p2 = self.path[i + 2]
            
            # Vectores
            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])
            
            # Cambio de ángulo (curvatura aproximada)
            angle = abs(math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0]))
            angle = abs(math.atan2(math.sin(angle), math.cos(angle)))  # Normalizar
            
            dist = math.hypot(v1[0], v1[1]) + 1e-6
            curvatures.append(angle / dist)
        
        return max(curvatures) if curvatures else 0.0

    def get_adaptive_v_target(self, idx):
        """Velocidad de referencia inversamente proporcional a la curvatura"""
        kappa = self.get_path_curvature(idx)
        
        v_min = 0.15   # Velocidad mínima en curvas cerradas
        v_max = 0.5    # Velocidad crucero en rectas
        k_curve = 0.5  # Sensibilidad a la curvatura (ajustar según el robot)
        
        v_ref = v_max / (1.0 + k_curve * kappa)
        return float(np.clip(v_ref, v_min, v_max))

    def get_errors(self):
        if not self.path: return 0.0, 0.0, 0.0
        
        # Buscar punto más cercano
        lookahead = 20
        dx = [self.x_robot - p[0] for p in self.path[self.target_idx:self.target_idx+lookahead]]
        dy = [self.y_robot - p[1] for p in self.path[self.target_idx:self.target_idx+lookahead]]
        dist = np.hypot(dx, dy)
        self.target_idx += np.argmin(dist)
        
        ref_x, ref_y = self.path[min(self.target_idx, len(self.path)-1)]
        
        # Ángulo deseado
        if self.target_idx < len(self.path) - 1:
            next_p = self.path[self.target_idx + 1]
            ref_yaw = math.atan2(next_p[1] - ref_y, next_p[0] - ref_x)
        else:
            ref_yaw = self.yaw_robot

        e_psi = math.atan2(math.sin(self.yaw_robot - ref_yaw), math.cos(self.yaw_robot - ref_yaw))
        cte = (self.y_robot - ref_y) * math.cos(ref_yaw) - (self.x_robot - ref_x) * math.sin(ref_yaw)
        
        # Error de velocidad
        # e_v = self.v_current - self.v_target
        self.v_target = self.get_adaptive_v_target(self.target_idx)
        e_v = self.v_current - self.v_target

        send_cte = Float32()
        send_cte.data = cte
        self.pub_CTE.publish(send_cte)

        send_e_psi = Float32()
        send_e_psi.data = e_psi
        self.pub_e_psi.publish(send_e_psi)

        return cte, e_psi, e_v

    def control_loop(self):
        if not self.executing or not self.path:
            return

        # 1. Obtener errores
        cte, e_psi, e_v = self.get_errors()
        current_state = np.array([cte, e_psi, e_v])

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
        accel = res.x[start_u + 1]

        # 4. Enviar comandos (Integrando la aceleración)
        t = Twist()
        # Aplicamos la aceleración a la velocidad actual para el siguiente comando
        t.linear.x = self.v_current + (accel * self.Ts)
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