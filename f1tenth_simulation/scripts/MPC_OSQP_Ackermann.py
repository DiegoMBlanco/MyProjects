#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import osqp
from scipy import sparse
from scipy.signal import cont2discrete
from std_srvs.srv import Empty
import time

class AckermannMPC(Node):
    def __init__(self):
        super().__init__('ackermann_mpc_node')

        # Parámetros del Vehículo 
        self.L = 0.33   # Distancia entre ejes (Wheelbase) definida en ackermann.xacro. Reemplazar por su valor real.
        self.v_ref = 0.5     # Velocidad lineal constante (m/s)
        self.Ts = 0.1        # Tiempo de muestreo 
        self.N = 15          # Horizonte de predicción
        
        self.nx = 2          # Estados: [crosstrack_error (e_y), heading_error (e_psi)]
        self.nu = 1          # Entrada: [steering_angle (delta)]

        # Modelo de Error Linealizado
        Ac = np.array([[0.0, self.v_ref],
                       [0.0, 0.0]])
        Bc = np.array([[0.0],
                       [self.v_ref / self.L]])
        
        # Discretización
        Ad, Bd, _, _, _ = cont2discrete((Ac, Bc, np.eye(2), np.zeros((2, 1))), self.Ts)
        self.Ad = sparse.csc_matrix(Ad)
        self.Bd = sparse.csc_matrix(Bd)

        # Pesos del MPC
        Q = sparse.diags([40.0, 50.0])   # Penaliza error lateral y de ángulo  
        QN = sparse.diags([40.0, 50.0])  # Costo terminal
        R = 0.5 * sparse.eye(self.nu)    # Penaliza uso excesivo de volante  

        # Restricciones 
        self.delta_max = 0.6  
        self.umin = np.array([-self.delta_max])
        self.umax = np.array([ self.delta_max])
        
        self.xmin = np.array([-np.inf, -np.inf])
        self.xmax = np.array([ np.inf,  np.inf])

        # Configuración OSQP 
        self.setup_osqp(Q, QN, R)

        self.x_robot = 0.0
        self.y_robot = 0.0
        self.yaw_robot = 0.0
        self.path = []
        self.target_idx = 0
        self.executing = False

        self.sub_path = self.create_subscription(Path, "/drawn_plan", self.path_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/r1/odom", self.odom_cb, 10) # se incluye el namespace de la simulación del ackermann. Ajustar a su tópico correspondiente
        self.pub_cmd = self.create_publisher(Twist, "/r1/cmd_vel", 10)
        self.srv_start = self.create_service(Empty, '/start_execution', self.start_cb)
        
        self.timer = self.create_timer(self.Ts, self.control_loop)
        self.get_logger().info("Ackermann MPC Localizado listo.")

    def setup_osqp(self, Q, QN, R):
        # Matriz P (Costo cuadrático)
        P = sparse.block_diag([
            sparse.kron(sparse.eye(self.N), Q), QN,
            sparse.kron(sparse.eye(self.N), R)
        ], format='csc')

        # Vector q 
        self.q_vec = np.zeros((self.N + 1) * self.nx + self.N * self.nu)

        # Matriz Aeq (Dinámica del sistema: x_{k+1} = Ad*x_k + Bd*u_k)
        Ax = sparse.kron(sparse.eye(self.N + 1), -sparse.eye(self.nx)) + \
             sparse.kron(sparse.eye(self.N + 1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        
        self.leq = np.hstack([-np.zeros(self.nx), np.zeros(self.N * self.nx)])
        self.ueq = self.leq.copy()

        # Matriz Aineq (Límites de u)
        Aineq = sparse.eye((self.N + 1) * self.nx + self.N * self.nu)
        self.lineq = np.hstack([np.tile(self.xmin, self.N+1), np.tile(self.umin, self.N)])
        self.uineq = np.hstack([np.tile(self.xmax, self.N+1), np.tile(self.umax, self.N)])

        A = sparse.vstack([Aeq, Aineq], format='csc')
        self.l = np.hstack([self.leq, self.lineq])
        self.u = np.hstack([self.ueq, self.uineq])

        self.prob = osqp.OSQP()
        self.prob.setup(P, self.q_vec, A, self.l, self.u, warm_start=True, verbose=False)

    def odom_cb(self, msg: Odometry):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw_robot = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def path_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.target_idx = 0
        self.get_logger().info(f"Path recibido con {len(self.path)} puntos.")

    def start_cb(self, req, res):
        if self.path:
            self.executing = True
            self.get_logger().info("¡Ejecutando!")
        return res

    def get_errors(self):
        """Calcula Crosstrack Error (ey) y Heading Error (epsi)"""
        if not self.path: return 0.0, 0.0
        
        # Buscar punto más cercano
        dx = [self.x_robot - p[0] for p in self.path[self.target_idx:self.target_idx+20]]
        dy = [self.y_robot - p[1] for p in self.path[self.target_idx:self.target_idx+20]]
        dist = np.hypot(dx, dy)
        idx_rel = np.argmin(dist)
        self.target_idx += idx_rel
        
        ref_x, ref_y = self.path[self.target_idx]
        
        # Calcular ángulo deseado (tangente a la ruta)
        if self.target_idx < len(self.path) - 1:
            next_p = self.path[self.target_idx + 1]
            ref_yaw = math.atan2(next_p[1] - ref_y, next_p[0] - ref_x)
        else:
            ref_yaw = self.yaw_robot

        # Error de orientación (normalizado)
        e_psi = math.atan2(math.sin(self.yaw_robot - ref_yaw), math.cos(self.yaw_robot - ref_yaw))

        # Crosstrack Error (Proyección perpendicular)
        cte = (self.y_robot - ref_y) * math.cos(ref_yaw) - (self.x_robot - ref_x) * math.sin(ref_yaw)

        return cte, e_psi

    def control_loop(self):
        if not self.executing or not self.path:
            return

        cte, e_psi = self.get_errors()
        current_state = np.array([cte, e_psi])

        # Actualizar condición inicial en el optimizador
        self.l[:self.nx] = -current_state
        self.u[:self.nx] = -current_state
        
        self.prob.update(l=self.l, u=self.u)
        res = self.prob.solve()

        if res.info.status != 'solved':
            self.pub_cmd.publish(Twist())
            return

        # Extraer primer control (steering angle delta)
        start_u = (self.N + 1) * self.nx
        delta = res.x[start_u]

        # Enviar comandos
        t = Twist()
        t.linear.x = self.v_ref
        t.angular.z = float(delta) 
        self.pub_cmd.publish(t)

        if self.target_idx >= len(self.path) - 1:
            self.executing = False
            self.pub_cmd.publish(Twist())
            self.get_logger().info("Meta alcanzada.")

def main():
    rclpy.init()
    node = AckermannMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()