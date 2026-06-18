import numpy as np
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

class ExtendedKalmanFilter(Node):
    def __init__(self, Q, R, x0, P0):
        super().__init__('ekf_node')
        self.subscription = self.create_subscription(Odometry, '/r1/odom', self.odom_callback, 10)
        self.sub_imu = self.create_subscription(Imu, "/r1/imu", self.imu_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0
        self.theta_anterior = float(x0[2, 0])
        self.last_time_odom = None
        self.last_time_imu = None
        self.theta_imu = float(x0[2, 0])
        self.bias = -0.00075

    def yaw_to_quaternion(self, yaw):
        """Convierte un ángulo Yaw (radianes) a un mensaje Quaternion de ROS 2"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def publish_ekf_odom(self, stamp):
        """Construye y publica el mensaje de odometría filtrada"""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'r1/odom'          # Ajusta según tu árbol de tf
        msg.child_frame_id = 'r1/base_footprint' # Ajusta según tu árbol de tf

        # Llenar la posición estimada por el EKF
        msg.pose.pose.position.x = float(self.x[0, 0])
        msg.pose.pose.position.y = float(self.x[1, 0])
        msg.pose.pose.position.z = 0.0

        # Convertir el Yaw estimado a Cuaternión para la orientación
        msg.pose.pose.orientation = self.yaw_to_quaternion(self.x[2, 0])

        # Publicar el mensaje
        self.odom_pub.publish(msg)

    def wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def odom_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time_odom is None:
            self.last_time_odom = current_time
            return
        
        dt = current_time - self.last_time_odom
        self.last_time_odom = current_time

        if dt <= 0.0:
            return

        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        u = np.array([[v], [omega]])

        # Ejecutar Predicción
        self.predict(u, dt)
        
        self.publish_ekf_odom(msg.header.stamp)

    def imu_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time_imu is None:
            self.last_time_imu = current_time
            return
        
        dt = current_time - self.last_time_imu
        self.last_time_imu = current_time

        if dt <= 1e-4:
            return

        z_omega = msg.angular_velocity.z - self.bias

        self.theta_imu = self.wrap_angle(self.theta_imu + z_omega * dt) #Integración para obtener velocidad


        z = np.array([[self.theta_imu]])

        # Ejecutar Corrección/Actualización
        self.update(z)
        
        self.publish_ekf_odom(msg.header.stamp)


    def predict(self, u, dt):
        v = u[0, 0]
        omega = u[1, 0]
        theta = self.x[2, 0]

        # 1. Movimiento del estado usando la función no lineal g(u, x) [Línea 2 del algoritmo]
        self.x = np.array([[self.x[0, 0] + v * np.cos(theta) * dt],
                            [self.x[1, 0] + v * np.sin(theta) * dt],
                            [self.wrap_angle(theta + omega * dt)]])

        # 2. Calcular el Jacobiano G evaluado en el estado actual [Línea 3 del algoritmo]
        G = np.array([[1.0, 0.0, -v * np.sin(theta) * dt],
                      [0.0, 1.0,  v * np.cos(theta) * dt],
                      [0.0, 0.0,  1.0]])

        # 3. Propagar la covarianza usando el Jacobiano G en vez de A [Línea 3 del algoritmo]
        self.P = np.dot(G, np.dot(self.P, G.T)) + self.Q
        return self.x
    
    def update(self, z):
        # 1. Jacobiano H para un sensor IMU
        H = np.array([[0.0, 0.0, 1.0]])

        # 2. Calcular la ganancia de Kalman K usando H en vez de C [Línea 4 del algoritmo]
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(np.dot(H, np.dot(self.P, H.T)) + self.R))

        h_x = np.array([[self.x[2, 0]]])

        # 4. Corregir el estado usando la innovación [Línea 5 del algoritmo]
        self.x = self.x + np.dot(K, (z - h_x))

        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))

        self.theta_anterior = float(self.x[2,0])

        # 5. Actualizar la covarianza a posteriori [Línea 6 del algoritmo]
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, H), self.P)
        return self.x


def main(args=None):
    rclpy.init(args=args)

    # Matrices de ruido para el Puzzlebot
    Q = np.array([[0.0004, 0, 0], 
                  [0, 0.00002, 0], 
                  [0, 0, 0.0157]]) #0.0157
    
    R = np.array([[0.000081]]) #0.000081

    # Estado inicial: [x=0m, y=0m, theta=0rad]
    x0 = np.array([[0.0], [0.0], [0.0]])
    P0 = np.array([[0.1, 0, 0], 
                   [0, 0.1, 0], 
                   [0, 0, 0.1]])

    # Instanciar el EKF con tu misma estructura
    ekf_node = ExtendedKalmanFilter(Q, R, x0, P0)

    rclpy.spin(ekf_node)

    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()