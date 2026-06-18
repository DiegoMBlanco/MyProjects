#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')

        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,      # Tomado de tu terminal: RELIABLE
            durability=DurabilityPolicy.VOLATILE,        # Tomado de tu terminal: VOLATILE
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscriber = self.create_subscription(LaserScan, '/r1/hokuyo', self.lidar_callback, custom_qos)
        self.publisher = self.create_publisher(Bool, '/obstacle_detected_sim', 10) # 1 = hay obstáculo, 0 = no hay

    def lidar_callback(self,msg):
        range = msg.ranges
        message = Bool()
        if math.isnan(range[0]) or math.isinf(range[0]):
        # Opción A: Si es nan, asumimos que NO hay obstáculo (camino libre)
            message.data = False
        elif range[0] < 0.5:
            message.data = True
        else:
            message.data = False

        self.publisher.publish(message)
            


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
