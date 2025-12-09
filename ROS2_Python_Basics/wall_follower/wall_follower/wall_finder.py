#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan
from custom_interfaces.srv import FindWall
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time

class FindWallService(Node):
    def __init__(self):
        super().__init__('find_wall_service')
        self.reentrant_group_1 = ReentrantCallbackGroup()
        self.service = self.create_service(FindWall, '/find_wall', self.service_callback,callback_group=self.reentrant_group_1)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth = 10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.reentrant_group_1)
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth = 10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.reentrant_group_1)
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10, callback_group=self.reentrant_group_1)
        self.message = LaserScan()
    def laserscan_callback(self, msg):
        self.message = msg
        self.front_range = msg.ranges[0]
        self.right_range = msg.ranges[540] # 720/4
        
    

    def service_callback(self, request, response):
        self.get_logger().info("Request received")
        self.min_range = min(self.message.ranges)
        for i in range(len(self.message.ranges)):
            if (self.message.ranges[i] == self.min_range): #Index for min
                self.min_index = float(i)/2.0
                self.goal_angle = self.message.angle_min + (self.min_index * self.message.angle_increment)
                self.get_logger().info(f'Goal Angle [{i}]: {self.goal_angle:.4f}')
                break
        print(self.goal_angle)
        msg = Twist()

        error = self.goal_angle - self.odom_orientation_y
        while (abs(error) > 0.05):
            msg.angular.z = 0.4
            self.publisher.publish(msg)
            self.get_logger().info('Yaw: "%s'%self.odom_orientation_y)
            error = self.goal_angle - self.odom_orientation_y
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        #self.get_logger().info('Publishing: "%s'%msg)


        while(self.front_range > 0.3):
            msg.linear.x = 0.05
            self.publisher.publish(msg)
            #self.get_logger().info('Publishing: "%s'%msg)
        msg.linear.x = 0.0
        self.publisher.publish(msg)
        #self.get_logger().info('Publishing: "%s'%msg)


        self.get_logger().info('Orientation seek started')

        while(self.odom_orientation_y > self.message.angle_increment):
            msg.angular.z = 0.4
            self.publisher.publish(msg)
            self.get_logger().info('Yaw: "%s'%self.odom_orientation_y)
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        #self.get_logger().info('Publishing: "%s'%msg)



        response.wallfound = True
        
        
        return response

    def odom_callback(self, msg):
        angles = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.odom_orientation_r = round(angles["r"], 3)
        self.odom_orientation_p = round(angles["p"], 3)
        self.odom_orientation_y = round(angles["y"], 3)

        
    def euler_from_quaternion(self, quat_x, quat_y, quat_z, quat_w):
        # function to convert quaternions to euler angles
        # calculate roll
        sinr_cosp = 2 * (quat_w * quat_x + quat_y * quat_z)
        cosr_cosp = 1 - 2 * (quat_x * quat_x + quat_y * quat_y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # calculate pitch
        sinp = 2 * (quat_w * quat_y - quat_z * quat_x)
        pitch = math.asin(sinp)
        # calculate yaw
        siny_cosp = 2 * (quat_w * quat_z + quat_x * quat_y)
        cosy_cosp = 1 - 2 * (quat_y * quat_y + quat_z * quat_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # store the angle values in a dict
        angles = dict()
        angles["r"] = roll
        angles["p"] = pitch
        angles["y"] = yaw
        # return the angle values
        return angles

def main(args = None):
    rclpy.init(args=args)
    server = FindWallService()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    try:
        executor.spin()
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
