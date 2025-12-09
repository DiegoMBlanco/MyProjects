import rclpy
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.action import ActionServer
from custom_interfaces.action import OdomRecord
import math
import asyncio
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class OdomRecorder(Node):
    def __init__(self):
        super().__init__('Odom_node_action')
        self.reentrant_group_1 = ReentrantCallbackGroup()
        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth = 10, reliability = ReliabilityPolicy.RELIABLE),
            callback_group=self.reentrant_group_1)
        self.action_server = ActionServer(
            self,
            OdomRecord, 
            '/record_odom', 
            self.execute_callback, 
            callback_group=self.reentrant_group_1)
        self.i = 1
        self.last_odom = Point()
        self.last_odom_2 = Point()
        self.first_odom = Point()
        self.timer_period = 1.0
        self.current_distance = 0.0
        self.odom_record = []
        self.total_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.odom_callback_flag = False
        
    def odom_callback(self, msg):
        self.message = msg

    def timer_counter(self):
        self.last_odom_2.x = self.message.pose.pose.position.x
        self.last_odom_2.y = self.message.pose.pose.position.y
        self.odom_record.append(self.last_odom)

    async def execute_callback(self,goal_handle):
        self.get_logger().info('Action goal started')
        self.first_odom.x = self.message.pose.pose.position.x
        self.first_odom.y = self.message.pose.pose.position.y
        self.create_timer(self.timer_period, self.timer_counter, callback_group=self.reentrant_group_1) # Inicia el timer para almacenar cada segundo la posición actual
        feedback_msg = OdomRecord.Feedback()
        self.current_distance = math.hypot(self.last_odom.x - self.first_odom.x, self.last_odom.y - self.first_odom.y)
        flag = False
        self.i = 0
        self.last_odom.x = self.message.pose.pose.position.x
        self.last_odom.y = self.message.pose.pose.position.y
        self.last_x = self.last_odom.x
        self.last_y = self.last_odom.y
        while(flag == False):
            self.last_odom.x = self.message.pose.pose.position.x
            self.last_odom.y = self.message.pose.pose.position.y
            self.total_distance += math.hypot(self.last_odom.x - self.last_x, self.last_odom.y - self.last_y)
            self.last_x = self.last_odom.x
            self.last_y = self.last_odom.y
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            self.current_distance = math.hypot(self.last_odom.x - self.first_odom.x, self.last_odom.y - self.first_odom.y)
            if self.i > 10000:
                if self.current_distance < 0.05:
                    flag = True
            self.i += 1
            #self.get_logger().info('Current Distance: "%s'%self.current_distance)
        goal_handle.succeed()
        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        return result

def main (args = None):
    rclpy.init(args = args)
    node = OdomRecorder()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
