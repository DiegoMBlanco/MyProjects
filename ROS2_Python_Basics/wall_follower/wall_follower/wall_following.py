import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.srv import FindWall
from rclpy.action import ActionClient
from custom_interfaces.action import OdomRecord

class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_node')
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth = 10, reliability=ReliabilityPolicy.RELIABLE))
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10)
        self.timer_period = 0.3
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.front_range = 0.0
        self.action_client = ActionClient(self,OdomRecord,'/record_odom')
        self.client = self.create_client(FindWall, '/find_wall')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available")
    def call_service(self):
        self.req = FindWall.Request()
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    def laserscan_callback(self, msg):
        #self.get_logger().info(f'Min angle: {msg.angle_min:.4f}') #used interface show
        #self.get_logger().info(f'Max angle: {msg.angle_max:.4f}')
        #self.get_logger().info(f'Min range: {msg.range_min:.4f}')
        #self.get_logger().info(f'Max range: {msg.range_max:.4f}')
        #self.get_logger().info(f'Min angle range: {msg.ranges[int(msg.angle_min)]:.4f}')
        #self.get_logger().info(f'Angle increment: {msg.angle_increment:.6f}')
        self.front_range = msg.ranges[0]
        #self.get_logger().info(f'Angle increment: {msg.angle_increment:.6f}')
        
        self.right_range = msg.ranges[540] #(720/4)*3
    

    def timer_callback(self):
        msg = Twist() # Vector3 linear,angular
        if (self.front_range < 0.5):
            msg.linear.x = 0.0
            msg.angular.z = 0.2
        elif (self.right_range > 0.35):
            msg.linear.x = 0.07
            msg.angular.z = -0.15
        elif(self.right_range < 0.3):
            msg.linear.x = 0.07
            msg.angular.z = 0.15
        else:
            msg.linear.x = 0.07
            msg.angular.z = 0.0
        self.publisher.publish(msg)
        #self.get_logger().info('Publishing: "%s'%msg)

    def send_goal(self):
        goal_msg = OdomRecord.Goal()
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg,feedback_callback = self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.list_of_odoms))
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        self.publisher.publish(msg)
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Total distance: {0}'.format(feedback.current_total))
            
    
def main(args = None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    response = node.call_service()
    node.get_logger().info(f'Respuesta del servidor:  {response.wallfound}')
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
