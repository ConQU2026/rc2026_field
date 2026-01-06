import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TargetController(Node):


    def __init__(self):
        """初始化节点，声明参数并创建发布者和定时器。"""
        super().__init__('target_controller')
        
        self.declare_parameter('radius', 1.5)
        self.declare_parameter('speed', 0.6)
        
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Target Controller Started. Moving in circle with radius {self.radius} and speed {self.speed}.')

    def timer_callback(self):
        msg = Twist()
        
        msg.linear.x = self.speed
        if self.radius != 0:
             msg.angular.z = self.speed / self.radius
        else:
             msg.angular.z = 0.0
        
        self.publisher_.publish(msg)    


def main(args=None):
    """主函数，初始化并运行节点。"""
    rclpy.init(args=args)
    node = TargetController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

