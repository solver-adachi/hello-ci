import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
