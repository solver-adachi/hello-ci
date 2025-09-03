import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import unittest
import threading
import time

from my_py_pkg.simple_node import SimpleNode


class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.msg = None
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.msg = msg.data


class TestSimpleNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_publishes_message(self):
        pub_node = SimpleNode()
        sub_node = TestSubscriber()

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(pub_node)
        executor.add_node(sub_node)

        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        timeout_sec = 3.0
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if sub_node.msg is not None:
                break
            time.sleep(0.1)

        self.assertIsNotNone(sub_node.msg)
        self.assertEqual(sub_node.msg, 'Hello, ROS 2!')

        executor.shutdown()
        pub_node.destroy_node()
        sub_node.destroy_node()


if __name__ == '__main__':
    unittest.main()
