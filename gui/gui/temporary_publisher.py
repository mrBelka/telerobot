import rclpy
from rclpy.node import Node

from telerobot_interfaces.msg import Power
import random

class TestPublisher(Node):
    def __init__(self):
        super().__init__('Test_Charge_Publisher')
        self.publisher_ = self.create_publisher(Power, 'power', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        self.get_logger().info("test Charge Pub is working")
    def publish_message(self):
        msg = Power()
        msg.percent = random.uniform(1.0, 100.0)
        msg.voltage = random.uniform(1.0, 100.0)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    charge_pub = TestPublisher()
    rclpy.spin(charge_pub)
    charge_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()