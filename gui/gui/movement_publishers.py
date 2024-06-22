import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class BodyMovePublisher(Node):
    def __init__(self):
        super().__init__('body_move_pub')
        self.get_logger().info("BodyMovePublisher is working")
        self.body_move_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_body_move_message)

        self.l_x = 0.0
        self.a_z = 0.0
        self.angular_speed = 1.0
        self.linear_speed = 1.0
    def publish_body_move_message(self):
        msg = Twist()
        msg.linear.x = self.l_x * self.linear_speed
        msg.angular.z = self.a_z * self.angular_speed
        self.body_move_pub_.publish(msg)

    def change_angular_speed(self, speed):
        self.angular_speed = speed
    def change_linear_speed(self, speed):
        self.linear_speed = speed

    def change_movement_components(self, l_x = 0.0, a_z = 0.0):
        self.l_x = l_x
        self.a_z = a_z