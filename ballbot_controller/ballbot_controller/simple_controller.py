#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")


        self.declare_parameter("wheel_radius", 0.17)
        self.declare_parameter("wheel_separation", 0.33)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value


        self.get_logger().info(f"Using wheel radius {self.wheel_radius_}")
        self.get_logger().info(f"Using wheel separation {self.wheel_separation_}")

        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray, "/simple_velocity_controller/commands", 10)

        self.vel_sub_ = self.create_subscription(
            TwistStamped, "/ballbot_controller/cmd_vel", self.velCallback, 10)

        # Invert to get wheel speeds from robot speeds
        self.speed_conversion_ = np.array([
            [self.wheel_radius_ / 2,              self.wheel_radius_ / 2],
            [self.wheel_radius_ / self.wheel_separation_, -self.wheel_radius_ / self.wheel_separation_]
        ])
        self.get_logger().info(f"The conversion matrix is:\n{self.speed_conversion_}")

    def velCallback(self, msg: TwistStamped):
        robot_speed = np.array([
            [msg.twist.linear.x],
            [msg.twist.angular.z]
        ])
        wheel_speed = np.linalg.inv(self.speed_conversion_) @ robot_speed

        wheel_speed_msg = Float64MultiArray()

        # BUG FIX 4: Wheel order was swapped [1,0] instead of [0,1]
        # simple_velocity_controller expects [left, right]
        # matching the joints order in ballbot_controllers.yaml:
        #   joints: [wheel_left_joint, wheel_right_joint]
        wheel_speed_msg.data = [wheel_speed[0, 0], wheel_speed[1, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)


def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
