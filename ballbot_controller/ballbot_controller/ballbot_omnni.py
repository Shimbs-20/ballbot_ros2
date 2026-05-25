#!/usr/bin/env python3
"""
ballbot_omni_controller.py
Converts /cmd_vel (Twist) to 3-omniwheel velocities.

Wheel layout (looking from above):
  Wheel 1: angle = -30°  (rpy -2.8521 in URDF)
  Wheel 2: angle = 90°   (rpy  3.0028 in URDF)
  Wheel 3: angle = 210°  (rpy -2.8751 in URDF)

Inverse kinematics for each wheel i at angle alpha_i:
  omega_i = (-sin(alpha_i)*vx + cos(alpha_i)*vy + rk*wz) / rw

Where:
  vx  = linear velocity x (m/s)
  vy  = linear velocity y (m/s)
  wz  = angular velocity z (rad/s)
  rk  = distance from robot centre to wheel contact (m)
  rw  = wheel radius (m)

Publishes to /ballbot_velocity_controller/commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class BallbotOmniController(Node):

    def __init__(self):
        super().__init__('ballbot_omni_controller')

        # Parameters — tune these to match your physical robot
        self.declare_parameter('wheel_radius', 0.05)   # rw (metres)
        self.declare_parameter('robot_radius', 0.125)  # rk (metres)
        
        self.rw = self.get_parameter('wheel_radius').value
        self.rk = self.get_parameter('robot_radius').value

        # Wheel angles in radians derived from URDF joint origins:
        # wheel1 rpy -2.8521 → contact angle ≈ -30°  = -π/6
        # wheel2 rpy  3.0028 → contact angle ≈  90°  =  π/2
        # wheel3 rpy -2.8751 → contact angle ≈ 210°  =  7π/6
        self.wheel_angles = [
            math.radians(-30),   # wheel1
            math.radians(90),    # wheel2
            math.radians(210),   # wheel3
        ]

        # Subscriber: receives Nav2 MPPI / joystick / keyboard commands
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher: sends wheel velocities to ros2_control
        self.vel_pub = self.create_publisher(
            Float64MultiArray,
            '/ballbot_velocity_controller/commands',
            10)

        self.get_logger().info(
            f'Ballbot omni controller ready | rw={self.rw} rk={self.rk}')

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        wheel_velocities = Float64MultiArray()
        w_list = []

        for alpha in self.wheel_angles:
            omega = (
                -math.sin(alpha) * vx
                + math.cos(alpha) * vy
                + self.rk * wz
            ) / self.rw
            w_list.append(omega)

        wheel_velocities.data = w_list
        self.vel_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = BallbotOmniController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

