#!/usr/bin/env python3
"""
fake_odom.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Publishes fake odometry by integrating /cmd_vel over time.

USE THIS ONLY until your STM32 sends real odom packets.
Remove it the moment your ballbot_stm32_bridge receives
real data — fake odom drifts and will confuse SLAM
on a long session, but it is good enough for:
  - Verifying RViz visualization works
  - Testing SLAM map building in a small area
  - Verifying Nav2 goal sending works end-to-end

Publishes:
  /odom            nav_msgs/Odometry
  TF: odom → base_footprint

Subscribes:
  /cmd_vel         geometry_msgs/Twist
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math


class FakeOdom(Node):

    def __init__(self):
        super().__init__('fake_odom')

        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0

        self._vx  = 0.0
        self._vy  = 0.0
        self._wz  = 0.0

        self._last_time = self.get_clock().now()

        self._tf_bcast = TransformBroadcaster(self)

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_cb, 10)

        # Update at 50 Hz — matches STM32 tx rate
        self._timer = self.create_timer(0.02, self._update)

        self.get_logger().info(
            'Fake odometry running — remove when STM32 sends real odom')

    def _cmd_cb(self, msg: Twist):
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._wz = msg.angular.z

    def _update(self):
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0.0 or dt > 0.5:
            return


        cos_t = math.cos(self._theta)
        sin_t = math.sin(self._theta)

        self._x     += (cos_t * self._vx - sin_t * self._vy) * dt
        self._y     += (sin_t * self._vx + cos_t * self._vy) * dt
        self._theta += self._wz * dt

        # Wrap theta to [-pi, pi]
        self._theta = math.atan2(
            math.sin(self._theta), math.cos(self._theta))

        now_msg = now.to_msg()
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)

        tf = TransformStamped()
        tf.header.stamp    = now_msg
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_footprint'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self._tf_bcast.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp    = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = self._vx
        odom.twist.twist.linear.y    = self._vy
        odom.twist.twist.angular.z   = self._wz

        odom.pose.covariance[0]  = 0.05
        odom.pose.covariance[7]  = 0.05
        odom.pose.covariance[35] = 0.1
        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
