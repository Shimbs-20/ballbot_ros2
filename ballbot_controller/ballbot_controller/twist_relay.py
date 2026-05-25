#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay")

        # twist_mux → /ballbot_controller/cmd_vel_unstamped (Twist)
        # this node  → /ballbot_controller/cmd_vel          (TwistStamped)
        # diff_drive_controller reads TwistStamped (use_stamped_vel: true)
        self.controller_sub = self.create_subscription(
            Twist,
            "/ballbot_controller/cmd_vel_unstamped",
            self.controller_twist_callback,
            10
        )
        self.controller_pub = self.create_publisher(
            TwistStamped,
            "/ballbot_controller/cmd_vel",
            10
        )

        # BUG FIX 15: Joy path removed entirely from Python version.
        # The joy_sub subscribed to /input_joy/cmd_vel_stamped (TwistStamped)
        # and joy_pub published to /input_joy/cmd_vel (Twist).
        # But joy_teleop.yaml already publishes Twist on /input_joy/cmd_vel.
        # This created TWO publishers with DIFFERENT types on the same topic
        # causing: "topic contains more than one type: [Twist, TwistStamped]"
        # joy_teleop → /input_joy/cmd_vel (Twist) → twist_mux directly.
        # No stripping relay needed anymore.

    def controller_twist_callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_footprint"
        twist_stamped.twist = msg
        self.controller_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
