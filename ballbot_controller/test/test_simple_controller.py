#!/usr/bin/env python3
"""
Unit tests for the SimpleController differential drive kinematics.

Differential drive forward model:
    v = r/2 * (w_right + w_left)
    w = r/L * (w_right - w_left)

Inverse (what SimpleController computes in velCallback):
    w_right = (2*v + w*L) / (2*r)
    w_left  = (2*v - w*L) / (2*r)

SimpleController publishes Float64MultiArray with data = [w_left, w_right],
matching simple_velocity_controller joints: [wheel_left_joint, wheel_right_joint].
"""
import unittest

import rclpy
from geometry_msgs.msg import TwistStamped

from ballbot_controller.simple_controller import SimpleController


def expected_wheel_speeds(v, w, r, sep):
    w_right = (2.0 * v + w * sep) / (2.0 * r)
    w_left = (2.0 * v - w * sep) / (2.0 * r)
    return w_left, w_right


def make_twist(v, w):
    msg = TwistStamped()
    msg.twist.linear.x = float(v)
    msg.twist.angular.z = float(w)
    return msg


class TestSimpleController(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = SimpleController()
        self.r = self.node.wheel_radius_
        self.sep = self.node.wheel_separation_
        # Capture published wheel commands instead of sending them to DDS
        self.published = []
        self.node.wheel_cmd_pub_.publish = lambda m: self.published.append(m)

    def tearDown(self):
        self.node.destroy_node()

    def _assert_wheels(self, v, w):
        self.published.clear()
        self.node.velCallback(make_twist(v, w))
        self.assertEqual(len(self.published), 1,
                         "velCallback must publish exactly one message")
        data = self.published[0].data
        self.assertEqual(len(data), 2,
                         "wheel command must contain [left, right]")
        exp_left, exp_right = expected_wheel_speeds(v, w, self.r, self.sep)
        self.assertAlmostEqual(data[0], exp_left, places=6,
                               msg=f"left wheel: got {data[0]}, expected {exp_left}")
        self.assertAlmostEqual(data[1], exp_right, places=6,
                               msg=f"right wheel: got {data[1]}, expected {exp_right}")
        return data

    def test_parameters_loaded(self):
        self.assertGreater(self.r, 0.0)
        self.assertGreater(self.sep, 0.0)

    def test_zero_velocity(self):
        left, right = self._assert_wheels(0.0, 0.0)
        self.assertAlmostEqual(left, 0.0, places=9)
        self.assertAlmostEqual(right, 0.0, places=9)

    def test_pure_forward(self):
        """Pure linear motion -> both wheels equal, positive, = v/r."""
        data = self._assert_wheels(1.0, 0.0)
        self.assertAlmostEqual(data[0], data[1], places=6)
        self.assertGreater(data[0], 0.0)
        self.assertAlmostEqual(data[0], 1.0 / self.r, places=6)

    def test_pure_backward(self):
        data = self._assert_wheels(-0.5, 0.0)
        self.assertAlmostEqual(data[0], data[1], places=6)
        self.assertLess(data[0], 0.0)

    def test_pure_rotation_ccw(self):
        """Positive angular.z (CCW) -> right wheel forward, left wheel backward,
        equal magnitude."""
        data = self._assert_wheels(0.0, 1.0)
        self.assertAlmostEqual(data[0], -data[1], places=6)
        self.assertGreater(data[1], 0.0)  # right wheel forward
        self.assertLess(data[0], 0.0)     # left wheel backward

    def test_pure_rotation_cw(self):
        data = self._assert_wheels(0.0, -1.0)
        self.assertGreater(data[0], 0.0)  # left wheel forward
        self.assertLess(data[1], 0.0)     # right wheel backward

    def test_combined_motion(self):
        self._assert_wheels(0.5, 0.3)
        self._assert_wheels(0.2, -0.8)
        self._assert_wheels(-0.3, 0.4)

    def test_forward_kinematics_roundtrip(self):
        """Feed computed wheel speeds back through the forward model and
        recover the original (v, w)."""
        v_in, w_in = 0.4, -0.6
        self.published.clear()
        self.node.velCallback(make_twist(v_in, w_in))
        w_left, w_right = self.published[0].data
        v_out = self.r / 2.0 * (w_right + w_left)
        w_out = self.r / self.sep * (w_right - w_left)
        self.assertAlmostEqual(v_out, v_in, places=6)
        self.assertAlmostEqual(w_out, w_in, places=6)


if __name__ == "__main__":
    unittest.main()
