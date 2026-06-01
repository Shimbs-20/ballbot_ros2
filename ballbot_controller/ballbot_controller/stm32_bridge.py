#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster

import serial
import struct
import threading
import math

TX_SYNC1    = 0xAA
TX_SYNC2    = 0x55
TX_SIZE     = 15

RX_SYNC1    = 0xBB
RX_SYNC2    = 0x66
RX_SIZE     = 27


def xor_checksum(data: bytes) -> int:

    c = 0
    for b in data:
        c ^= b
    return c


class BallbotSTM32Bridge(LifecycleNode):

    def __init__(self):
        super().__init__('ballbot_stm32_bridge')

        self.declare_parameter('serial_port',     '/dev/ttyACM0')
        self.declare_parameter('baud_rate',       115200)
        self.declare_parameter('tx_rate_hz',      50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.declare_parameter('max_vx',       0.5)
        self.declare_parameter('max_vy',       0.5)
        self.declare_parameter('max_yaw_rate', 1.0)

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self._lock      = threading.Lock()
        self._cmd_vx    = 0.0
        self._cmd_vy    = 0.0
        self._cmd_yaw   = 0.0
        self._last_cmd  = None

        self._serial    = None
        self._tx_timer  = None
        self._rx_thread = None
        self._rx_running = False

        self._cmd_sub   = None
        self._odom_pub  = None
        self._tf_bcast  = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        try:
            self._serial = serial.Serial(
                port, baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
            )
            self.get_logger().info(f'Serial opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial {port}: {e}')
            return TransitionCallbackReturn.FAILURE

        self._cmd_sub  = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        self._odom_pub = self.create_publisher(
            Odometry, 'odom', 10)

        self._tf_bcast = TransformBroadcaster(self)

        self._last_cmd = self.get_clock().now()

        self.get_logger().info('Configured — serial ready, waiting for activate')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._rx_running = True
        self._rx_thread  = threading.Thread(
            target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        rate = self.get_parameter('tx_rate_hz').value
        self._tx_timer = self.create_timer(1.0 / rate, self._tx_tick)

        self.get_logger().info(f'Active — TX at {rate:.0f} Hz, RX thread running')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        with self._lock:
            self._cmd_vx = self._cmd_vy = self._cmd_yaw = 0.0
        self._send_cmd_packet(0.0, 0.0, 0.0)

        if self._tx_timer:
            self._tx_timer.cancel()
            self._tx_timer = None

        self._rx_running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self._rx_thread = None

        self.get_logger().info('Deactivated — velocity zeroed, threads stopped')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._serial and self._serial.is_open:
            self._serial.close()
        if self._cmd_sub:
            self.destroy_subscription(self._cmd_sub)
        if self._odom_pub:
            self.destroy_publisher(self._odom_pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        try:
            self._send_cmd_packet(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error('Error state — attempting safe shutdown')
        return self.on_shutdown(state)



    def _cmd_vel_cb(self, msg: Twist):
        """Buffer incoming cmd_vel with velocity clamping."""
        max_vx  = self.get_parameter('max_vx').value
        max_vy  = self.get_parameter('max_vy').value
        max_yaw = self.get_parameter('max_yaw_rate').value

        with self._lock:
            self._cmd_vx  = max(-max_vx,  min(max_vx,  msg.linear.x))
            self._cmd_vy  = max(-max_vy,  min(max_vy,  msg.linear.y))
            self._cmd_yaw = max(-max_yaw, min(max_yaw, msg.angular.z))
            self._last_cmd = self.get_clock().now()

    def _tx_tick(self):
        timeout = self.get_parameter('cmd_vel_timeout').value

        with self._lock:
            elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
            if elapsed > timeout:
                vx = vy = yaw = 0.0
            else:
                vx, vy, yaw = self._cmd_vx, self._cmd_vy, self._cmd_yaw

        self._send_cmd_packet(vx, vy, yaw)

    def _send_cmd_packet(self, vx: float, vy: float, yaw: float):
        if not self._serial or not self._serial.is_open:
            return
        try:
            payload  = struct.pack('<3f', vx, vy, yaw)   # 12 bytes, little-endian
            checksum = xor_checksum(payload)
            packet   = bytes([TX_SYNC1, TX_SYNC2]) + payload + bytes([checksum])
            self._serial.write(packet)                     # 15 bytes total
        except serial.SerialException as e:
            self.get_logger().warn(f'TX error: {e}', throttle_duration_sec=2.0)

    # =========================================================================
    # RX — receive odometry from STM32
    # =========================================================================

    def _rx_loop(self):
        """
        Runs in dedicated thread. Continuously reads bytes from serial,
        hunts for RX_SYNC1/RX_SYNC2, then reads full 27-byte packet.
        """
        buf = bytearray()

        while self._rx_running and rclpy.ok():
            try:
                if not self._serial or not self._serial.is_open:
                    break

                byte = self._serial.read(1)
                if not byte:
                    continue
                buf += byte

                # ── Frame sync: search for [0xBB][0x66] ──────────
                while len(buf) >= 2:
                    if buf[0] == RX_SYNC1 and buf[1] == RX_SYNC2:
                        break
                    buf.pop(0)   # discard leading byte, keep hunting

                # Wait until we have the full packet
                if len(buf) < RX_SIZE:
                    continue
                packet   = bytes(buf[:RX_SIZE])
                payload  = packet[2:26]   
                checksum = packet[26]

                if xor_checksum(payload) != checksum:
                    self.get_logger().warn(
                        'Bad checksum on RX packet — discarding',
                        throttle_duration_sec=1.0)
                    buf.pop(0)
                    continue

                x, y, theta, vx, vy, yaw_rate = struct.unpack('<6f', payload)

                self._publish_odom(x, y, theta, vx, vy, yaw_rate)

                # Consume the packet from buffer
                del buf[:RX_SIZE]

            except serial.SerialException as e:
                self.get_logger().error(
                    f'RX serial error: {e}', throttle_duration_sec=2.0)
                break
            except Exception as e:
                self.get_logger().error(
                    f'RX unexpected error: {e}', throttle_duration_sec=2.0)

    def _publish_odom(self, x, y, theta, vx, vy, yaw_rate):
        """
        Convert parsed STM32 EKF state to ROS Odometry + TF.
        Theta from STM32 is the robot's heading in radians.
        vx/vy are body-frame velocities.
        """
        now = self.get_clock().now().to_msg()

        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value

        # Quaternion from heading angle (z-axis rotation only)
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        # ── Publish TF: odom → base_footprint ────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now
        tf_msg.header.frame_id = odom_frame
        tf_msg.child_frame_id  = base_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z    = qz
        tf_msg.transform.rotation.w    = qw
        self._tf_bcast.sendTransform(tf_msg)

        # ── Publish Odometry ──────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = odom_frame
        odom.child_frame_id  = base_frame

        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocities in base_frame
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = yaw_rate

        # Covariance — tune once robot is running
        # Diagonal: [x, y, z, roll, pitch, yaw]
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[14] = 1e6    # z (robot stays flat)
        odom.pose.covariance[21] = 1e6    # roll (locked)
        odom.pose.covariance[28] = 1e6    # pitch (locked)
        odom.pose.covariance[35] = 0.05   # yaw

        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.01
        odom.twist.covariance[35] = 0.05

        self._odom_pub.publish(odom)



def main(args=None):
    rclpy.init(args=args)

    node = BallbotSTM32Bridge()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
