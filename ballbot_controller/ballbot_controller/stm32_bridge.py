#!/usr/bin/env python3
"""
stm32_bridge.py  —  ballbot_controller package
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Lifecycle node: Pi ↔ STM32 serial bridge.

TX packet  (Pi → STM32, 16 bytes, 50 Hz):
  [0xAA][0x55][mode:u8][vx:f32 LE][vy:f32 LE][yaw:f32 LE][XOR]
   0     1     2        3-6        7-10        11-14        15
  XOR covers bytes 2..14  (mode + 3 floats = 13 bytes)

RX packet  (STM32 → Pi, 27 bytes, 20 Hz — Modes 2 & 3 only):
  [0xBB][0x66][x:f32][y:f32][θ:f32][vx:f32][vy:f32][yaw:f32][XOR]
   0     1     2-5    6-9   10-13  14-17   18-21   22-25     26
  XOR covers bytes 2..25  (6 floats = 24 bytes)

Mode table:
  0  STANDBY     — motors off
  1  BALANCE     — stationary balance, velocity commands ignored by STM32
  2  TELEOP      — LQI balance + /cmd_vel commands
  3  FLOOR_DRIVE — direct inverse-kinematics, no balance (robot on casters)

Switch mode at runtime:
  ros2 topic pub --once /ballbot/mode std_msgs/msg/UInt8 '{data: 3}'

Physical wiring:
  Pi GPIO0 (TXD, pin 27)  →  STM32 PA3 (USART2_RX)
  Pi GPIO1 (RXD, pin 28)  →  STM32 PA2 (USART2_TX)
  Pi GND   (pin 25)       →  STM32 GND
  ALL lines are 3.3 V — no level shifter needed on F411.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, String

from tf2_ros import TransformBroadcaster

import serial
import struct
import threading
import math

# ── Packet constants ──────────────────────────────────────────────────────
TX_SYNC1 = 0xAA
TX_SYNC2 = 0x55
TX_SIZE  = 16   # [sync(2)][mode(1)][3×f32(12)][xor(1)]

RX_SYNC1 = 0xBB
RX_SYNC2 = 0x66
RX_SIZE  = 27   # [sync(2)][6×f32(24)][xor(1)]

MODE_NAMES = {0: 'STANDBY', 1: 'BALANCE', 2: 'TELEOP', 3: 'FLOOR_DRIVE'}


def xor_checksum(data: bytes) -> int:
    """XOR of all bytes in data."""
    c = 0
    for b in data:
        c ^= b
    return c


class BallbotSTM32Bridge(LifecycleNode):

    def __init__(self):
        super().__init__('ballbot_stm32_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('serial_port',     '/dev/ttyAMA2')
        self.declare_parameter('baud_rate',       115200)
        self.declare_parameter('tx_rate_hz',      50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('max_vx',          0.5)
        self.declare_parameter('max_vy',          0.5)
        self.declare_parameter('max_yaw_rate',    1.0)
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('base_frame',      'base_footprint')
        self.declare_parameter('default_mode',    1)   # 1=BALANCE on startup

        # ── Internal state ─────────────────────────────────────────────────
        self._lock         = threading.Lock()
        self._cmd_vx       = 0.0
        self._cmd_vy       = 0.0
        self._cmd_yaw      = 0.0
        self._last_cmd     = None        # set in on_configure
        self._current_mode = 1          # safe default until parameter loads

        self._serial     = None
        self._tx_timer   = None
        self._rx_thread  = None
        self._rx_running = False

        self._cmd_sub    = None
        self._mode_sub   = None
        self._odom_pub   = None
        self._mode_pub   = None
        self._tf_bcast   = None

    # ── Lifecycle callbacks ────────────────────────────────────────────────

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

        # Load default_mode from parameter
        self._current_mode = int(self.get_parameter('default_mode').value)

        # /cmd_vel → velocity for Modes 2 and 3
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        # /ballbot/mode → switch between modes 0/1/2/3
        self._mode_sub = self.create_subscription(
            UInt8, '/ballbot/mode', self._mode_cb, 10)

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._mode_pub = self.create_publisher(String, '/ballbot/mode_status', 10)
        self._tf_bcast = TransformBroadcaster(self)
        self._last_cmd = self.get_clock().now()

        name = MODE_NAMES.get(self._current_mode, '?')
        self.get_logger().info(
            f'Configured — default mode={self._current_mode} ({name})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._rx_running = True
        self._rx_thread  = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        rate = self.get_parameter('tx_rate_hz').value
        self._tx_timer = self.create_timer(1.0 / rate, self._tx_tick)

        self.get_logger().info(
            f'Active — TX {rate:.0f} Hz, RX thread running, '
            f'mode={self._current_mode} ({MODE_NAMES.get(self._current_mode, "?")})')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        with self._lock:
            self._cmd_vx = self._cmd_vy = self._cmd_yaw = 0.0
        self._send_cmd_packet(0, 0.0, 0.0, 0.0)   # STANDBY + zero vel

        if self._tx_timer:
            self._tx_timer.cancel()
            self._tx_timer = None

        self._rx_running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        self._rx_thread = None

        self.get_logger().info('Deactivated — velocity zeroed')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._serial and self._serial.is_open:
            self._serial.close()
        for sub in (self._cmd_sub, self._mode_sub):
            if sub:
                self.destroy_subscription(sub)
        for pub in (self._odom_pub, self._mode_pub):
            if pub:
                self.destroy_publisher(pub)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        try:
            self._send_cmd_packet(0, 0.0, 0.0, 0.0)   # safe: STANDBY
        except Exception:
            pass
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error('Error state — safe shutdown')
        return self.on_shutdown(state)

    # ── Subscribers ────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        """Buffer /cmd_vel with velocity clamping.  Used by Modes 2 and 3."""
        max_vx  = self.get_parameter('max_vx').value
        max_vy  = self.get_parameter('max_vy').value
        max_yaw = self.get_parameter('max_yaw_rate').value
        with self._lock:
            self._cmd_vx   = max(-max_vx,  min(max_vx,  msg.linear.x))
            self._cmd_vy   = max(-max_vy,  min(max_vy,  msg.linear.y))
            self._cmd_yaw  = max(-max_yaw, min(max_yaw, msg.angular.z))
            self._last_cmd = self.get_clock().now()

    def _mode_cb(self, msg: UInt8):
        """
        Switch mode via /ballbot/mode (std_msgs/UInt8).

        Valid values:
          0  STANDBY     — motors off
          1  BALANCE     — stationary balance (default on startup)
          2  TELEOP      — balance + /cmd_vel (robot on ball)
          3  FLOOR_DRIVE — direct drive, no balancing (robot on casters)
        """
        new_mode = int(msg.data)
        if new_mode not in (0, 1, 2, 3):
            self.get_logger().warn(
                f'Invalid mode {new_mode} — valid: 0(STANDBY) 1(BALANCE) '
                f'2(TELEOP) 3(FLOOR_DRIVE)')
            return
        with self._lock:
            old_mode           = self._current_mode
            self._current_mode = new_mode
            if new_mode != old_mode:
                self._cmd_vx = self._cmd_vy = self._cmd_yaw = 0.0

        name = MODE_NAMES.get(new_mode, '?')
        self.get_logger().info(
            f'Mode {old_mode}({MODE_NAMES.get(old_mode,"?")}) → {new_mode}({name})')

        if self._mode_pub:
            s = String()
            s.data = name
            self._mode_pub.publish(s)

    # ── TX ─────────────────────────────────────────────────────────────────

    def _tx_tick(self):
        """Periodic TX at tx_rate_hz (default 50 Hz)."""
        timeout = self.get_parameter('cmd_vel_timeout').value
        with self._lock:
            mode = self._current_mode
            elapsed = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
            if elapsed > timeout:
                vx = vy = yaw = 0.0
            else:
                vx, vy, yaw = self._cmd_vx, self._cmd_vy, self._cmd_yaw
        self._send_cmd_packet(mode, vx, vy, yaw)

    def _send_cmd_packet(self, mode: int, vx: float,
                          vy: float, yaw: float):
        """
        Build and transmit the 16-byte command packet.

        Layout (matches USART2_IRQHandler_Extension in main.c):
          [0xAA][0x55][mode:u8][vx:f32 LE][vy:f32 LE][yaw:f32 LE][XOR]
           0     1     2        3-6        7-10        11-14        15
          XOR covers bytes 2..14 (mode byte + 3 floats = 13 bytes).
        """
        if not self._serial or not self._serial.is_open:
            return
        try:
            payload  = struct.pack('<B3f', int(mode) & 0xFF, vx, vy, yaw)
            checksum = xor_checksum(payload)
            packet   = bytes([TX_SYNC1, TX_SYNC2]) + payload + bytes([checksum])
            self._serial.write(packet)
        except serial.SerialException as e:
            self.get_logger().warn(f'TX error: {e}', throttle_duration_sec=2.0)

    # ── RX ─────────────────────────────────────────────────────────────────

    def _rx_loop(self):
        """
        Dedicated RX thread.
        Hunts for [0xBB 0x66] sync then reads the full 27-byte odom packet.
        The STM32 only sends binary odom in Modes 2 (TELEOP) and 3 (FLOOR_DRIVE).
        In Mode 1 it sends ASCII debug strings — the sync search skips these
        safely because 0xBB (187) never appears in standard ASCII output.
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

                # Hunt for sync header
                while len(buf) >= 2:
                    if buf[0] == RX_SYNC1 and buf[1] == RX_SYNC2:
                        break
                    buf.pop(0)

                if len(buf) < RX_SIZE:
                    continue

                packet   = bytes(buf[:RX_SIZE])
                payload  = packet[2:26]   # 24 bytes = 6 floats
                checksum = packet[26]

                if xor_checksum(payload) != checksum:
                    self.get_logger().warn(
                        'Bad checksum on RX packet — discarding',
                        throttle_duration_sec=1.0)
                    buf.pop(0)
                    continue

                x, y, theta, vx, vy, yaw_rate = struct.unpack('<6f', payload)
                self._publish_odom(x, y, theta, vx, vy, yaw_rate)
                del buf[:RX_SIZE]

            except serial.SerialException as e:
                self.get_logger().error(f'RX serial: {e}',
                                        throttle_duration_sec=2.0)
                break
            except Exception as e:
                self.get_logger().error(f'RX error: {e}',
                                        throttle_duration_sec=2.0)

    def _publish_odom(self, x: float, y: float, theta: float,
                       vx: float, vy: float, yaw_rate: float):
        """Publish nav_msgs/Odometry + TF from STM32 dead-reckoning state."""
        now        = self.get_clock().now().to_msg()
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        # TF: odom → base_footprint
        tf = TransformStamped()
        tf.header.stamp     = now
        tf.header.frame_id  = odom_frame
        tf.child_frame_id   = base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self._tf_bcast.sendTransform(tf)

        # Odometry message
        odom = Odometry()
        odom.header.stamp     = now
        odom.header.frame_id  = odom_frame
        odom.child_frame_id   = base_frame
        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = yaw_rate

        # Covariance (tune once robot is running — these are starting points)
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[14] = 1e6    # z (flat ground)
        odom.pose.covariance[21] = 1e6    # roll (locked by balance)
        odom.pose.covariance[28] = 1e6    # pitch (locked by balance)
        odom.pose.covariance[35] = 0.05   # yaw

        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.01
        odom.twist.covariance[35] = 0.05

        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node     = BallbotSTM32Bridge()
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
