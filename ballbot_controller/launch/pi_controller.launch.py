#!/usr/bin/env python3
"""
pi_controller.launch.py  —  ballbot_controller package
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Terminal 1 — ALWAYS run this first.

Testing (no STM32, no real movement):
  ros2 launch ballbot_controller pi_controller.launch.py

Real robot (STM32 connected on /dev/ttyAMA2):
  ros2 launch ballbot_controller pi_controller.launch.py \
    use_fake_odom:=false

Complete data flow:
  PS4 L1+stick → /cmd_vel_joy ──┐
  keyboard     → /key_vel    ───┤→ twist_mux → /cmd_vel
  Nav2 MPPI    → /cmd_vel_nav ──┘         ↓
                                    stm32_bridge
                                    ↙            ↘
                              TX: 15-byte      RX: 27-byte
                              velocity       odom packet
                              to STM32       from STM32
                                                  ↓
                                            /odom + TF
                                         odom→base_footprint
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gp_pkg   = get_package_share_directory("gp_description")
    ctrl_pkg = get_package_share_directory("ballbot_controller")

    # ── Arguments ─────────────────────────────────────────────────
    use_fake_odom_arg = DeclareLaunchArgument(
        "use_fake_odom",
        default_value="true",
        description=(
            "true  = fake_odom.py — integrate /cmd_vel → /odom (no STM32 needed) "
            "false = stm32_bridge — real odom from STM32 USART2 via /dev/ttyAMA2"
        )
    )
    use_fake_odom = LaunchConfiguration("use_fake_odom")

    # ── Robot description ─────────────────────────────────────────
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(
            gp_pkg, "urdf", "ballbot1.urdf.xacro")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_static_tf",
        arguments=[
            "--x",   "-0.007",
            "--y",   "0.126",
            "--z",   "0.7943",
            "--roll",  "0",
            "--pitch", "0",
            "--yaw",   "0",
            "--frame-id",       "base_link",
            "--child-frame-id", "laser_link",
        ],
        parameters=[{"use_sim_time": False}],
    )

    # ── LD06 Lidar on /dev/ttyS0 ──────────────────────────────────
    lidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="lidar_node",
        output="screen",
        parameters=[
            {"product_name":           "LDLiDAR_LD06"},
            {"topic_name":             "scan"},
            {"frame_id":               "laser_link"},
            {"port_name":              "/dev/ttyS0"},
            {"port_baudrate":          230400},
            {"laser_scan_dir":         True},
            {"enable_angle_crop_func": False},
        ],
    )


    fake_odom = GroupAction(
        condition=IfCondition(use_fake_odom),
        actions=[
            Node(
                package="ballbot_controller",
                executable="fake_odom.py",
                name="fake_odom",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ],
    )

    # ── stm32_bridge (use_fake_odom:=false) ───────────────────────
    # Real odometry from STM32 EKF via USART2 on /dev/ttyAMA2.
    # Lifecycle node: UNCONFIGURED → INACTIVE → ACTIVE
    # The lifecycle_manager below activates it automatically.
    #
    # TX path: /cmd_vel → stm32_bridge → 15-byte packet → STM32 USART2
    # RX path: STM32 USART2 → 27-byte packet → stm32_bridge → /odom + TF
    #
    # Wire: Pi pin27 GPIO0 TXD → STM32 PA3 USART2_RX
    #       Pi pin28 GPIO1 RXD → STM32 PA2 USART2_TX
    #       Pi pin25 GND       → STM32 GND
    stm32_odom = GroupAction(
        condition=UnlessCondition(use_fake_odom),
        actions=[
            LifecycleNode(
                package="ballbot_controller",
                executable="stm32_bridge.py",
                name="ballbot_stm32_bridge",
                namespace="",
                output="screen",
                parameters=[{
                    "serial_port":     "/dev/ttyAMA2",
                    "baud_rate":       115200,
                    "tx_rate_hz":      50.0,
                    "cmd_vel_timeout": 0.5,
                    "max_vx":          0.5,
                    "max_vy":          0.5,
                    "max_yaw_rate":    1.0,
                    "odom_frame":      "odom",
                    "base_frame":      "base_footprint",
                    "use_sim_time":    False,
                }],
            ),
            # Manages the lifecycle: configure → activate automatically
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_stm32",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "autostart":    True,
                    "node_names":   ["ballbot_stm32_bridge"],
                }],
            ),
        ],
    )

    # ══════════════════════════════════════════════════════════════
    # VELOCITY INPUT — PS4, keyboard, Nav2 all go through twist_mux
    # ══════════════════════════════════════════════════════════════

    # PS4 DualShock4 — hold L1 (button 4) to enable motion
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            os.path.join(ctrl_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": False},
        ],
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        output="screen",
        parameters=[
            os.path.join(ctrl_pkg, "config", "joy_teleop_gp.yaml"),
            {"use_sim_time": False},
        ],
    )

    # twist_mux merges all velocity sources by priority:
    #   /cmd_vel_joy  priority 100  PS4 controller
    #   /key_vel      priority 50   keyboard teleop
    #   /cmd_vel_nav  priority 10   Nav2 MPPI autonomous
    # Output /cmd_vel_out is remapped to /cmd_vel
    # stm32_bridge subscribes to /cmd_vel and sends to STM32
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            os.path.join(ctrl_pkg, "config", "twist_mux_topic_gp.yaml"),
            os.path.join(ctrl_pkg, "config", "twist_mux_locks.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    return LaunchDescription([
        use_fake_odom_arg,
        robot_state_publisher,
        joint_state_publisher,
        laser_tf,
        lidar_node,
        fake_odom,
        stm32_odom,
        joy_node,
        joy_teleop,
        twist_mux,
    ])
