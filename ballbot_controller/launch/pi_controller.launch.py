#!/usr/bin/env python3
"""
pi_controller.launch.py  —  ballbot_controller package
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 1 — ALWAYS run this first.

── Development / testing (no STM32 needed) ──────────────────
  ros2 launch ballbot_controller pi_controller.launch.py

── Real robot — stationary balance only ─────────────────────
  ros2 launch ballbot_controller pi_controller.launch.py \
    use_fake_odom:=false robot_mode:=1

── Real robot — teleop (balance + drive on ball) ────────────
  ros2 launch ballbot_controller pi_controller.launch.py \
    use_fake_odom:=false robot_mode:=2

── SLAM mapping on floor (casters, no balancing) ────────────
  ros2 launch ballbot_controller pi_controller.launch.py \
    use_fake_odom:=false robot_mode:=3

── Switch mode without restarting ───────────────────────────
  ros2 topic pub --once /ballbot/mode std_msgs/msg/UInt8 '{data: 2}'

TX 16-byte [0xAA][0x55][mode][vx][vy][yaw][XOR]  @50 Hz
RX 27-byte [0xBB][0x66][x][y][θ][vx][vy][yaw][XOR] @20 Hz (Modes 2+3)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             ExecuteProcess, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gp_pkg   = get_package_share_directory("gp_description")
    ctrl_pkg = get_package_share_directory("ballbot_controller")

    # ── Arguments ─────────────────────────────────────────────────────────
    use_fake_odom_arg = DeclareLaunchArgument(
        "use_fake_odom",
        default_value="true",
        description=(
            "true  = fake_odom.py (integrate /cmd_vel → /odom, no STM32 needed)\n"
            "false = stm32_bridge  (real 27-byte binary odom from STM32)"
        )
    )

    robot_mode_arg = DeclareLaunchArgument(
        "robot_mode",
        default_value="1",
        description=(
            "STM32 starting mode: "
            "0=STANDBY, 1=BALANCE, 2=TELEOP, 3=FLOOR_DRIVE"
        )
    )

    use_fake_odom = LaunchConfiguration("use_fake_odom")
    robot_mode    = LaunchConfiguration("robot_mode")

    # ── Robot description ──────────────────────────────────────────────────
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

    # base_link → laser_link static TF
    # REMOVE this node if your URDF already publishes this transform —
    # having both causes "TF_REPEATED_DATA" warnings.
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

    # ── LD06 LiDAR on /dev/ttyS0 ──────────────────────────────────────────
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

    # ── Fake odometry (testing without STM32) ─────────────────────────────
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

    # ── STM32 bridge (real robot) ──────────────────────────────────────────
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
                    "default_mode":    robot_mode,
                    "max_vx":          0.5,
                    "max_vy":          0.5,
                    "max_yaw_rate":    1.0,
                    "odom_frame":      "odom",
                    "base_frame":      "base_footprint",
                    "use_sim_time":    False,
                }],
            ),
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
            # Belt-and-suspenders: re-send mode 5s after bridge activates
            # so STM32 gets the command even if it booted late.
            TimerAction(
                period=5.0,
                actions=[ExecuteProcess(
                    cmd=[
                        "bash", "-c",
                        ["ros2 topic pub --once /ballbot/mode "
                         "std_msgs/msg/UInt8 '{data: ", robot_mode, "}'"]
                    ],
                    output="screen",
                )],
            ),
        ],
    )

    # ── Velocity inputs ────────────────────────────────────────────────────
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

    # twist_mux priority: PS4(100) > keyboard(50) > Nav2(10)
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
        robot_mode_arg,
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
