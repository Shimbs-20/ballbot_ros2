#!/usr/bin/env python3
"""
hardware.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 1 — run this first, always.
Starts only the hardware drivers. ~400 MB RAM.

  ros2 launch ballbot_controller hardware.launch.py

After this is running and lidar is publishing /scan,
open Terminal 2 and run nav.launch.py.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gp_pkg   = get_package_share_directory("gp_description")
    ctrl_pkg = get_package_share_directory("ballbot_controller")

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(
            gp_pkg, "urdf", "ballbot1.urdf.xacro")]),
        value_type=str,
    )

    # ── Robot description ─────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    # ── Static TF: base_link → laser_link ─────────────────────────
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

    # ── LD06 Lidar ────────────────────────────────────────────────
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

    # ── Odometry ──────────────────────────────────────────────────
    # fake_odom integrates /cmd_vel → /odom + TF odom→base_footprint
    # Replace with stm32_bridge.py when STM32 sends real odom packets
    fake_odom = Node(
        package="ballbot_controller",
        executable="fake_odom.py",
        name="fake_odom",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # ── STM32 bridge (velocity commands to hardware) ───────────────
    stm32_bridge = Node(
        package="ballbot_controller",
        executable="stm32_bridge.py",
        name="ballbot_stm32_bridge",
        output="screen",
        parameters=[{
            "serial_port":     "/dev/ttyAMA2",
            "baud_rate":       115200,
            "tx_rate_hz":      50.0,
            "cmd_vel_timeout": 0.5,
            "max_vx":          0.5,
            "max_vy":          0.5,
            "max_yaw_rate":    1.0,
            "use_sim_time":    False,
        }],
    )

    # ── PS4 Joystick + twist_mux ──────────────────────────────────
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            os.path.join(ctrl_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": False},
        ],
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[
            os.path.join(ctrl_pkg, "config", "joy_teleop_gp.yaml"),
            {"use_sim_time": False},
        ],
    )

    # twist_mux priorities:
    #   PS4      /cmd_vel_joy  priority 100
    #   keyboard /key_vel      priority 50
    #   Nav2     /cmd_vel_nav  priority 10
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(ctrl_pkg, "config", "twist_mux_topic_gp.yaml"),
            os.path.join(ctrl_pkg, "config", "twist_mux_locks.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("/cmd_vel_out", "/cmd_vel")],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        laser_tf,
        lidar_node,
        fake_odom,
        stm32_bridge,
        joy_node,
        joy_teleop,
        twist_mux,
    ])
