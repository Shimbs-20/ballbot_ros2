#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    gp_pkg   = get_package_share_directory("gp_description")
    ctrl_pkg = get_package_share_directory("ballbot_controller")

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(
            gp_pkg, "urdf", "ballbot1.urdf.xacro")]),
        value_type=str,
    )

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


    fake_odom = Node(
        package="ballbot_controller",
        executable="fake_odom.py",
        name="fake_odom",
        output="screen",
    )

    camera = Node(
        package="camera_pi",
        executable="camer_node",
        name="Pi_stream",
        output="screen",
        parameters=[{
            "width": 640,
            "height": 480,
            "framerate": 10
        }],
    )

    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen"
    )

    stm32_tx = Node(
        package="ballbot_controller",
        executable="stm32_bridge.py",
        name="ballbot_stm32_bridge",
        output="screen",
        parameters=[{
            "serial_port":     "/dev/ttyACM0",
            "baud_rate":       115200,
            "tx_rate_hz":      50.0,
            "cmd_vel_timeout": 0.5,
            "max_vx":          0.5,
            "max_vy":          0.5,
            "max_yaw_rate":    1.0,
        }],
    )

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

    # slam = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("ballbot_mapping"),
    #         "launch",
    #         "slam.launch.py"
    #     ),
    #     # condition=IfCondition(use_slam)
    # )

    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml"
            )
        )
    )


    core_hardware = [
        robot_state_publisher,
        joint_state_publisher,
        lidar_node,
        fake_odom,
        stm32_tx,
        joy_node,
        joy_teleop,
        twist_mux,
    ]

    # 2. Wait 5 seconds, then boot Camera and Web Interfaces
    delayed_web_nodes = TimerAction(
        period=5.0,
        actions=[camera, web_video_server, rosbridge_server]
    )

    # 3. Wait 10 seconds, then boot SLAM (gives LiDAR & TF time to stabilize)
    # delayed_slam = TimerAction(
    #     period=10.0,
    #     actions=[slam]
    # )

    return LaunchDescription([
        use_slam_arg,
        *core_hardware,
        delayed_web_nodes,
        # delayed_slam
    ])
