#!/usr/bin/env python3
# display.launch.py
# Launches robot_state_publisher + joint_state_publisher + RViz2
# for URDF visualization without Gazebo.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("gp_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg, "urdf", "ballbot1.urdf.xacro"),
        description="Absolute path to robot URDF xacro file",
    )

    # Expand xacro → URDF string at launch time
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    # Publishes TF transforms from the URDF joint tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }],
    )

    # Publishes zero joint states so RViz shows the robot in its
    # default pose. Replace with joint_state_publisher_gui if you
    # want interactive sliders for the wheel joints.
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # RViz2 with a saved config (create one interactively and save it)
    rviz_config = os.path.join(pkg, "rviz", "display.rviz")
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
    # )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher,
        # rviz_node,
    ])
