#!/usr/bin/env python3
"""
nav2.launch.py  —  ballbot_navigation package
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Shelf testing (lidar enclosed, no real robot movement):
  ros2 launch ballbot_navigation nav2.launch.py \
    use_slam:=false map_name:=small_house shelf_test:=true

Real robot on floor:
  ros2 launch ballbot_navigation nav2.launch.py \
    use_slam:=false map_name:=small_house shelf_test:=false
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    map_pkg = get_package_share_directory("ballbot_mapping")
    nav_pkg = get_package_share_directory("ballbot_navigation")

    nav2_params       = os.path.join(nav_pkg, "config", "nav2_param.yaml")
    nav2_params_shelf = os.path.join(nav_pkg, "config", "nav2_param_shelf.yaml")
    amcl_config       = os.path.join(map_pkg, "config", "amcl.yaml")
    bt_xml            = os.path.join(nav_pkg, "behavior_trees", "ballbot_bt.xml")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false",
        description="true = SLAM | false = AMCL localization"
    )
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Map folder: small_house | small_warehouse"
    )
    shelf_test_arg = DeclareLaunchArgument(
        "shelf_test",
        default_value="true"
    )

    use_slam   = LaunchConfiguration("use_slam")
    map_name   = LaunchConfiguration("map_name")
    shelf_test = LaunchConfiguration("shelf_test")

    map_yaml_path = PathJoinSubstitution([
        map_pkg, "maps", map_name, "map.yaml"
    ])


    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time":  False,
            "yaml_filename": map_yaml_path,
        }],
        condition=UnlessCondition(use_slam),
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config,
            {
                "use_sim_time":     False,
                "set_initial_pose": True,
                "initial_pose.x":   0.0,
                "initial_pose.y":   0.0,
                "initial_pose.z":   0.0,
                "initial_pose.yaw": 0.0,
            },
        ],
        condition=UnlessCondition(use_slam),
    )

    localization_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart":    True,
            "node_names":   ["map_server", "amcl"],
        }],
        condition=UnlessCondition(use_slam),
    )


    nav2_shelf = GroupAction(
        condition=IfCondition(shelf_test),
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    nav2_params_shelf,
                    {"use_sim_time": False},
                ],
                remappings=[("/cmd_vel", "/cmd_vel_nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[
                    nav2_params_shelf,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[
                    nav2_params_shelf,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    nav2_params_shelf,
                    {
                        "use_sim_time": False,
                        "default_nav_to_pose_bt_xml": bt_xml,
                    },
                ],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[
                    nav2_params_shelf,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "autostart":    True,
                    "node_names":   [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "smoother_server",
                    ],
                }],
            ),
        ],
    )


    nav2_real = GroupAction(
        condition=UnlessCondition(shelf_test),
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[
                    nav2_params,
                    {"use_sim_time": False},
                ],
                remappings=[("/cmd_vel", "/cmd_vel_nav")],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[
                    nav2_params,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[
                    nav2_params,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    nav2_params,
                    {
                        "use_sim_time": False,
                        "default_nav_to_pose_bt_xml": bt_xml,
                    },
                ],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[
                    nav2_params,
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "autostart":    True,
                    "node_names":   [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "smoother_server",
                    ],
                }],
            ),
        ],
    )

    delayed_nav2 = TimerAction(
        period=3.0,
        actions=[nav2_shelf, nav2_real],
    )

    return LaunchDescription([
        use_slam_arg,
        map_name_arg,
        shelf_test_arg,
        map_server,
        amcl,
        localization_lifecycle,
        delayed_nav2,
    ])
