#!/usr/bin/env python3
"""
nav.launch.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 2 — run AFTER hardware.launch.py is stable.
Starts SLAM or AMCL localization. ~600-900 MB RAM.

SLAM mapping (build a new map):
  ros2 launch ballbot_controller nav.launch.py use_slam:=true

Localization on small_house map:
  ros2 launch ballbot_controller nav.launch.py use_slam:=false \
    map_yaml:=/home/GP-Robot/ros2_ws/src/ballbot_controller/maps/small_house/map.yaml

Localization on small_warehouse map:
  ros2 launch ballbot_controller nav.launch.py use_slam:=false \
    map_yaml:=/home/GP-Robot/ros2_ws/src/ballbot_controller/maps/small_warehouse/map.yaml

Localization on your real-world saved map:
  ros2 launch ballbot_controller nav.launch.py use_slam:=false \
    map_yaml:=/home/GP-Robot/maps/my_room.yaml

Save map after SLAM session (run in a 3rd terminal):
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():

    ctrl_pkg = get_package_share_directory("ballbot_controller")
    map_pkg  = get_package_share_directory("ballbot_mapping")

    # ── Arguments ─────────────────────────────────────────────────
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true",
        description="true = SLAM mapping  |  false = AMCL localization"
    )
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value=os.path.join(
            ctrl_pkg, "maps", "small_house", "map.yaml"
        ),
        description="Absolute path to saved map .yaml (used when use_slam=false)"
    )

    use_slam = LaunchConfiguration("use_slam")
    map_yaml = LaunchConfiguration("map_yaml")

    # ══════════════════════════════════════════════════════════════
    # SLAM MODE (use_slam:=true)
    # slam_toolbox async — queues scans so Pi never misses one
    # map_saver_server — call map_saver_cli to dump the map
    # lifecycle_manager — activates both in correct order
    # ══════════════════════════════════════════════════════════════
    slam = GroupAction(
        condition=IfCondition(use_slam),
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    os.path.join(map_pkg, "config", "slam_toolbox.yaml"),
                    {"use_sim_time": False},
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_saver_server",
                name="map_saver_server",
                output="screen",
                parameters=[{
                    "use_sim_time":            False,
                    "save_map_timeout":        5.0,
                    "free_thresh_default":     0.196,
                    "occupied_thresh_default": 0.65,
                }],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "autostart":    True,
                    "node_names":   ["map_saver_server", "slam_toolbox"],
                }],
            ),
        ],
    )


    localization = GroupAction(
        condition=UnlessCondition(use_slam),
        actions=[
            # map_server starts immediately — serves the saved .pgm map
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{
                    "use_sim_time":  False,
                    "yaml_filename": map_yaml,
                }],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="nav2_amcl",
                        executable="amcl",
                        name="amcl",
                        output="screen",
                        parameters=[
                            os.path.join(ctrl_pkg, "config", "amcl.yaml"),
                            {"use_sim_time": False},
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_localization",
                        output="screen",
                        parameters=[{
                            "use_sim_time": False,
                            "autostart":    True,
                            "node_names":   ["map_server", "amcl"],
                        }],
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription([
        use_slam_arg,
        map_yaml_arg,
        slam,
        localization,
    ])
