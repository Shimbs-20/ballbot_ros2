import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction
)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_pkg = get_package_share_directory("ballbot_mapping")
    nav_pkg = get_package_share_directory("ballbot_navigation")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true",
        description="true = SLAM mapping  |  false = AMCL localization"
    )

    # ── NEW: Simple map selector argument ──
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Name of the map folder (e.g., small_house, small_warehouse)"
    )

    use_slam = LaunchConfiguration("use_slam")
    map_name = LaunchConfiguration("map_name")

    # Dynamic path construction using PathJoinSubstitution
    map_yaml_path = PathJoinSubstitution([
        map_pkg, "maps", map_name, "map.yaml"
    ])

    localization = GroupAction(
        condition=UnlessCondition(use_slam),
        actions=[
            # Map Server
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": False, "yaml_filename": map_yaml_path}],
            ),
            # AMCL Localizer
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[os.path.join(map_pkg, "config", "amcl.yaml"), {"use_sim_time": False}],
            ),
            # Controller Server (MPPI)
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[os.path.join(nav_pkg, "config", "nav2_param.yaml")],
                remappings=[('/cmd_vel', '/cmd_vel_nav')]
            ),
            # Planner Server (Smac2D)
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[os.path.join(nav_pkg, "config", "nav2_param.yaml")],
            ),
            # Behavior Tree Navigator
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[os.path.join(nav_pkg, "config", "nav2_param.yaml")],
            ),
            # Lifecycle Manager coordinating the whole stack
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "autostart":    True,
                    "node_names":   ["map_server", "amcl", "planner_server", "controller_server", "bt_navigator"],
                }],
            ),
        ],
    )

    return LaunchDescription([
        use_slam_arg,
        map_name_arg,
        localization,
    ])
