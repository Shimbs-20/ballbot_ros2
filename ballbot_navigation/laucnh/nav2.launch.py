import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_pkg = get_package_share_directory("ballbot_mapping")
    nav_pkg  = get_package_share_directory("ballbot_navigation")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true",
        description="true = SLAM mapping  |  false = AMCL localization"
    )
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value=os.path.join(
            map_pkg, "maps", "small_house", "map.yaml"
        ),
        description="Absolute path to saved map .yaml (used when use_slam=false)"
    )

    use_slam = LaunchConfiguration("use_slam")
    map_yaml = LaunchConfiguration("map_yaml")

    localization = GroupAction(
        condition=UnlessCondition(use_slam),
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": False, "yaml_filename": map_yaml}],
            ),
            # AMCL Localizer
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[os.path.join(nav_pkg, "config", "amcl.yaml"), {"use_sim_time": False}],
            ),
            # Controller Server (MPPI)
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[os.path.join(nav_pkg, "config", "nav2_params.yaml")],
                remappings=[('/cmd_vel', '/cmd_vel_nav')] # Routes commands safely through your twist_mux
            ),
            # Planner Server (Smac2D)
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[os.path.join(nav_pkg, "config", "nav2_params.yaml")],
            ),
            # Behavior Tree Navigator
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[os.path.join(nav_pkg, "config", "nav2_params.yaml")],
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
        map_yaml_arg,
        localization,
    ])
