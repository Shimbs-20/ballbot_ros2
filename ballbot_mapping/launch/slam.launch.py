import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    lifecycle_nodes = ["map_saver_server", "slam_toolbox"]

    slam_config = LaunchConfiguration("slam_config")
    use_sim_true = LaunchConfiguration("use_sim_time")

    use_sim_true_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("ballbot_mapping"),
            "config",
            "slam_toolbox.yaml"
        )
    )

    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_true},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65},
        ],
    )

    slam_tool_box = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_true},
        ] #
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_true},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        use_sim_true_arg,
        slam_config_arg,
        nav2_map_saver,
        slam_tool_box,
        nav2_lifecycle_manager,
    ])
