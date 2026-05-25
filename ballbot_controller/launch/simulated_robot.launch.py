import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    gp_pkg = get_package_share_directory("ballbot_description")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gp_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )



    # Delay controller spawning by 5 seconds to let Gazebo
    # fully load and the ros2_control plugin initialize.
    # Without this, the spawner often starts before the
    # controller_manager exists.
    controller = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("ballbot_controller"),
                    "launch",
                    "controller.launch.py",
                ),
                launch_arguments={
                    "use_simple_controller": "False",
                    "use_python": "False",
                }.items(),
            ),
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
    )


    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ballbot_controller"),
            "launch",
            "joystick.launch.py",
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items(),
    )

    safety_stop = Node(
        package="ballbot_utils",
        executable="safety_stop",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ballbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ballbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )


    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("ballbot_localization"),
                "rviz",
                "global_localization.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("ballbot_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        gazebo,
        use_slam_arg,
        controller,
        joystick,
        safety_stop,
        localization,
        slam
        # rviz_localization,
        # rviz_slam
    ])
