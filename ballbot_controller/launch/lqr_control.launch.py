import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    gp_pkg  = get_package_share_directory("gp_description")
    ctrl_pkg = get_package_share_directory("ballbot_controller")

    # ── 1. Gazebo + robot_state_publisher + ros_gz_bridge ────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gp_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )


    # ── 3. Spawn controllers (delay 5s for gz_ros2_control) ──────
    spawn_jsb = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster",
                           "--controller-manager", "/controller_manager"],
            ),
        ],
    )

    spawn_vel = TimerAction(
        period=5.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["ballbot_velocity_controller",
                           "--controller-manager", "/controller_manager"],
            ),
        ],
    )

    omni_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="ballbot_controller",
                executable="ballbot_omnni.py",
                name="ballbot_omni_controller",
                parameters=[{
                    "wheel_radius": 0.05,    # rw
                    "robot_radius": 0.125,   # rk (centre to wheel)
                    "use_sim_time": True,
                }],
                output="screen",
            ),
        ],
    )

    # ── 5. Joystick teleop ────────────────────────────────────────
    joystick = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ctrl_pkg, "launch", "joystick_gp.launch.py")
                ),
                launch_arguments={"use_sim_time": "True"}.items(),
            ),
        ],
    )

    # ── 6. RViz2 ─────────────────────────────────────────────────
    rviz_config = os.path.join(gp_pkg, "rviz", "display.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription([
        gazebo,
        spawn_jsb,
        spawn_vel,
        omni_controller,
        joystick,
        rviz2,
    ])
