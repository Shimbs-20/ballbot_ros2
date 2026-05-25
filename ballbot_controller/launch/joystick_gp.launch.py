import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time",
    )

    ballbot_controller_pkg = get_package_share_directory("ballbot_controller")
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            os.path.join(ballbot_controller_pkg, "config", "joy_config.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    joy_teleop_node = Node(
        package="joy_teleop",
        executable="joy_teleop",
        name="joy_teleop",
        parameters=[
            os.path.join(ballbot_controller_pkg, "config", "joy_teleop_gp.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )


    # twist_mux merges /cmd_vel_nav (priority 10), /cmd_vel_key (50),
    # and /input_joy/cmd_vel (100) → output on /ballbot_controller/cmd_vel_unstamped
    #
    # BUG FIX: The original remapping sent twist_mux output to
    # "/ballbot_controller/cmd_vel_unstamped" which is correct for the
    # diff_drive_controller path when use_stamped_vel=false, BUT the YAML
    # had use_stamped_vel=true and no stamping was happening at that point.
    # Now the chain is:
    #   twist_mux → /ballbot_controller/cmd_vel_unstamped (Twist)
    #   twist_relay subscribes there, stamps it, publishes on
    #   ballbot_controller/cmd_vel (TwistStamped) → diff_drive_controller
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            # BUG FIX 3: Missing leading slash on cmd_vel_out.
            # Without it twist_mux publishes on a relative topic and
            # twist_relay never receives anything.
            "cmd_vel_out": "/ballbot_controller/cmd_vel_unstamped",
            "config_locks":  os.path.join(ballbot_controller_pkg, "config", "twist_mux_locks.yaml"),
            # BUG FIX 4: Filename was "twist_mux_topics.yaml" (with an 's')
            # but the actual file is "twist_mux_topic.yaml" (no 's').
            # This caused a FileNotFoundError at launch.
            "config_topics": os.path.join(ballbot_controller_pkg, "config", "twist_mux_topic_gp.yaml"),
            "config_joy":    os.path.join(ballbot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time":  LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    # twist_relay stamps the Twist coming out of twist_mux and forwards it as
    # TwistStamped on /ballbot_controller/cmd_vel, which is what
    # diff_drive_controller subscribes to when use_stamped_vel=true.
    #
    # Full chain (works for joy_teleop, NAV2 /cmd_vel_nav, keyboard /cmd_vel_key,
    # PS4 controller — all go through twist_mux):
    #   source → twist_mux → /ballbot_controller/cmd_vel_unstamped (Twist)
    #          → twist_relay → /ballbot_controller/cmd_vel (TwistStamped)
    #          → diff_drive_controller
    twist_relay_node = Node(
        package="ballbot_controller",
        executable="twist_relay",          # compiled C++ binary
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_node,
        joy_teleop_node,
        twist_mux_launch,
    ])
