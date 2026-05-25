from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition



def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.17",
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.33",
    )

    # BUG FIX: wheel_radius_error was declared but never forwarded to any node.
    # Kept here in case you add an odometry calibration node later; harmless.
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )

    # LaunchConfiguration objects are substitutions used INSIDE node definitions.
    # BUG FIX 1: "use_sim_time" LaunchConfiguration was added to the
    # LaunchDescription list directly — that is what caused the crash:
    # "'LaunchConfiguration' object has no attribute 'describe_sub_entities'"
    # Only DeclareLaunchArgument objects (and actions/nodes) go in the list.
    use_sim_time          = LaunchConfiguration("use_sim_time")
    use_python            = LaunchConfiguration("use_python")
    wheel_radius          = LaunchConfiguration("wheel_radius")
    wheel_separation      = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    # Full diff_drive_controller path (use_simple_controller=False)
    wheel_simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ballbot_controller",
            "--controller-manager", "/controller_manager",
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    # Simple velocity controller path (use_simple_controller=True)
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager", "/controller_manager",
                ],
            ),
            # Python variant
            Node(
                package="ballbot_controller",
                executable="simple_controller.py",
                parameters=[{
                    "wheel_radius":    wheel_radius,
                    # BUG FIX: parameter key must be "wheel_separation" to match
                    # declare_parameter() call in simple_controller.cpp/py
                    "wheel_separation": wheel_separation,
                    "use_sim_time": use_sim_time,
                }],
                condition=IfCondition(use_python),
            ),
            # C++ variant
            Node(
                package="ballbot_controller",
                executable="simple_controller",
                parameters=[{
                    "wheel_radius":    wheel_radius,
                    "wheel_separation": wheel_separation,
                    "use_sim_time": use_sim_time,
                }],
                condition=UnlessCondition(use_python),
            ),
        ],
    )

    return LaunchDescription([
        use_python_arg,
        use_simple_controller_arg,
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        joint_state_broadcaster,
        simple_controller,
        wheel_simple_controller,
    ])
