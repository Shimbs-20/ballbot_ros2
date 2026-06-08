from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Node 1: Camera streaming
        Node(
            package='camera_pi',
            executable='camera_node',
            name='easy_camera',
            parameters=[{
                'width': 640,      # smaller = faster for detector
                'height': 480,
                'framerate': 30
            }]
        ),

        Node(
            package='camera_pi',
            executable='hazard_detector',
            name='hazard_detector',
        ),

        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'port': 8080}]
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}]
        ),
    ])
