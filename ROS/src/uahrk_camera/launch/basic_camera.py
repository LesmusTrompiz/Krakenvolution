from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uahrk_camera',
            namespace='camera1',
            executable='camera_node',
            name='camera1'
        )
    ])