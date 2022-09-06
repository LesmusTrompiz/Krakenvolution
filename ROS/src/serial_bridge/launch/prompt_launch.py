from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_bridge',
            executable='serial',
            name='serial'
        ),
        Node(
            package='serial_bridge',
            executable='serial_client',
            name='prompt'
        )
    ])