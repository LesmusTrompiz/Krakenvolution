from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    serial_brige = Node(
        package='serial_bridge',
        executable='serial',
        arguments=['/dev/arduino'],
        output      = 'screen'
    )

    # Lidar launch
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    stop_node = Node(
        package='uahrk_lidar',
        executable='stop_node',
        name='stop_node',
        output="screen"
    )

    analize_scan = Node(
        package='uahrk_lidar',
        executable='analize_scan',
        name='analize_scan',
        output="screen"
    )
    sequencer = Node(
        package     = 'uahrk_sequencer',
        executable  = 'sequencer',
        output      = 'screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(serial_brige)
    ld.add_action(ldlidar_node)
    ld.add_action(analize_scan)
    ld.add_action(stop_node)
    ld.add_action(sequencer)
    return ld
