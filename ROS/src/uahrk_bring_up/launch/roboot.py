from launch import LaunchDescription
from launch_ros.actions import Node
from os.path import join

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
 
    serial_brige = Node(
        package='serial_bridge',
        executable='serial',
        arguments=['/dev/arduino'],
        output      = 'screen'
    )

    
    # Robot TF Node
    robot_tf = Node(
        package     = 'uahrk_localization',
        executable  = 'Localization.py',
        output      = 'screen'
    )

    # Move to pose node
    move_to_pose = Node(
        package     = 'uahrk_navigation',
        executable  = 'move_to_pose_main',
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

    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_to_base_laser',
        arguments=['0','0','0.4','0','0','0','robot','base_laser']
    )

    analize_scan = Node(
        package='uahrk_lidar',
        executable='analize_scan',
        name='analize_scan',
        output="screen"
    )

    track_node = Node(
        package='uahrk_lidar',
        executable='track_node',
        name='track_node',
        output="screen"
    )

    stop_node = Node(
        package='uahrk_lidar',
        executable='stop_node',
        name='stop_node',
        output="screen"
    )
    
    # Decision making 
    decision_making = Node(
        package     = 'uahrk_decision_making',
        executable  = 'decision'
    )

    ld = LaunchDescription()
    ld.add_action(robot_tf)
    ld.add_action(move_to_pose)
    ld.add_action(decision_making)
    ld.add_action(ldlidar_node)
    ld.add_action(analize_scan)
    ld.add_action(stop_node)
    ld.add_action(track_node)
    ld.add_action(base_link_to_laser_tf_node)
    return ld
