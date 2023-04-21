from launch import LaunchDescription
from launch_ros.actions import Node
from os.path import join

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    

    
    # Move to pose node
    move_to_pose = Node(
        package     = 'uahrk_navigation',
        executable  = 'move_to_pose_main',
        output      = 'screen'
    )

    # Rviz interface node
    rviz2_interface = Node(
        package='uahrk_rviz_interface',
        executable='rviz_interface_main',
        output='screen'
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

    pkg_dir     = get_package_share_directory('uahrk_lidar')
    #config_file = join(pkg_dir, '/config/', 'lidar_rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
       # arguments=['-d ' + config_file],
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



    ld = LaunchDescription()
    ld.add_action(ldlidar_node)
    ld.add_action(analize_scan)
    ld.add_action(base_link_to_laser_tf_node)
    ld.add_action(move_to_pose)
    ld.add_action(rviz2_interface)
    return ld
