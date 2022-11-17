from launch import LaunchDescription
from launch_ros.actions import Node
from os.path import join

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Dumy serial node
    dummy_serial = Node(
        package     = 'serial_bridge',
        executable  = 'dummy_serial',
        output      = 'screen'
    )

    # Robot TF Node
    robot_tf = Node(
        package     = 'uahrk_navigation',
        executable  = 'robot_tf_main',
        output      = 'screen'
    )
    
    # Move to pose node
    move_to_pose = Node(
        package     = 'uahrk_navigation',
        executable  = 'move_to_pose_main',
        output      = 'screen'
    )

    # Rviz interface node
    rviz2_interface = Node(
        package='uahrk_navigation',
        executable='rviz_interface',
        output='screen'
    )

    # Rviz2 with a configurated view to see the tfs
    pkg_dir     = get_package_share_directory('uahrk_navigation')
    config_file = join(pkg_dir, 'config/rviz', 'Tf_config')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + config_file],
    )


    ld = LaunchDescription()
    ld.add_action(dummy_serial)
    ld.add_action(robot_tf)
    ld.add_action(move_to_pose)
    ld.add_action(rviz2_interface)
    ld.add_action(rviz2)

    return ld
