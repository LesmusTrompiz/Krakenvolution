from launch import LaunchDescription
from launch_ros.actions import Node
from os.path import join

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Robot TF Node
    robot_tf = Node(
        package     = 'uahrk_navigation',
        executable  = 'robot_tf_main',
        output      = 'screen'
    )
    
    # Rviz2 with a configurated view to see the tfs
    pkg_dir     = get_package_share_directory('uahrk_navigation')
    config_file = join(pkg_dir, 'config/rviz', 'Tf_config')

    rviz2_tf = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + config_file],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(robot_tf)
    ld.add_action(rviz2_tf)

    return ld
