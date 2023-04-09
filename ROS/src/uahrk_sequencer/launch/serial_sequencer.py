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
    sequencer = Node(
        package     = 'uahrk_sequencer',
        executable  = 'sequencer',
        output      = 'screen'
    )
    
    # Robot TF Node
    robot_tf = Node(
        package     = 'uahrk_navigation',
        executable  = 'robot_tf_main',
        output      = 'screen'
    )
    


    ld = LaunchDescription()
    ld.add_action(serial_brige)
    ld.add_action(sequencer)
    ld.add_action(robot_tf)
    return ld
