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

    # Grid node
    grid = Node(
        package     = 'uahrk_navigation',
        executable  = 'grid_main',
        output      = 'screen'
    )

    # Path Finding node
    path_finding = Node(
        package     = 'uahrk_navigation',
        executable  = 'path_finding_main',
        output      = 'screen'
    )

    # Decision making 
    decision_making = Node(
        package     = 'uahrk_decision_making',
        executable  = 'decision',
        output      = 'screen'
    )

    # Rviz interface node
    rviz2_interface = Node(
        package='uahrk_rviz_interface',
        executable='rviz_interface_main',
        output='screen'
    )
  

    ld = LaunchDescription()
    ld.add_action(move_to_pose)
    #ld.add_action(path_finding)
    ld.add_action(rviz2_interface)
    #ld.add_action(grid)
    #ld.add_action(decision_making)



    return ld
