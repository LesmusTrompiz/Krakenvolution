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
    # Rviz2 with a configurated view to see the tfs
    pkg_dir     = get_package_share_directory('uahrk_navigation')
    config_file = join(pkg_dir, 'config/rviz', 'Tf_config')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + config_file],
    )

    # Rviz interface node
    rviz_map_markers = Node(
        package='uahrk_rviz_interface',
        executable='map_markers_main',
        output='screen'
    )

    # Rviz obstacles markers
    rviz_obstacles_markers = Node(
        package='uahrk_rviz_interface',
        executable='represent_enemy_obstacles_main',
        output='screen'
    )

    # Rviz robot markers
    rviz_robot_markers = Node(
        package='uahrk_rviz_interface',
        executable='represent_robot_main',
        output='screen'
    )


    ld = LaunchDescription()
    #ld.add_action(dummy_serial)
    #ld.add_action(robot_tf)
    ld.add_action(move_to_pose)
    ld.add_action(path_finding)
    ld.add_action(rviz_obstacles_markers)
    ld.add_action(rviz_robot_markers)
    ld.add_action(rviz2_interface)
    ld.add_action(rviz_map_markers)
    #ld.add_action(rviz2)
    ld.add_action(grid)
    #ld.add_action(decision_making)



    return ld
