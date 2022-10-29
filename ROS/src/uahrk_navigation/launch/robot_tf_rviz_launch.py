from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    
    # Robot TF Node
    robot_tf = Node(
        package     = 'uahrk_navigation',
        executable  = 'robot_tf_main',
        output      = 'screen'
    )
    
    # Rviz2 with a
    # 
    rviz2_tf = Node(
        package='rviz2',
        executable='rviz2',
        # parameters=[{
        # 'particles': 300,
        # 'topics': ['scan', 'image'],
        # 'topic_types': ['sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/Image']
        # }],
        # output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(robot_tf)
    ld.add_action(rviz2_tf)

    return ld
