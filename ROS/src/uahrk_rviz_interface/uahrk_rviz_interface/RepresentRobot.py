import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from math import radians
from geometry_msgs.msg import Quaternion, TransformStamped
from math import cos, sin
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

ROBOT_X = 0.325
ROBOT_Y = 0.325
ROBOT_Z = 0.4

def quaternion_from_angle(angle):
    q = Quaternion()
    cy = cos(angle * 0.5)
    sy = sin(angle * 0.5)
    q.w = cy
    q.x = 0.0
    q.y = 0.0
    q.z = sy
    return q

def init_robot_marker(marker_id) -> Marker:
    robot_marker = Marker()
    robot_marker.id       = marker_id
    robot_marker.ns       = "robot_marker"
    robot_marker.type     = 1
    robot_marker.action   = 0
    robot_marker.scale.x  = ROBOT_X
    robot_marker.scale.y  = ROBOT_Y
    robot_marker.scale.z  = ROBOT_Z
    robot_marker.header.frame_id = "map"
    robot_marker.color.a = 0.6
    robot_marker.color.b = 1.0
    return robot_marker

class RepresentRobotNode(Node): 
    def __init__(self): 
        Node.__init__(self,'Rviz_interface')
        self.pub_obstacles = self.create_publisher(Marker, 'robot_maker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_maker = init_robot_marker(1)
        
    
    def get_robot_tf(self) -> TransformStamped: 
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'robot',
                rclpy.time.Time())
            return t
        except TransformException as ex:
            self.get_logger().info(f'Could not transform map to robot_tf: {ex}')
            return None
    

    def timer_callback(self):
        # Look for robot TF
        robot_pose = self.get_robot_tf()
        
        # If no new tf, just dont do anything
        if robot_pose == None: return

        # Update robot marker
        self.robot_maker.pose.position.x    = robot_pose.transform.translation.x
        self.robot_maker.pose.position.y    = robot_pose.transform.translation.y
        self.robot_maker.pose.position.z    = ROBOT_Z / 2
        self.robot_maker.pose.orientation.w = robot_pose.transform.rotation.w
        self.robot_maker.pose.orientation.x = robot_pose.transform.rotation.x
        self.robot_maker.pose.orientation.y = robot_pose.transform.rotation.y
        self.robot_maker.pose.orientation.z = robot_pose.transform.rotation.z
        self.pub_obstacles.publish(self.robot_maker)

def main(args=None):
    rclpy.init(args=args)
    node_ = RepresentRobotNode()
    rclpy.spin(node_)





