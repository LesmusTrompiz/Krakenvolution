import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from math import radians
from geometry_msgs.msg import Quaternion, TransformStamped
from math import cos, sin
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray


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

def create_robot_marker(marker_id) -> Marker:
    robot_marker = Marker()
    robot_marker.id       = marker_id
    robot_marker.ns       = "robot_marker"
    robot_marker.type     = 1
    robot_marker.action   = 0
    robot_marker.scale.x  = ROBOT_X
    robot_marker.scale.y  = ROBOT_Y
    robot_marker.scale.z  = ROBOT_Z
    robot_marker.header.frame_id = "map"
    robot_marker.color.a = 1.0
    robot_marker.color.r = 1.0
    return robot_marker

class RepresentRobotNode(Node): 
    def __init__(self): 
        Node.__init__(self,'Rviz_interface')
        self.obstacles = MarkerArray()
        self.pub_obstacles = self.create_publisher(MarkerArray, 'obstacles_markers', 10)
        self.sub_obstacles = self.create_subscription(PoseArray, 'obstacles', self.obstacles_cb, 10)
    
    def obstacles_cb(self, new_obstacles : PoseArray):
        if self.obstacles.markers:
            self.erase_obstacles()
        
        for i,obstacle in enumerate(new_obstacles.poses):
            robot_marker = create_robot_marker(i)
            robot_marker.pose.position = obstacle.position
            robot_marker.pose.position.z = ROBOT_Z / 2

            self.obstacles.markers += [robot_marker]
        self.pub_obstacles.publish(self.obstacles)

    def erase_obstacles(self):
        for marker in self.obstacles.markers:
            marker.action = 3
        self.pub_obstacles.publish(self.obstacles)
        self.obstacles.markers = []

def main(args=None):
    rclpy.init(args=args)
    node_ = RepresentRobotNode()
    rclpy.spin(node_)





