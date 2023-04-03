import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from math import radians
from geometry_msgs.msg import Quaternion
from math import cos, sin
Z_WALL = 0.1
Z_PORTACEREZAS = 0.03
Z_SPAWN = 0.01




def quaternion_from_angle(angle):
    q = Quaternion()
    cy = cos(angle * 0.5)
    sy = sin(angle * 0.5)
    q.w = cy
    q.x = 0.0
    q.y = 0.0
    q.z = sy
    return q

def create_wall_marker(x_range,y_range,rot,marker_id):
    wall_marker = Marker()
    wall_marker.id       = marker_id
    wall_marker.ns       = "map_marker"
    wall_marker.type     = 1
    wall_marker.action   = 0
    wall_marker.scale.x  = float(x_range[1] - x_range[0])
    wall_marker.scale.y  = float(y_range[1] - y_range[0])
    wall_marker.scale.z  = Z_WALL
    wall_marker.pose.position.x = float(x_range[0] + x_range[1])/2
    wall_marker.pose.position.y = float(y_range[0] + y_range[1])/2
    wall_marker.pose.position.z = Z_WALL / 2
    quat                        = quaternion_from_angle(radians(rot))
    wall_marker.pose.orientation = quat
    wall_marker.header.frame_id = "map"
    wall_marker.color.a = 1.0
    wall_marker.color.g = 1.0
    return wall_marker

def create_porta_cerezas(x_range,y_range,rot,marker_id):
    porta_cerezas = Marker()
    porta_cerezas.id       = marker_id
    porta_cerezas.ns       = "porta_cerezas"
    porta_cerezas.type     = 1
    porta_cerezas.action   = 0
    porta_cerezas.scale.x  = float(x_range[1] - x_range[0])
    porta_cerezas.scale.y  = float(y_range[1] - y_range[0])
    porta_cerezas.scale.z  = Z_PORTACEREZAS
    porta_cerezas.pose.position.x = float(x_range[0] + x_range[1])/2
    porta_cerezas.pose.position.y = float(y_range[0] + y_range[1])/2
    porta_cerezas.pose.position.z = Z_PORTACEREZAS/2
    quat                        = quaternion_from_angle(radians(rot))
    porta_cerezas.pose.orientation = quat
    porta_cerezas.header.frame_id = "map"
    porta_cerezas.color.a = 1.0
    porta_cerezas.color.g = 1.0
    return porta_cerezas


def create_spawns(x_range,y_range,rot,marker_id,color):
    porta_cerezas = Marker()
    porta_cerezas.id       = marker_id
    porta_cerezas.ns       = "spawns"
    porta_cerezas.type     = 1
    porta_cerezas.action   = 0
    porta_cerezas.scale.x  = float(x_range[1] - x_range[0])
    porta_cerezas.scale.y  = float(y_range[1] - y_range[0])
    porta_cerezas.scale.z  = Z_SPAWN
    porta_cerezas.pose.position.x = float(x_range[0] + x_range[1])/2
    porta_cerezas.pose.position.y = float(y_range[0] + y_range[1])/2
    porta_cerezas.pose.position.z = Z_SPAWN / 2
    quat                        = quaternion_from_angle(radians(rot))
    porta_cerezas.pose.orientation = quat
    porta_cerezas.header.frame_id = "map"
    porta_cerezas.color.a = 0.8

    if color == "g":
        porta_cerezas.color.g = 1.0
    elif color == "b":
        porta_cerezas.color.b = 1.0
    return porta_cerezas


class rviz_map_markers(Node): 
    def __init__(self): 
        Node.__init__(self,'Rviz_interface')
        self.pub_obstacles = self.create_publisher(MarkerArray, 'map_elements', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        map_elements = MarkerArray()
        map_elements.markers += [create_wall_marker((-0.1, 2.1), (-0.1, 0.0), 0, 1)]
        map_elements.markers += [create_wall_marker((-0.1, 0.0), (-0.1, 3.1), 0, 2)]
        map_elements.markers += [create_wall_marker((-0.1, 2.1), ( 3.0, 3.1), 0, 3)]
        map_elements.markers += [create_wall_marker(( 2.0, 2.1), (-0.1, 3.1), 0, 4)]
        map_elements.markers += [create_porta_cerezas((1 - 0.015, 1 + 0.015), (0     , 0.3  ), 0, 5)]
        map_elements.markers += [create_porta_cerezas((1 - 0.015, 1 + 0.015), (2.7   , 3.0  ), 0, 6)]
        map_elements.markers += [create_porta_cerezas((0.0   , 0.03 ), (1.350 , 1.650), 0, 7)]
        map_elements.markers += [create_porta_cerezas((1.97  , 2.0  ), (1.350 , 1.650), 0, 8)]
        map_elements.markers += [create_spawns((0           ,  0.45), (0                      ,                         0.45), 0,  9,'g')]
        map_elements.markers += [create_spawns((0           ,  0.45), (0.9                    ,                   0.9 + 0.45), 0, 10,'b')]
        map_elements.markers += [create_spawns((0           ,  0.45), (0.9 + 0.3 + 0.45       ,      0.9 + 0.45 + 0.3 + 0.45), 0, 11,'g')]
        map_elements.markers += [create_spawns((0           ,  0.45), (0.9 + 0.3 + 0.45 + 0.9 , 0.9 + 0.45 + 0.3 + 0.9+ 0.45), 0, 12,'b')]
        map_elements.markers += [create_spawns((1.999 - 0.45, 1.999), (0                      ,                         0.45), 0, 13,'b')]
        map_elements.markers += [create_spawns((1.999 - 0.45, 1.999), (0.9                    ,                   0.9 + 0.45), 0, 14,'g')]
        map_elements.markers += [create_spawns((1.999 - 0.45, 1.999), (0.9 + 0.3 + 0.45       ,      0.9 + 0.45 + 0.3 + 0.45), 0, 15,'b')]
        map_elements.markers += [create_spawns((1.999 - 0.45, 1.999), (0.9 + 0.3 + 0.45 + 0.9 , 0.9 + 0.45 + 0.3 + 0.9+ 0.45), 0, 16,'g')]
        map_elements.markers += [create_spawns((0.5         ,  0.95), (0                      ,                         0.45), 0, 17,'b')]
        map_elements.markers += [create_spawns((1.05        ,  1.50), (0                      ,                         0.45), 0, 18,'g')]



        self.pub_obstacles.publish(map_elements)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = rviz_map_markers()
    rclpy.spin(minimal_publisher)





