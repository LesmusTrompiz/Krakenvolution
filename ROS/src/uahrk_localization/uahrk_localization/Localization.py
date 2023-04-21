#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TwistStamped
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener

from tf_transformations import translation_matrix,quaternion_matrix,quaternion_from_euler, euler_from_quaternion
from numpy import dot,array
import rclpy.duration
from ukf import *
from uahrk_navigation_msgs.srv import SetPose2d
from math import degrees, radians, pi


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        self.tf_br       = TransformBroadcaster(self)
        self.tf_map_odom = TransformStamped()
        self.odom_pose   = Pose()
        self.lidar_pose  = PoseStamped()
        self.camera_pose = PoseStamped()
        self.v = 0
        self.w = 0
        dt = 0.1

        points = MerweScaledSigmaPoints(n=5, alpha=.01, beta=2, kappa=0, subtract=residual_state)
        self.ukf = UKF(dim_x=5, dim_z=3, fx=fx, hx=Hx, dt=dt, points=points,
            x_mean_fn=state_mean, z_mean_fn=z_mean,
            residual_x=residual_state, residual_z=residual_z)
        
        #self.ukf.P = np.diag([0.1**2, 0.1**2, 3**2, 0.1**2, 0.1**2])
        #self.ukf.Q = np.diag([0.3**2, 0.3**2, 6**2, 0.3**2, 0.3**2])
        #self.ukf.R = np.diag([0.2**2, 0.2**2, 1**2])



        self.best_pose_pub = self.create_publisher(
            PoseStamped,
            'best_pose',
            10)
        
        self.vel_sub = self.create_subscription(
            TwistStamped,
            'robot_vel',
            self.vel_cb,
            10)

        self.odom_sub = self.create_subscription(
            Pose,
            'odom_pose',
            self.odom_cb,
            10)

        self.lidar_sub = self.create_subscription(
            PoseStamped,
            'lidar_pose',
            self.lidar_cb,
            10)      

        self.camera_sub = self.create_subscription(
            PoseStamped,
            'camera_pose',
            self.camera_cb,
            10) 
        
        self.tf_reset = self.create_service(
            SetPose2d,
            'reset_odom',
            self.update_tf_odom) 

        self.reset_pose = self.create_service(
            SetPose2d,
            'set_pose',
            self.update_pose) 

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.tick_node)


    def update_pose(self, request: SetPose2d.Request, response: SetPose2d.Response):
        print(f"Update pose {request.pose.x}, {request.pose.y} {request.pose.theta}")
        self.tf_map_odom.transform.translation.x += request.pose.x
        self.tf_map_odom.transform.translation.y += request.pose.y

        q = quaternion_from_euler(0,0,request.pose.theta)
        self.tf_map_odom.transform.rotation.x = q[0]
        self.tf_map_odom.transform.rotation.y = q[1]
        self.tf_map_odom.transform.rotation.z = q[2]
        self.tf_map_odom.transform.rotation.w = q[3]

        self.ukf.P = np.diag([0.1**2, 0.1**2, 0.1**2, 0.1**2, 0.1**2])
        self.ukf.Q = np.diag([0.3**2, 0.3**2, 0.3**2, 0.3**2, 0.3**2])
        self.ukf.R = np.diag([0.2**2, 0.2**2, 0.2**2])
        self.ukf.x = np.array([request.pose.x, request.pose.y, radians(request.pose.theta)])

        return response


    def update_tf_odom(self, request: SetPose2d.Request, response: SetPose2d.Response):
        q = [self.tf_map_odom.transform.rotation.x,
             self.tf_map_odom.transform.rotation.y,
             self.tf_map_odom.transform.rotation.z,
             self.tf_map_odom.transform.rotation.w]
        _,_,yaw = euler_from_quaternion(q)
        
        print(f"Reset odom {request.pose.x}, {request.pose.y} {request.pose.theta}")
        self.tf_map_odom.transform.translation.x += request.pose.x * cos(yaw)
        self.tf_map_odom.transform.translation.y += request.pose.x * sin(yaw)
        self.tf_map_odom.transform.translation.x += request.pose.y * sin(yaw)
        self.tf_map_odom.transform.translation.y += request.pose.y * cos(yaw)

        yaw +=  radians(request.pose.theta)
        
        if   yaw > 2 * pi: yaw -= 2 * pi
        elif yaw < -2 * pi: yaw += 2 * pi

        q = quaternion_from_euler(0,0,yaw)
        self.tf_map_odom.transform.rotation.x = q[0]
        self.tf_map_odom.transform.rotation.y = q[1]
        self.tf_map_odom.transform.rotation.z = q[2]
        self.tf_map_odom.transform.rotation.w = q[3]
        return response

    def odom_cb(self, pose: Pose):
        pose = self.do_tf(pose)
        self.odom_pose = pose

    def lidar_cb(self, pose: PoseStamped):
        self.lidar_pose = pose

    def camera_cb(self, pose: PoseStamped):
        self.camera_pose = pose

    def vel_cb(self, vel : TwistStamped):
        self.v = vel.twist.linear.x 
        self.w = vel.twist.angular.z

    def do_tf(self, pose: Pose) -> TransformStamped: 
        p = Pose()
        
        trans = [self.tf_map_odom.transform.translation.x,
                    self.tf_map_odom.transform.translation.y,
                    self.tf_map_odom.transform.translation.z]
        rot   = [self.tf_map_odom.transform.rotation.x,
                    self.tf_map_odom.transform.rotation.y,
                    self.tf_map_odom.transform.rotation.z,
                    self.tf_map_odom.transform.rotation.w]
        
        matrix = dot(translation_matrix(trans),                     # Creat a np transformation matrix.
                    quaternion_matrix(rot))
        
        _,_,tf_yaw = euler_from_quaternion(rot)
        q = [pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w]
        _,_,pose_yaw = euler_from_quaternion(q)

        final_yaw = tf_yaw + pose_yaw

        x,y = tuple(dot(matrix,                                  # Calculate the x,y pose refered to map coordinates system
                        array([
                            pose.position.x, 
                            pose.position.y, 
                            pose.position.z, 
                            1.0])))[:2]
        
        p.position.x = x
        p.position.y = y

        q = quaternion_from_euler(0,0,final_yaw)
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        return p
        

    def tick_node(self):
        # Create z sensor
        _,_,yaw = euler_from_quaternion([ self.odom_pose.orientation.x,
                                  self.odom_pose.orientation.y,
                                  self.odom_pose.orientation.z,
                                  self.odom_pose.orientation.z,]
                                  )
        z_sensor = np.array([self.odom_pose.position.x,
                             self.odom_pose.position.y,
                             yaw])
        self.ukf.predict(u = np.array([self.v, self.w]))
        self.ukf.update(z_sensor)

        b = TransformStamped()
        b.header.frame_id = "map"
        b.child_frame_id = "robot"
        b.header.stamp = self.get_clock().now().to_msg()
        b.transform.rotation = self.odom_pose.orientation
        b.transform.translation.x = self.odom_pose.position.x
        b.transform.translation.y = self.odom_pose.position.y
        self.tf_br.sendTransform(b)
        
        best_pose = PoseStamped()
        best_pose.pose.position.x = self.ukf.x[0]
        best_pose.pose.position.y = self.ukf.x[1]
        q = quaternion_from_euler(0, 0, self.ukf.x[2])
        best_pose.pose.orientation.x = q[0]
        best_pose.pose.orientation.y = q[1]
        best_pose.pose.orientation.z = q[2]
        best_pose.pose.orientation.w = q[3]
        best_pose.header.frame_id = "map"
        best_pose.header.stamp = self.get_clock().now().to_msg()
        self.best_pose_pub.publish(best_pose)

        

def main(args=None):
    rclpy.init(args=args)
    localization = LocalizationNode()
    rclpy.spin(localization)

if __name__ == "__main__":
    main()
