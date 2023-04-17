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




class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        self.tf_br       = TransformBroadcaster(self)
        
        self.tf_map_odom = TransformStamped()
        self.odom_pose   = PoseStamped()
        self.lidar_pose  = PoseStamped()
        self.camera_pose = PoseStamped()
        self.v = 0
        self.w = 0
        dt = 0.1

        points = MerweScaledSigmaPoints(n=5, alpha=.01, beta=2, kappa=0, subtract=residual_state)
        
        self.ukf = UKF(dim_x=5, dim_z=2, fx=fx, hx=Hx, dt=dt, points=points,
            x_mean_fn=state_mean, z_mean_fn=state_mean,
            residual_x=residual_state, residual_z=residual_state)

        self.vel_sub = self.create_subscription(
            TwistStamped,
            'robot_vel',
            self.vel_cb,
            10)

        self.odom_sub = self.create_subscription(
            PoseStamped,
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
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.tick_node)

    def odom_cb(self, pose: PoseStamped):
        pose = self.do_tf(pose)
        self.odom_pose = pose

    def lidar_cb(self, pose: PoseStamped):
        self.lidar_pose = pose

    def camera_cb(self, pose: PoseStamped):
        self.camera_pose = pose

    def vel_cb(self, vel : TwistStamped):
        self.v = vel.twist.linear.x 
        self.w = vel.twist.angular.z


    def do_tf(self, pose: PoseStamped) -> TransformStamped: 
        p = PoseStamped()
        
        trans = [self.tf_map_odom.transform.translation.x,
                    self.tf_map_odom.transform.translation.y,
                    self.tf_map_odom.transform.translation.z]
        rot   = [self.tf_map_odom.transform.rotation.x,
                    self.tf_map_odom.transform.rotation.y,
                    self.tf_map_odom.transform.rotation.z,
                    self.tf_map_odom.transform.rotation.w]
        
        matrix = dot(translation_matrix(trans),                     # Creat a np transformation matrix.
                    quaternion_matrix(rot))

        _,_, tf_yaw = quaternion_matrix
        q = [pose.pose.orientation.x,
             pose.pose.orientation.y,
             pose.pose.orientation.z,
            pose.pose.orientation.w]
        _,_,pose_yaw = euler_from_quaternion(q)

        final_yaw = tf_yaw + pose_yaw

        x,y = tuple(dot(matrix,                                  # Calculate the x,y pose refered to map coordinates system
                        array([
                            pose.pose.position.x, 
                            pose.pose.position.y, 
                            pose.pose.position.z, 
                            1.0])))[:2]
        
        p.pose.position.x = x
        p.pose.position.y = y

        q = quaternion_from_euler(0,0,final_yaw)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        return p
        

    def tick_node(self):
        # Create z sensor
        a = euler_from_quaternion([ self.odom_pose.pose.orientation.x,
                                  self.odom_pose.pose.orientation.y,
                                  self.odom_pose.pose.orientation.z,
                                  self.odom_pose.pose.orientation.z,]
                                  )
        print(a)
        '''
        z_sensor = np.array([self.odom_pose.pose.position.x,
                            self.odom_pose.pose.position.y,
                            a[2]])

        self.ukf.predict(u = np.array([self.v, self.w]))
        '''
        b = TransformStamped()
        b.header.frame_id = "map"
        b.child_frame_id = ""

        b.transform.rotation = self.odom_pose.pose.orientation
        b.transform.translation = self.odom_pose.pose.position

        self.tf_br.sendTransform()
        #self.ukf.update(z_sensor)



def main(args=None):
    rclpy.init(args=args)
    localization = LocalizationNode()
    rclpy.spin(localization)

if __name__ == "__main__":
    main()
