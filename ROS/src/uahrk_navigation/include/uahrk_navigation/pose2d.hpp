#pragma once

extern "C"{
    #include <math.h>
}
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"


inline float RAD2DEG(float x) {return x * 180 / M_PI;}


struct Pose2d
{
    float x;
    float y;
    float a;

    Pose2d()                             : x{0.0} , y{0.0} , a{0.0} {};
    Pose2d(float _x, float _y)           : x{_x}  , y{_y}  , a{0.0} {};
    Pose2d(float _x, float _y, float _a) : x{_x}  , y{_y}  , a{_a}  {};

    Pose2d(const geometry_msgs::msg::TransformStamped &tf):
        x{(float)tf.transform.translation.x}, 
        y{(float)tf.transform.translation.y}
    {
        double roll, pitch, yaw;

        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        a = RAD2DEG(yaw);
    }
    Pose2d(const geometry_msgs::msg::Pose &rosp):
        x{(float)rosp.position.x}, 
        y{(float)rosp.position.y}
    {
        double roll, pitch, yaw;
        tf2::Quaternion q(
            rosp.orientation.x,
            rosp.orientation.y,
            rosp.orientation.z,
            rosp.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        a = RAD2DEG(yaw);
    }


    friend inline bool operator==(const Pose2d& lhs, const Pose2d& rhs) { 
        return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.a == rhs.a); 
    }

    friend inline bool operator!=(const Pose2d& lhs, const Pose2d& rhs) {
        return !(lhs == rhs);
    }

    void operator=(geometry_msgs::msg::TransformStamped &tf){
        double roll, pitch, yaw;
        x = tf.transform.translation.x; 
        y = tf.transform.translation.y;
        tf2::Quaternion q(
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        a = RAD2DEG(yaw);
    }

    void operator=(const geometry_msgs::msg::Pose rosp){
        double roll, pitch, yaw;
        x = rosp.position.x; 
        y = rosp.position.y;
        tf2::Quaternion q(
            rosp.orientation.x,
            rosp.orientation.y,
            rosp.orientation.z,
            rosp.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        a = RAD2DEG(yaw);
    }
};
