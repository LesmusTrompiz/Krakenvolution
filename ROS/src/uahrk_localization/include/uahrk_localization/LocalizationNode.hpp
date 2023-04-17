#pragma once

#include <Eigen/Dense>
#include <chrono>

#include "uahrk_localization/ekf_krakens.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class LocalizationNode : public rclcpp::Node
{
    public:
        // Constructor.
        LocalizationNode(const char *node_name, const char *topic_name);
    private:
        // Filtro de Kalman.
        EKFilter  LocalizationFilter;
        // Bucle del filtro.
        void pose_pub_callback();
        // Obtención de datos.
        void odom_callback(geometry_msgs::msg::PoseStamped::UniquePtr odom_msg);
        void lidar_callback(geometry_msgs::msg::PoseStamped::UniquePtr lidar_msg);
        void camera_callback(geometry_msgs::msg::PoseStamped::UniquePtr camera_msg);
        void vel_callback(geometry_msgs::msg::TwistStamped::UniquePtr twist_msg);

        // Int. por timer y publisher.
        rclcpp::TimerBase::SharedPtr timer;
        
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr final_pose_publisher;        
        // Subscriber.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_sub;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;


        geometry_msgs::msg::PoseStamped::UniquePtr last_lidar_pose;
        geometry_msgs::msg::PoseStamped::UniquePtr last_camera_pose;
        geometry_msgs::msg::PoseStamped::UniquePtr last_odom_pose;
        geometry_msgs::msg::TwistStamped::UniquePtr last_vel;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        geometry_msgs::msg::TransformStamped tf_robot_odom;
};