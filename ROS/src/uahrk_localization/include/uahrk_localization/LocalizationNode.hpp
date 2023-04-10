#pragma once

#include <Eigen/Dense>
#include <chrono>

#include "uahrk_localization/ekf_krakens.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

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
        void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr odom_msg);
        void lidar_callback(const geometry_msgs::msg::PoseStamped::SharedPtr lidar_msg);
        void camera_callback(const geometry_msgs::msg::PoseStamped::SharedPtr camera_msg);
        // Int. por timer y publisher.
        rclcpp::TimerBase::SharedPtr timer_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr final_pose_publisher;        
        // Subscriber.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr lidar_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr camera_sub;
};