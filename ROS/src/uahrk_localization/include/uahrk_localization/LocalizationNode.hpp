#pragma once

#include <Eigen/Dense>
#include <chrono>

#include "uahrk_localization/ekf_krakens.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

class LocalizationNode : public rclcpp::Node
{
    public:
        // Constructor
        LocalizationNode(const char *node_name, const char *topic_name);
    private:
        // Callbacks
        void pose_pub_callback();
        // Timer, publisher y demás
        rclcpp::TimerBase::SharedPtr timer_pub;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;        
        // Nombre del nodo
        const char *node_name;
        // Dimensiones de las matrices...
        const int m = 1;
        const int n = 3;
        // Filtro de Kalman
        EKFilter  LocalizationFilter;
};