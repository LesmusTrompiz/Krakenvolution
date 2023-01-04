#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <math.h>

using namespace std;
using namespace std::chrono_literals;

// Posición inicial
constexpr float x_0 = 0.0;
constexpr float y_0 = 0.0;
constexpr float z_0 = 0.134;

class RobotSimNode : public rclcpp::Node
{
    public:
        //Constructor.
        RobotSimNode(const char *node_name, const char *topic_name, int it_sim, float referencia[3]);
        // Variables de estado
        vector<float>   x, y, phi;
    private:
        // Publisher
        void robot_callback();
        // Int. por timer y publisher
        rclcpp::TimerBase::SharedPtr timer_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pub;
        // Referencias 
        float           x_ref, y_ref, phi_ref;
        // Velocidades de control
        vector<float>   vu, wu;
        // Constantes de control
        const float     Kx      = 5;
        const float     Ky      = 1;
        const float     Kphi    = 2.5;
        // Tiempo de muestreo -> Equivalente a 100ms
        const float     Ts = 0.1;
};