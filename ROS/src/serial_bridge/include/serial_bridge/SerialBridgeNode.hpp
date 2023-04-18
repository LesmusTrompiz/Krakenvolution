#pragma once


// CPP Libraries
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional>
#include <iostream>
#include <array>

// ROS Libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// MSGs Libraries
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "serial_bridge_actions/action/order.hpp"
#include "uahrk_navigation_msgs/srv/set_pose2d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_srvs/srv/empty.hpp"


// Own Libraries
#include "serial_bridge/pose2d.hpp"
#include "serial_bridge/geometry_utils.hpp"

// Protocol
#include "SharedKrakenUART/protocol.hpp"


// Cuestión de legibiliad
using namespace std::chrono_literals;
using Order     = serial_bridge_actions::action::Order;
using GoalOrder = rclcpp_action::ServerGoalHandle<Order>;
using namespace std::placeholders;


class SerialBridgeNode : public rclcpp::Node
{
  public:
    SerialBridgeNode(std::string port_name);
    ~SerialBridgeNode();

  private:
        // Control cycle function, this function will be called every 500ms
    void control_cycle();

    // Service Callback
    void set_pose(const std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Request> request,
             std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Response> response);

    void stop(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    // Action server interface
    rclcpp_action::GoalResponse     handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const Order::Goal> goal);
    rclcpp_action::CancelResponse   handle_cancel(const std::shared_ptr<GoalOrder> goal_handle);
    void                            handle_accepted(const std::shared_ptr<GoalOrder> goal_handle);

    // Odom wil store the robot odometry value.
    Pose2d odom;

    // ROS Objects for interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<serial_bridge_actions::action::Order>::SharedPtr order_server;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Service<uahrk_navigation_msgs::srv::SetPose2d>::SharedPtr reset_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service;


    // Protocol utility attributes
    std::array<std::shared_ptr<GoalOrder>, uahruart::messages::ActionFinished::SIZE> m_handles;
    std::list<std::shared_ptr<GoalOrder>> m_pending_handles;
    std::thread read_thread;
    bool m_pending_last_odom = false;
};



