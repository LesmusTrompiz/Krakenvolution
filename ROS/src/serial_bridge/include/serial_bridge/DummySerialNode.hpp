#pragma once

// CPP Libraries
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional>
#include <tuple>

// ROS Libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// MSGs Libraries
#include "geometry_msgs/msg/pose.hpp"
#include "serial_bridge_actions/action/order.hpp"
#include "uahrk_navigation_msgs/srv/set_pose2d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Own Libraries
#include "serial_bridge/pose2d.hpp"
#include "serial_bridge/simulate_orders.hpp"
#include "serial_bridge/geometry_utils.hpp"



// Typedefs and using to improve readibility
typedef std::function<bool(Pose2d&,int16_t&)> fn_thread;
using Order     = serial_bridge_actions::action::Order;
using GoalOrder = rclcpp_action::ServerGoalHandle<Order>;


class DummySerialBridgeNode : public rclcpp::Node
{
  public:
    DummySerialBridgeNode(std::string port_name);
    ~DummySerialBridgeNode();


  private:
  
    // Control cycle function, this function will be called every 500ms
    void control_cycle();

    // Service Callback
    void set_pose(const std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Request> request,
             std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Response> response);
    
    // Action server interface
    rclcpp_action::GoalResponse     handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const Order::Goal> goal);
    rclcpp_action::CancelResponse   handle_cancel(const std::shared_ptr<GoalOrder> goal_handle);
    void                            handle_accepted(const std::shared_ptr<GoalOrder> goal_handle);

    // Odom wil store the robot odometry value.
    Pose2d odom;

    // This vector will store the functions that simulate the RMI behaviour..
    std::vector<std::tuple<fn_thread, int16_t>> tick_functions;
    
    // ROS Objects for interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<serial_bridge_actions::action::Order>::SharedPtr order_server;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Service<uahrk_navigation_msgs::srv::SetPose2d>::SharedPtr reset_service;
};