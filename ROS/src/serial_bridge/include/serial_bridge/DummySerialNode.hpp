#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "serial_bridge_actions/action/order.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "uahrk_navigation_msgs/srv/set_pose2d.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "serial_bridge/pose2d.hpp"
#include "serial_bridge/simulate_orders.hpp"




class DummySerialBridgeNode : public rclcpp::Node
{
  public:
    using Order     = serial_bridge_actions::action::Order;
    using GoalOrder = rclcpp_action::ServerGoalHandle<Order>;
    DummySerialBridgeNode(std::string port_name);
    ~DummySerialBridgeNode();
    rclcpp_action::GoalResponse     handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const Order::Goal> goal);
    rclcpp_action::CancelResponse   handle_cancel(const std::shared_ptr<GoalOrder> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalOrder> goal_handle);

  private:
    void control_cycle();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<serial_bridge_actions::action::Order>::SharedPtr order_server;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Service<uahrk_navigation_msgs::srv::SetPose2d>::SharedPtr reset_service;
    void set_pose(const std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Request> request,
             std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Response> response);

    Pose2d odom;
};