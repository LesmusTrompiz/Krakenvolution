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


using namespace std::chrono_literals;


class SerialBridgeNode : public rclcpp::Node
{
  public:
    using Order     = serial_bridge_actions::action::Order;
    using GoalOrder = rclcpp_action::ServerGoalHandle<Order>;
    SerialBridgeNode(std::string port_name);
    ~SerialBridgeNode();
    rclcpp_action::GoalResponse     handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const Order::Goal> goal);
    rclcpp_action::CancelResponse   handle_cancel(const std::shared_ptr<GoalOrder> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalOrder> goal_handle);

    //?api::Instance protocol;

  private:
    void timer_callback();
    rclcpp_action::Server<serial_bridge_actions::action::Order>::SharedPtr order_server;
};



