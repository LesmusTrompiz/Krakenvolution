#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "uahrk_navigation_msgs/action/go_to_pose.hpp"



using namespace std::chrono_literals;
using GoToPose  = uahrk_navigation_msgs::action::GoToPose;

using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RvizInterfaceNode : public rclcpp::Node
{
  public:
    RvizInterfaceNode()
    : Node("rviz_interface")
    {
      publisher_    = this->create_publisher<geometry_msgs::msg::Pose>("robot_odom", 10);
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                  "initialpose", 10,
                std::bind(&RvizInterfaceNode::pose_estimate_callback, this, std::placeholders::_1));
      subscription2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                  "goal_pose", 10,
                std::bind(&RvizInterfaceNode::goal_pose, this, std::placeholders::_1));
      client_ptr_ = rclcpp_action::create_client<GoToPose>(this, "move_server");

    }

  private:
    void pose_estimate_callback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr rviz_pose)
    {
      geometry_msgs::msg::Pose p;
      p.position    = rviz_pose->pose.pose.position;
      p.orientation = rviz_pose->pose.pose.orientation;
      publisher_->publish(p);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", p.position.x);
      return;
    }

    void goal_pose(geometry_msgs::msg::PoseStamped::UniquePtr rviz_pose)
    {
      using namespace std::placeholders;
      RCLCPP_INFO(this->get_logger(), "Sending goal: '%f'", rviz_pose->pose.position.x);
      auto goal_msg = GoToPose::Goal();
      goal_msg.pose = *rviz_pose;

      auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&RvizInterfaceNode::goal_response_callback, this, _1);
      send_goal_options.result_callback = std::bind(&RvizInterfaceNode::result_callback, this, _1);
      client_ptr_->async_send_goal(goal_msg, send_goal_options);
      return;
    }

    void goal_response_callback(std::shared_future<GoalHandleGoToPose::SharedPtr> future)
    {
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }
    void result_callback(const GoalHandleGoToPose::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      std::stringstream ss;
      ss << "Result received: " << result.result->result.data;
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}