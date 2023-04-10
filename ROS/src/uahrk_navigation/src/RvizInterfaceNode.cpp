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
#include "uahrk_navigation_msgs/srv/set_pose2d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/pose_array.hpp"




using namespace std::chrono_literals;
using GoToPose  = uahrk_navigation_msgs::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;


enum RVIZ_MODES {NAV_GOAL, GRID};
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class RvizInterfaceNode : public rclcpp::Node
{
  public:
    RvizInterfaceNode()
    : Node("rviz_interface")
    {
      mode = NAV_GOAL;
      set_pose_client = this->create_client<uahrk_navigation_msgs::srv::SetPose2d>("set_pose");
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                  "initialpose", 10,
                std::bind(&RvizInterfaceNode::pose_estimate_callback, this, std::placeholders::_1));
      subscription2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                  "goal_pose", 10,
                std::bind(&RvizInterfaceNode::goal_pose, this, std::placeholders::_1));

      obstacles_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("obstacles", 10);
      client_ptr_ = rclcpp_action::create_client<GoToPose>(this, "move_server");
      //client_ptr_ = rclcpp_action::create_client<GoToPose>(this, "path_finding_server");
      
      timer_ = this->create_wall_timer(500ms, std::bind(&RvizInterfaceNode::timer_callback, this));
    }

  private:
    RVIZ_MODES mode;
    geometry_msgs::msg::PoseArray obstacles;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_publisher;
    void pose_estimate_callback(geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr rviz_pose)
    {
      tf2::Quaternion q(
        rviz_pose->pose.pose.orientation.x,
        rviz_pose->pose.pose.orientation.y,
        rviz_pose->pose.pose.orientation.z,
        rviz_pose->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      auto request = std::make_shared<uahrk_navigation_msgs::srv::SetPose2d::Request>();
      request->x = rviz_pose->pose.pose.position.x;
      request->y = rviz_pose->pose.pose.position.y;
      request->a = yaw * 180 / M_PI;
      set_pose_client->async_send_request(request);
      return;
    }

    void goal_pose(geometry_msgs::msg::PoseStamped::UniquePtr rviz_pose){
      using namespace std::placeholders;
      auto goal_msg = GoToPose::Goal();
      auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();

      switch (mode){
        case NAV_GOAL:
          RCLCPP_INFO(this->get_logger(), "Sending goal: '%f'", rviz_pose->pose.position.x);
          goal_msg.pose = *rviz_pose;

          send_goal_options.goal_response_callback = std::bind(&RvizInterfaceNode::goal_response_callback, this, _1);
          send_goal_options.result_callback = std::bind(&RvizInterfaceNode::result_callback, this, _1);
          client_ptr_->async_send_goal(goal_msg, send_goal_options);
          break;
        
        case GRID:
          obstacles.poses.push_back(rviz_pose->pose);
          RCLCPP_INFO(this->get_logger(), "New grid obstacle: '%f'", rviz_pose->pose.position.x);
          break;
        default:
          break;
      }
      return;
    }

    void goal_response_callback(std::shared_future<GoalHandleGoToPose::SharedPtr> future){
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }
    void result_callback(const GoalHandleGoToPose::WrappedResult & result){
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

    void timer_callback(){
      obstacles_publisher->publish(obstacles);
      return;
    }

    rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription2_;
    rclcpp::Client<uahrk_navigation_msgs::srv::SetPose2d>::SharedPtr set_pose_client;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_enemies;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}