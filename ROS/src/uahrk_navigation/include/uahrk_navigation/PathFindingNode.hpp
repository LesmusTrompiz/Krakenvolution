#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "uahrk_navigation/grid.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uahrk_navigation_msgs/action/go_to_pose.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "uahrk_navigation/pose2d.hpp"
#include "uahrk_navigation/astar_ros.hpp"
#include "uahrk_navigation_msgs/action/path.hpp"



using GoToPose           = uahrk_navigation_msgs::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

using Path     = uahrk_navigation_msgs::action::Path;
using RequestHandlePath = rclcpp_action::ClientGoalHandle<Path>;


class PathFindingNode : public rclcpp::Node
{
  public:
    PathFindingNode();

  private:
    // Action server
    rclcpp_action::Server<GoToPose>::SharedPtr go_to_pose_server;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GoToPose::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleGoToPose> goal_handle);

    void handle_accepted(
      const std::shared_ptr<GoalHandleGoToPose> goal_handle);  
  
    //
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Action client
    rclcpp_action::Client<Path>::SharedPtr    path_client;

    // Pub path for debug purpose
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_simplified_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_reduced_pub;


    // Subscriber del grid
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub;
    
    // Grid del campo general
    nav_msgs::msg::OccupancyGrid grid;
  
    // Obstacle cb
    void grid_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    std::shared_ptr<GoalHandleGoToPose> goal;
    Pose2d get_robot_pose();

    void result_callback(const RequestHandlePath::WrappedResult & result);
    void goal_response_callback(std::shared_future<RequestHandlePath::SharedPtr> future);
};