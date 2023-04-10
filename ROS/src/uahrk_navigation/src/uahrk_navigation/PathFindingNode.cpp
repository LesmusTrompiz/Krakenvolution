#include "uahrk_navigation/PathFindingNode.hpp"
#include <chrono>
using namespace std::chrono_literals;

PathFindingNode::PathFindingNode()
: Node("path_finding_node"){
  using namespace std::placeholders;

  goal = NULL;
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  path_client = rclcpp_action::create_client<Path>(this, "move_server");


  path_pub =  this->create_publisher<geometry_msgs::msg::PoseArray>(
    "path", 10);

  path_simplified_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "path_simplified_pub", 10);

  path_reduced_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "path_reduced", 10);

  grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "grid", 10, std::bind(&PathFindingNode::grid_cb, this, _1));


  go_to_pose_server = rclcpp_action::create_server<GoToPose>(
    this,
    "path_finding_server",
    std::bind(&PathFindingNode::handle_goal,     this, _1, _2),
    std::bind(&PathFindingNode::handle_cancel,   this, _1),
    std::bind(&PathFindingNode::handle_accepted, this, _1));

}

rclcpp_action::GoalResponse PathFindingNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToPose::Goal> goal){

  RCLCPP_INFO(this->get_logger(), 
    "Received goal request with order x: %f y: %f  ",
    goal->pose.pose.position.x,
    goal->pose.pose.position.y);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse PathFindingNode::handle_cancel(
    const std::shared_ptr<GoalHandleGoToPose> goal_handle){
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

bool inside_grid(Pose2d pose, const nav_msgs::msg::OccupancyGrid &grid){
  return (
       pose.x >= 0 
    && pose.y >= 0 
    && (pose.x / grid.info.resolution) < grid.info.width
    && (pose.y / grid.info.resolution) < grid.info.height);
}

void PathFindingNode::handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle){
  // Update the actual handle variable an
  // acomplish the goal in control_cycle
  auto robot_pose = get_robot_pose();
  const auto ros_goal = goal_handle->get_goal()->pose.pose;
  Pose2d goal_pose{ros_goal};
  auto result = std::make_shared<GoToPose::Result>();


  if (!inside_grid(goal_pose,grid)){
    goal_handle->abort(result);
    return;
  }

  try{
    RCLCPP_INFO(this->get_logger(), "Handel accepted Pose %f - %f - %f", goal_pose.x, goal_pose.y , goal_pose.a);
    auto path = Astar(pose2dintcell(robot_pose,grid), pose2dintcell(goal_pose,grid),grid);
    path_pub->publish(intpath2rospath(path,grid));
    auto discretized_path = discretize_path(path);
    path_simplified_pub->publish(intpath2rospath(discretized_path,grid));
    auto reduced_path = reduce_by_slopes(discretized_path, grid);
    auto final_path = intpath2rospath(reduced_path,grid);
    final_path.poses.back().orientation = ros_goal.orientation;
    path_reduced_pub->publish(final_path);
    
    auto send_goal_options = rclcpp_action::Client<Path>::SendGoalOptions();
    auto goal_msg = Path::Goal();
    goal_msg.pose = final_path;
    send_goal_options.result_callback = std::bind(&PathFindingNode::result_callback, this, std::placeholders::_1);

    path_client->async_send_goal(goal_msg,send_goal_options);
    goal = std::move(goal_handle);
  }
  catch(...){
    goal_handle->abort(result);
  }
}



void PathFindingNode::grid_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  //RCLCPP_INFO(this->get_logger(), "Grid received");
  
  // Store the grid
  grid = *msg;
}


Pose2d PathFindingNode::get_robot_pose(){
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped robot_tf = tf_buffer_->lookupTransform(
      "map", "robot", now, 100ms);
    return {robot_tf};
}

void PathFindingNode::result_callback(const RequestHandlePath::WrappedResult & result){
  RCLCPP_INFO(this->get_logger(), "Result Code %d", result.code);
  if(goal != NULL){
    auto result = std::make_shared<GoToPose::Result>();

    goal->succeed(result);
    goal = NULL;
  }

}

