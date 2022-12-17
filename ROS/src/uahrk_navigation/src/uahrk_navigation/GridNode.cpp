#include "uahrk_navigation/GridNode.hpp"
using namespace std::chrono_literals;

GridNode::GridNode()
: Node("grid_node")
{
  using namespace std::placeholders;
  
  //reset_service = this->create_service<uahrk_navigation_msgs::srv::SetPose2d>("set_pose", 
  //    std::bind(&GridNode::set_pose, this, _1 ,_2));
  
  
  grid_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid", 10);
  timer_         = this->create_wall_timer(500ms, std::bind(&GridNode::tick_map, this));

}


void GridNode::tick_map(){
  nav_msgs::msg::OccupancyGrid grid;
  RCLCPP_INFO(this->get_logger(), "Publishing: grid");
  grid_publisher->publish(grid);
}

