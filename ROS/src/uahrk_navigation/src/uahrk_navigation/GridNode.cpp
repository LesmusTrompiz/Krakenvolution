#include "uahrk_navigation/GridNode.hpp"
using namespace std::chrono_literals;


nav_msgs::msg::OccupancyGrid eurobotgrid_to_rosgrid(const eurobot_grid &eurogrid){
  nav_msgs::msg::OccupancyGrid rosgrid;
  rosgrid.data.reserve(eurogrid.width * eurogrid.height);
  rosgrid.data.resize(eurogrid.width * eurogrid.height);

  rosgrid.info.width      = eurogrid.width;
  rosgrid.info.height     = eurogrid.height;
  rosgrid.info.resolution = (float)PLAYGROUND_WIDTH / (float)eurogrid.width;
  rosgrid.header.frame_id = "map";

  std::copy(eurogrid.cells, eurogrid.cells + (eurogrid.width * eurogrid.height) , rosgrid.data.begin());
  std::cout<< "vector size " << rosgrid.data.size() << std::endl;
  return rosgrid;
}
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
  RCLCPP_INFO(this->get_logger(), "Publishing: grid");
  grid_publisher->publish(eurobotgrid_to_rosgrid(grid));
}

