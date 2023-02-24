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



inline float mm_to_grid_resolution(const int mms, const float grid_resolution){
  return ((float)mms / 1000.0) / grid_resolution; // Grid resolution is in m/Cell_size
}


void draw_obstacle(nav_msgs::msg::OccupancyGrid &rosgrid, Obstacle obstacle){
  auto x_grid_start = obstacle.pose.x;
  auto y_grid_start = obstacle.pose.y;
  auto x_grid_end   = obstacle.pose.x + obstacle.size.x;
  auto y_grid_end   = obstacle.pose.y + obstacle.size.y;

  // Check grid limits
  if(x_grid_end >= rosgrid.info.width)  x_grid_end = rosgrid.info.width - 1; 
  if(y_grid_end >= rosgrid.info.height) y_grid_end = rosgrid.info.height - 1; 
  
  for(auto y = y_grid_start; y <= y_grid_end; y++){
    for(auto x = x_grid_start; x <= x_grid_end; x++){
      rosgrid.data[x + y*rosgrid.info.width] = 100;
    }
  }
}

void initialize_rosgrid(nav_msgs::msg::OccupancyGrid &rosgrid){
  // Initialize header
  rosgrid.header.frame_id = "map";
  
  // Initialize Grid info
  rosgrid.info.width      = EurobotGridWidth;
  rosgrid.info.height     = EurobotGridHeight;
  rosgrid.info.resolution = EurobotGridResolution;
  
  // Initialize grid cells
  rosgrid.data.reserve(rosgrid.info.width * rosgrid.info.height);
  rosgrid.data.resize(rosgrid.info.width  * rosgrid.info.height);
  std::fill(rosgrid.data.begin(), rosgrid.data.end(),0);
}


inline Obstacle mm_obstacle_to_grid_units(const Obstacle &obstacle_in_mm, const float grid_resolution){

  return {mm_to_grid_resolution(obstacle_in_mm.pose.x,grid_resolution),
          mm_to_grid_resolution(obstacle_in_mm.pose.y,grid_resolution),
          round(mm_to_grid_resolution(obstacle_in_mm.size.x,grid_resolution)),
          round(mm_to_grid_resolution(obstacle_in_mm.size.y,grid_resolution))
  };
}



GridNode::GridNode()
: Node("grid_node")
{
  using namespace std::placeholders;
  
  //reset_service = this->create_service<uahrk_navigation_msgs::srv::SetPose2d>("set_pose", 
  //    std::bind(&GridNode::set_pose, this, _1 ,_2));
  initialize_rosgrid(original_grid);

  std::vector<Obstacle> init_obstacles;

  const Obstacle sujeta_cerezas_sur   = mm_obstacle_to_grid_units({ { 999 - 15 ,          0}, { 30, 300}}, EurobotGridResolution);
  const Obstacle sujeta_cerezas_norte = mm_obstacle_to_grid_units({ { 999 - 15 , 2999 - 300}, { 30, 300}}, EurobotGridResolution);
  const Obstacle sujeta_cerezas_este  = mm_obstacle_to_grid_units({ {1999 - 30 , 1499 - 150}, { 30, 300}}, EurobotGridResolution);
  const Obstacle sujeta_cerezas_oeste = mm_obstacle_to_grid_units({ {        0 , 1499 - 150}, { 30, 300}}, EurobotGridResolution);

  init_obstacles.push_back(sujeta_cerezas_sur);
  init_obstacles.push_back(sujeta_cerezas_este);
  init_obstacles.push_back(sujeta_cerezas_oeste);
  init_obstacles.push_back(sujeta_cerezas_norte);

  for(const auto &obstacle: init_obstacles){
    draw_obstacle(original_grid, obstacle);
  } 

  const Obstacle Mantel_azul {mm_obstacle_to_grid_units({{0 , 2999 - 450}, {900, 450}}, EurobotGridResolution)};
  
  green_side_grid = original_grid;
  draw_obstacle(green_side_grid, Mantel_azul);
  
  Obstacle Mantel_verde {mm_obstacle_to_grid_units({{1999 - 1000, 2999 - 450}, {1000, 450}}, EurobotGridResolution)};
  blue_side_grid = original_grid;
  draw_obstacle(blue_side_grid, Mantel_verde);

  grid_publisher       = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid", 10);

  obstacle_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "obstacles", 10, std::bind(&GridNode::obstacle_cb, this, _1));

  this->declare_parameter("play_side", "blue");
  
  timer_               = this->create_wall_timer(500ms, std::bind(&GridNode::tick_map, this));
}


inline bool obstacle_in_playground(const Obstacle &new_obstacle,const nav_msgs::msg::OccupancyGrid &playground_grid){
  return (0 <= new_obstacle.pose.x && new_obstacle.pose.x < playground_grid.info.width
          && 
          0 <= new_obstacle.pose.y && new_obstacle.pose.y < playground_grid.info.height);
}

inline bool ros_obstacle_in_play_ground(const geometry_msgs::msg::Pose ros_obstacle){
    return (0 <= ros_obstacle.position.x && ros_obstacle.position.x < PLAYGROUND_WIDTH
            && 
            0 <= ros_obstacle.position.y && ros_obstacle.position.y < PLAYGROUND_HEIGHT);

}

inline int ros_units_to_grid_units(const float &meters, const nav_msgs::msg::OccupancyGrid &playground_grid){
  return meters / EurobotGridResolution;
}

void GridNode::obstacle_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg){
  //RCLCPP_INFO(this->get_logger(), "New grid obstacle: '%f'", msg->poses[0].position.x);
  

  robot_obstacles.clear();

  for(auto const &ros_obstacle : msg->poses){
    // Check if the obstacle is inside the play ground
    if(!ros_obstacle_in_play_ground(ros_obstacle)) continue;
    // Transform bewteen the obstacle fram ID and the map id.
    RobotObstacle new_obstacle{xy_coords{
      ros_units_to_grid_units(ros_obstacle.position.x , original_grid),
      ros_units_to_grid_units(ros_obstacle.position.y , original_grid),
      }};
    
    robot_obstacles.push_back(new_obstacle);
  }
}



void GridNode::tick_map(){
  // Update the play side
  std::string play_side =
    this->get_parameter("play_side").get_parameter_value().get<std::string>();

  // Take the grid according to the play side
  if(play_side == "green"){
    actual_grid = green_side_grid;
  } else if(play_side == "blue"){
    actual_grid = blue_side_grid;
  } else{
    actual_grid = original_grid;
  }
  
  // Draw the obstacles
  for(const auto &obstacle : robot_obstacles){
    draw_obstacle(actual_grid, obstacle);
  }

  // Update the header time
  actual_grid.header.stamp = this->get_clock()->now();

  // Publish the enemies
  grid_publisher->publish(actual_grid);
}

