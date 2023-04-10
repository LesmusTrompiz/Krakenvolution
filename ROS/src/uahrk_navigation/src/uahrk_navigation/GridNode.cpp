#include "uahrk_navigation/GridNode.hpp"
using namespace std::chrono_literals;
extern "C"{
    #include <math.h>
}

constexpr float ROBOT_WIDTH = 300.0;


// Función que convierte de grados a radianes.
inline float DEG2RAD(const float deg)
{
  return deg* M_PI / 180;
}


nav_msgs::msg::OccupancyGrid eurobotgrid_to_rosgrid(const eurobot_grid &eurogrid){
  nav_msgs::msg::OccupancyGrid rosgrid;
  rosgrid.data.reserve(eurogrid.width * eurogrid.height);
  rosgrid.data.resize(eurogrid.width * eurogrid.height);

  rosgrid.info.width      = eurogrid.width;
  rosgrid.info.height     = eurogrid.height;
  rosgrid.info.resolution = (float)PLAYGROUND_WIDTH / (float)eurogrid.width;
  rosgrid.header.frame_id = "map";

  std::copy(eurogrid.cells, eurogrid.cells + (eurogrid.width * eurogrid.height) , rosgrid.data.begin());
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
  if(x_grid_start < 0) x_grid_start = 0;
  if(y_grid_start < 0) y_grid_start = 0;
  if(x_grid_end >= rosgrid.info.width)  x_grid_end = rosgrid.info.width - 1; 
  if(y_grid_end >= rosgrid.info.height) y_grid_end = rosgrid.info.height - 1; 
  
  for(auto y = y_grid_start; y <= y_grid_end; y++){
    for(auto x = x_grid_start; x <= x_grid_end; x++){
      rosgrid.data[x + y*rosgrid.info.width] = 100;
    }
  }
}

void fill_space_between_objects(nav_msgs::msg::OccupancyGrid &rosgrid, float distance){
  int max_distance  = distance / rosgrid.info.resolution;
  int x = 1;
  int y = 0;

  for(int actual_cell = 1; actual_cell < rosgrid.data.size(); actual_cell++){

    if(rosgrid.data[actual_cell] == 100){
      x++;
      y  += x / rosgrid.info.width;
      x   = x % rosgrid.info.width;
      continue;
    }

    // Check x
    if(x > 1){
      if(rosgrid.data[actual_cell - 1] == 100){
        for(int next_x = 1; (next_x < max_distance && next_x < rosgrid.info.width - x) ; next_x++){
          if(rosgrid.data[actual_cell + next_x ] == 100){
            rosgrid.data[actual_cell] = 100;
            continue;
          }
        }
      }
    }

    // Can check y limits;
    if(rosgrid.data[actual_cell]){
      x++;
      y  += x / rosgrid.info.width;
      x   = x % rosgrid.info.width;
      continue;
    }

    if(y > 1){
      if(rosgrid.data[actual_cell - rosgrid.info.width]){
        for(int next_y = 1; (next_y < max_distance && next_y < rosgrid.info.height - y); next_y++){
          if(rosgrid.data[actual_cell + next_y * rosgrid.info.width]){
            rosgrid.data[actual_cell] = 100;
            continue;
          }
        }
      }
    }
    x++;
    y  += x / rosgrid.info.width;
    x   = x % rosgrid.info.width;
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
  int start_x = mm_to_grid_resolution(obstacle_in_mm.pose.x,grid_resolution);
  int start_y = mm_to_grid_resolution(obstacle_in_mm.pose.y,grid_resolution);
  int end_x   = mm_to_grid_resolution(obstacle_in_mm.pose.x + obstacle_in_mm.size.x, grid_resolution);
  int end_y   = mm_to_grid_resolution(obstacle_in_mm.pose.y + obstacle_in_mm.size.y, grid_resolution);
  return {start_x,
          start_y,
          end_x - start_x,
          end_y - start_y
  };
}

Pose2d pose_if_advance(const Pose2d &start, float distance){
  Pose2d new_pose = start;
  // Calculo la nueva posición del robot
  new_pose.x += distance * cos(DEG2RAD(start.a));
  new_pose.y += distance * sin(DEG2RAD(start.a));
  return new_pose;
}

Pose2d pose_if_spin(const Pose2d &start, float spin){
  Pose2d new_pose = start;
  // Calculo la nueva posición del robot
  new_pose.a += spin ;

  if(new_pose.a < 0){
    new_pose.a += 360;
  }
  else if(new_pose.a > 360){
    new_pose.a -= 360;
  }
  return new_pose;
}

inline bool in_map(const Pose2d &pose){
  return(
      0 < pose.x 
      &&
      pose.x < PLAYGROUND_WIDTH
      &&
      0 < pose.y
      &&
      pose.y < PLAYGROUND_HEIGHT);
}


inline int get_cell_index_from_pose2d(const Pose2d &pose, const nav_msgs::msg::OccupancyGrid &rosgrid){
  int x_index = mm_to_grid_resolution(pose.x * 1000,rosgrid.info.resolution);
  int y_index = mm_to_grid_resolution(pose.y * 1000,rosgrid.info.resolution);
  return x_index + y_index * rosgrid.info.width;
}

inline bool occupied_cell(const Pose2d &pose, const nav_msgs::msg::OccupancyGrid &rosgrid){
  return rosgrid.data[get_cell_index_from_pose2d(pose,rosgrid)] == 100;
}

    
void draw_advance
(nav_msgs::msg::OccupancyGrid &rosgrid,const Pose2d &pose, float distance){
  auto new_pose = pose;
  auto dx = cos(DEG2RAD(pose.a)) / 10;
  auto dy = sin(DEG2RAD(pose.a)) / 10;

  if (distance<0){
    dx = -dx;
    dy = -dy;
  } 
  float advanced_distance = 0;

  while(advanced_distance < abs(distance)){
    rosgrid.data[get_cell_index_from_pose2d(new_pose,rosgrid)] = 0;
    new_pose.x += dx;
    new_pose.y += dy;
    advanced_distance += abs(dx) + abs(dy);
  }
  rosgrid.data[get_cell_index_from_pose2d(new_pose,rosgrid)] = 0;
  return;
}

bool escape_advancing(nav_msgs::msg::OccupancyGrid &rosgrid,const Pose2d &robot_pose, int max_escape_distance){
  for(int distance_dm = 1; distance_dm <= max_escape_distance ; distance_dm++){
    Pose2d advance_escape = pose_if_advance(robot_pose,  (float)distance_dm / 10);
    Pose2d back_escape    = pose_if_advance(robot_pose, -(float)distance_dm / 10);

    if(in_map(advance_escape)&&!occupied_cell(advance_escape, rosgrid)){
      draw_advance(rosgrid, robot_pose,(float)distance_dm / 10);
      return true;
    }
    else if(in_map(back_escape)&&!occupied_cell(back_escape, rosgrid)){
      draw_advance(rosgrid, robot_pose,-(float)distance_dm / 10);
      return true;
    }
  }
  return false;
}




void GridNode::draw_escape_path
(nav_msgs::msg::OccupancyGrid &rosgrid,const Pose2d &robot_pose){  
  // Si avanzo 30 dm o retrocedo 30 dm salgo del marrón?
  //scape
  if(escape_advancing(rosgrid,robot_pose,3)) return;

  // If robot in corner... retroceder

  // Try spining:
  auto robot_pose_modified = robot_pose;
  for(int angle_increment = 1; angle_increment <=3 ; angle_increment++){
    robot_pose_modified.a = robot_pose.a + angle_increment * 20;
    if(escape_advancing(rosgrid,robot_pose_modified,3)) return;

    robot_pose_modified.a = robot_pose.a - angle_increment * 20;
    if(escape_advancing(rosgrid,robot_pose_modified,3)) return;
  }
}

Pose2d GridNode::get_robot_pose(){
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped robot_tf = tf_buffer_->lookupTransform(
      "map", "robot", now, 100ms);
    return {robot_tf};
}


GridNode::GridNode()
: Node("grid_node")
{
  using namespace std::placeholders;
  
  //reset_service = this->create_service<uahrk_navigation_msgs::srv::SetPose2d>("set_pose", 
  //    std::bind(&GridNode::set_pose, this, _1 ,_2));
  initialize_rosgrid(original_grid);
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::vector<Obstacle> init_obstacles;

  const Obstacle sujeta_cerezas_sur   = mm_obstacle_to_grid_units({{ 999 - 15 - OWN_ROBOT_SIZE/2,                             0}, {30 +   OWN_ROBOT_SIZE, 300 + OWN_ROBOT_SIZE/2}}, EurobotGridResolution);
  const Obstacle sujeta_cerezas_norte = mm_obstacle_to_grid_units({{ 999 - 15 - OWN_ROBOT_SIZE/2, 2999 - 300 - OWN_ROBOT_SIZE/2}, {30 +   OWN_ROBOT_SIZE, 300 + OWN_ROBOT_SIZE}}, EurobotGridResolution);
  const Obstacle sujeta_cerezas_este  = mm_obstacle_to_grid_units({{1999 - 30 - OWN_ROBOT_SIZE/2, 1499 - 150 - OWN_ROBOT_SIZE/2}, {30 +   OWN_ROBOT_SIZE, 300 + OWN_ROBOT_SIZE}}, EurobotGridResolution);
  const Obstacle sujeta_cerezas_oeste = mm_obstacle_to_grid_units({{                           0, 1499 - 150 - OWN_ROBOT_SIZE/2}, {30 + OWN_ROBOT_SIZE/2, 300 + OWN_ROBOT_SIZE}}, EurobotGridResolution);


  // Paredes
  const Obstacle pared_sur   = mm_obstacle_to_grid_units({{0                      ,                       0}, {            1999, OWN_ROBOT_SIZE/2}}, EurobotGridResolution);
  const Obstacle pared_norte = mm_obstacle_to_grid_units({{0                      , 2999 - OWN_ROBOT_SIZE/2}, {            1999, OWN_ROBOT_SIZE/2}}, EurobotGridResolution);
  const Obstacle pared_oeste = mm_obstacle_to_grid_units({{0                      ,                       0}, {OWN_ROBOT_SIZE/2,             2999}}, EurobotGridResolution);
  const Obstacle pared_este  = mm_obstacle_to_grid_units({{1999 - OWN_ROBOT_SIZE/2,                       0}, {OWN_ROBOT_SIZE/2,             2999}}, EurobotGridResolution);
  

  init_obstacles.push_back(sujeta_cerezas_sur);
  init_obstacles.push_back(sujeta_cerezas_este);
  init_obstacles.push_back(sujeta_cerezas_oeste);
  init_obstacles.push_back(sujeta_cerezas_norte);
  init_obstacles.push_back(pared_sur);
  init_obstacles.push_back(pared_norte);
  init_obstacles.push_back(pared_oeste);
  init_obstacles.push_back(pared_este);

  for(const auto &obstacle: init_obstacles){
    draw_obstacle(original_grid, obstacle);
  } 
  const Obstacle Mantel_azul {mm_obstacle_to_grid_units({{0, 2999 - 450 - OWN_ROBOT_SIZE/2}, {1000 + OWN_ROBOT_SIZE/2, 450 + OWN_ROBOT_SIZE}}, EurobotGridResolution)};
  
  green_side_grid = original_grid;
  draw_obstacle(green_side_grid, Mantel_azul);
  
  Obstacle Mantel_verde {mm_obstacle_to_grid_units({{1999 - 1000 - OWN_ROBOT_SIZE/2, 2999 - 450 - OWN_ROBOT_SIZE/2}, {1000 + OWN_ROBOT_SIZE, 450 + OWN_ROBOT_SIZE}}, EurobotGridResolution)};
  blue_side_grid = original_grid;
  draw_obstacle(blue_side_grid, Mantel_verde);

  grid_publisher       = this->create_publisher<nav_msgs::msg::OccupancyGrid>("grid", 10);

  obstacle_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "obstacles", 10, std::bind(&GridNode::obstacle_cb, this, _1));

  this->declare_parameter("play_side", "default");
  timer_ = this->create_wall_timer(500ms, std::bind(&GridNode::tick_map, this));
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
  robot_obstacles.clear();
  for(auto const &ros_obstacle : msg->poses){
    // Check if the obstacle is inside the play ground
    if(!ros_obstacle_in_play_ground(ros_obstacle)) continue;
    // Transform bewteen the obstacle fram ID and the map id.
    RobotObstacle new_obstacle{xy_coords{
      ros_obstacle.position.x*1000,
      ros_obstacle.position.y*1000}};
    robot_obstacles.push_back(mm_obstacle_to_grid_units(new_obstacle,EurobotGridResolution));
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

  // Try to get the robot pose
  try{
    auto pose = get_robot_pose();

    // If the robot is in danger try a scape route
    if(occupied_cell(pose,actual_grid)){
      draw_escape_path(actual_grid,pose);
    }
  }
  catch (tf2::TransformException & ex){
    RCLCPP_WARN(get_logger(), "Robot transform not found: %s", ex.what());
  }

  // Fill with black if two obstacles are extreamly near
  //fill_space_between_objects(actual_grid, 0.4);

  // Update the header time
  actual_grid.header.stamp = this->get_clock()->now();

  // Publish the enemies
  grid_publisher->publish(actual_grid);
}

