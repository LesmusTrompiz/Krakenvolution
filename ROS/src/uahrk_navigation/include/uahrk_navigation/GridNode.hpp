#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "uahrk_navigation/grid.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "uahrk_navigation/pose2d.hpp"


struct xy_coords{
  int x;
  int y;

  xy_coords()                                 : x{0} , y {0} {};
  xy_coords(int _x, int _y) : x{_x}, y {_y} {};

};


struct Obstacle{
  xy_coords pose;
  xy_coords size;
  Obstacle() : pose{}, size{} {};
  Obstacle(xy_coords _pose, xy_coords _size) : pose{_pose}, size{_size} {};
  Obstacle(int _xcoord, int _ycoord, int _xsize, int _ysize) : pose{_xcoord, _ycoord}, size{_xsize, _ysize} {};

};

constexpr int ROBOT_OBSTACLE_SIZE = 300;
constexpr int OWN_ROBOT_SIZE = 399;



struct RobotObstacle : Obstacle{
  RobotObstacle() : Obstacle({}, {ROBOT_OBSTACLE_SIZE,ROBOT_OBSTACLE_SIZE}) {};
  RobotObstacle(xy_coords _pose) : Obstacle({_pose.x - (float)(ROBOT_OBSTACLE_SIZE+OWN_ROBOT_SIZE)/2, _pose.y - (float)(ROBOT_OBSTACLE_SIZE+OWN_ROBOT_SIZE)/2}, {ROBOT_OBSTACLE_SIZE+OWN_ROBOT_SIZE,ROBOT_OBSTACLE_SIZE+OWN_ROBOT_SIZE}) {};
};




class GridNode : public rclcpp::Node
{
  public:
    GridNode();

  private:

    // Publisher de grid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher;

    // Subscriber con los enemigos
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub;
    
    // Grid del campo general
    nav_msgs::msg::OccupancyGrid original_grid;

    // Grid del campo verde
    nav_msgs::msg::OccupancyGrid green_side_grid;

    // Grid del campo azul
    nav_msgs::msg::OccupancyGrid blue_side_grid;

    // Actual grid
    nav_msgs::msg::OccupancyGrid actual_grid;

    // Vector con los robots enemigos y sus time stamps
    std::vector<Obstacle> robot_obstacles;

    // Función que publica el mapa cada n_segundos
    void tick_map();
    
    // Obstacle cb
    void obstacle_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    // Timer que tickea la función tick map
    rclcpp::TimerBase::SharedPtr timer_;

    // TF things
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    Pose2d get_robot_pose();
    
    void draw_escape_path(nav_msgs::msg::OccupancyGrid &rosgrid,const Pose2d &obstacle);
    void draw_advance(nav_msgs::msg::OccupancyGrid &rosgrid,const Pose2d &pose, float distance);
};