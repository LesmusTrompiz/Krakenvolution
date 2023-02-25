#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "uahrk_navigation/grid.hpp"



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

constexpr int ROBOT_SIZE = 3;
struct RobotObstacle : Obstacle{
  RobotObstacle() : Obstacle({}, {ROBOT_SIZE,ROBOT_SIZE}) {};
  RobotObstacle(xy_coords _pose) : Obstacle({_pose.x - (float)ROBOT_SIZE/2, _pose.y - (float)ROBOT_SIZE/2}, {ROBOT_SIZE,ROBOT_SIZE}) {};
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
    std::vector<RobotObstacle> robot_obstacles;

    // Función que publica el mapa cada n_segundos
    void tick_map();
    
    // Obstacle cb
    void obstacle_cb(const geometry_msgs::msg::PoseArray::SharedPtr msg);

    // Timer que tickea la función tick map
    rclcpp::TimerBase::SharedPtr timer_;

};