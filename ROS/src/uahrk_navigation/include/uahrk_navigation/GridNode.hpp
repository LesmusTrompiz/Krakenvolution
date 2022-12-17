#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "uahrk_navigation/grid.hpp"

class GridNode : public rclcpp::Node
{
  public:
    GridNode();

  private:
    // Publisher de grid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher;

    // Subscriber con los enemigos

    // Grid del campo
    eurobot_grid grid;

    // Vector con los robots enemigos y sus time stamps

    // Función que publica el mapa cada n_segundos
    void tick_map();
    
    // Timer que tickea la función tick map
    rclcpp::TimerBase::SharedPtr timer_;


};