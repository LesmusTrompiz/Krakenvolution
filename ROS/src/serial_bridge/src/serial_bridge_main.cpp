#include "rclcpp/rclcpp.hpp"
#include "serial_bridge/SerialBridgeNode.hpp"



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>("hola"));
  rclcpp::shutdown();
  return 0;
}







