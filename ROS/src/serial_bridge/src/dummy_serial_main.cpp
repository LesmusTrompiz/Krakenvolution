#include "rclcpp/rclcpp.hpp"
#include "serial_bridge/DummySerialNode.hpp"



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySerialBridgeNode>("hola"));
  rclcpp::shutdown();
  return 0;
}







