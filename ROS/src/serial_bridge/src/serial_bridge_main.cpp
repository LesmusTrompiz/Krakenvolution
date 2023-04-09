#include "rclcpp/rclcpp.hpp"
#include "serial_bridge/SerialBridgeNode.hpp"

#include <string>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string port = "hola";
  // Check if a serial port has been passed through the command line
  if (argc >= 2)
    port = std::string(argv[1]);

  rclcpp::spin(std::make_shared<SerialBridgeNode>(port.c_str()));
  rclcpp::shutdown();
  return 0;
}







