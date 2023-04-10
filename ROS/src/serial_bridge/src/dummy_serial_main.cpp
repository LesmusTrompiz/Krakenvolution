#include "rclcpp/rclcpp.hpp"
#include "serial_bridge/DummySerialNode.hpp"

/*
  Este ejecutable unicamente se encaga de lanzar
  el DummySerialBrigeNode y spinearlo. No hace ninguna
  clase de composición.
*/


int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummySerialBridgeNode>("hola"));
  rclcpp::shutdown();
  return 0;
}







