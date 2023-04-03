cmake_minimum_required(VERSION 3.5)
project(serial_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# This package uses features from Cpp 20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(serial_bridge_actions REQUIRED)
find_package(uahrk_navigation_msgs REQUIRED)


set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  serial_bridge_actions
  rclcpp_action
  uahrk_navigation_msgs
)



set(PROTOCOL_PATH "./include/SharedKrakenUART")
FILE(GLOB PROTOCOL_SRC 
  "${PROTOCOL_PATH}/sources/*.cpp"
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/serial_bridge/SerialBridgeNode.cpp
  src/serial_bridge/DummySerialNode.cpp
  src/serial_bridge/simulate_orders.cpp
  ${PROTOCOL_SRC}
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(serial src/serial_bridge_main.cpp)
ament_target_dependencies(serial ${dependencies})
target_link_libraries(serial ${PROJECT_NAME})


add_executable(dummy_serial src/dummy_serial_main.cpp)
ament_target_dependencies(dummy_serial ${dependencies})
target_link_libraries(dummy_serial ${PROJECT_NAME}) 


install(TARGETS
  ${PROJECT_NAME}
  serial
  dummy_serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ros_testing REQUIRED)

  # No me interesa hacer tests del linter,
  # solo unitarios
  #ament_lint_auto_find_test_dependencies()

  #ament_add_gtest(some_test tests/test0.cpp ${PROTOCOL_SRC})
  
  #target_include_directories(some_test PUBLIC tests/) 


endif()



ament_package()
