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
find_package(rclcpp_action REQUIRED)
find_package(serial_bridge_actions REQUIRED)






set(PROTOCOL_PATH "./include/SharedKrakenUART")
FILE(GLOB PROTOCOL_SRC 
  "${PROTOCOL_PATH}/sources/*.cpp"
)

FILE(GLOB PROTOCOL_HEADERS
  "${PROTOCOL_PATH}/headers/*.hpp"
)


include_directories(
  "${PROTOCOL_PATH}/headers/"
)

add_executable(serial src/serial_bridge_node.cpp ${PROTOCOL_SRC})
target_link_libraries(serial)
ament_target_dependencies(serial rclcpp std_msgs rclcpp_action serial_bridge_actions)

install(TARGETS
  serial
  DESTINATION lib/${PROJECT_NAME})

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
