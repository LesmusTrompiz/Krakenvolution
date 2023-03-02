cmake_minimum_required(VERSION 3.5)
project(uahrk_decision_making)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uahrk_navigation_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ZMQ)
if(ZMQ_FOUND)
  message(STATUS "ZeroMQ found.")
  add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(dependencies
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  ament_index_cpp
  uahrk_navigation_msgs
  rclcpp_action
)
include_directories(include ${ZMQ_INCLUDE_DIRS})
add_library(hello_bt_node SHARED src/uahrk_decision_making/HelloNode.cpp)
add_library(gotopose_bt_node SHARED src/uahrk_decision_making/GoToPoseBTNode.cpp)


list(APPEND plugin_libs
  hello_bt_node
  gotopose_bt_node
  #br2_back_bt_node
  #br2_turn_bt_node
  #br2_is_obstacle_bt_node
)
foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()
  
add_executable(decision src/decision_main.cpp)
ament_target_dependencies(decision ${dependencies})
target_link_libraries(decision ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  decision
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#  find_package(ament_cmake_gtest REQUIRED)
#  add_subdirectory(tests)
#endif()
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()

