cmake_minimum_required(VERSION 3.5)
project(uahrk_navigation)


set(CMAKE_CXX_STANDARD 17)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial_bridge_actions REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(uahrk_navigation_msgs REQUIRED)



set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  tf2 
  tf2_ros
  serial_bridge_actions
  rclcpp_action
  uahrk_navigation_msgs
)

include_directories(include)


add_library(${PROJECT_NAME} SHARED
  src/uahrk_navigation/MoveToPoseNode.cpp
  src/uahrk_navigation/RobotTFNode.cpp
  src/uahrk_navigation/GridNode.cpp
  src/uahrk_navigation/move_utils.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(move_to_pose_main src/move_to_pose_main.cpp)
ament_target_dependencies(move_to_pose_main ${dependencies})
target_link_libraries(move_to_pose_main ${PROJECT_NAME})

add_executable(robot_tf_main src/robot_tf_main.cpp)
ament_target_dependencies(robot_tf_main ${dependencies})
target_link_libraries(robot_tf_main ${PROJECT_NAME})

add_executable(rviz_interface src/RvizInterfaceNode.cpp)
ament_target_dependencies(rviz_interface ${dependencies})
target_link_libraries(rviz_interface ${PROJECT_NAME})


add_executable(grid_main src/grid_main.cpp)
ament_target_dependencies(grid_main ${dependencies})
target_link_libraries(grid_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  move_to_pose_main
  robot_tf_main
  grid_main
  rviz_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  #find_package(ros_testing REQUIRED)

  #ament_lint_auto_find_test_dependencies()
  ament_add_gtest(spin_to_goal_test tests/spin_to_goal_test.cpp)
  ament_target_dependencies(spin_to_goal_test ${dependencies})
  target_link_libraries(spin_to_goal_test ${PROJECT_NAME})

  ament_add_gtest(advance_to_goal_test tests/advance_to_goal_test.cpp)
  ament_target_dependencies(advance_to_goal_test ${dependencies})
  target_link_libraries(advance_to_goal_test ${PROJECT_NAME})
endif()


ament_package()
