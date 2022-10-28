cmake_minimum_required(VERSION 3.5)
project(uahrk_navigation)


set(CMAKE_CXX_STANDARD 17)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rclcpp_action REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(dependencies
  rclcpp
  # rclcpp_action
)

include_directories(include)


add_library(${PROJECT_NAME} SHARED
  src/uahrk_navigation/MoveToPoseNode.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(move_to_pose_main src/move_to_pose_main.cpp)
ament_target_dependencies(move_to_pose_main ${dependencies})
target_link_libraries(move_to_pose_main ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  move_to_pose_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  #find_package(ros_testing REQUIRED)

  #ament_lint_auto_find_test_dependencies()
  ament_add_gtest(spin_to_goal_test tests/spin_to_goal_test.cpp)
  ament_target_dependencies(spin_to_goal_test ${dependencies})
target_link_libraries(spin_to_goal_test ${PROJECT_NAME})
endif()


ament_package()
