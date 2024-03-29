cmake_minimum_required(VERSION 3.5)
project(uahrk_localization)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(dependencies
  rclcpp
  geometry_msgs
  Eigen3
  eigen3_cmake_module
  tf2 
  tf2_ros
  tf2_geometry_msgs
)



include_directories(
SYSTEM
${EIGEN3_INCLUDE_DIR}
include)
 
add_executable( localization_main 
                src/localization_main.cpp 
                src/uahrk_localization/ekf_krakens.cpp 
                src/uahrk_localization/LocalizationNode.cpp
                src/uahrk_localization/RobotSimNode.cpp)

target_include_directories(localization_main PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(localization_main ${dependencies})

install(TARGETS localization_main
DESTINATION lib/${PROJECT_NAME})

ament_package()
