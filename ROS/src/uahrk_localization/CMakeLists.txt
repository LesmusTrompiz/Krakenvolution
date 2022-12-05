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


include_directories(
SYSTEM
${EIGEN3_INCLUDE_DIR})

include_directories(
  include
)

add_library( ${PROJECT_NAME} SHARED 
src/ekf_krakens.cpp )
 
add_executable(localization_node src/localization_node.cpp)

target_include_directories(localization_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(localization_node rclcpp geometry_msgs)

install(TARGETS localization_node
DESTINATION lib/${PROJECT_NAME})

ament_package()
