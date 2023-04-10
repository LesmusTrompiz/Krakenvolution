#pragma once

// MSGs Libraries
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Own Libraries
#include "serial_bridge/pose2d.hpp"

// Utilidades
// Función que convierte de grados a radianes.
inline float DEG2RAD(const float deg)
{
  return deg* M_PI / 180;
}

geometry_msgs::msg::Pose Pose2dtoPose(const Pose2d &p2d);
