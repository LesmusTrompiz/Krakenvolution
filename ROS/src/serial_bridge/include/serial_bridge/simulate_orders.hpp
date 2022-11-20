#pragma once

#include "serial_bridge/pose2d.hpp"
extern "C"{
    #include <math.h>
}

void simulate_advance(Pose2d &p,int distance);
void simulate_spin(Pose2d &p,int spin);

inline float DEG2RAD(const float deg)
{
  return deg* M_PI / 180;
}

