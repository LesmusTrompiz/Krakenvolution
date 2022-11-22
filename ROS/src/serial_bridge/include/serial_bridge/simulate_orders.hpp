#pragma once
#include <unordered_map>
#include <functional>
#include "serial_bridge/pose2d.hpp"
extern "C"{
    #include <math.h>
}

constexpr float MIN_DIST = 100;
constexpr float MIN_SPIN = 5;
constexpr int   DIST_MAX_NOISE = 50;
constexpr int   SPIN_MAX_NOISE =  5;


void simulate_advance(Pose2d &p,int distance);
void simulate_spin(Pose2d &p,int spin);
inline float DEG2RAD(const float deg)
{
  return deg* M_PI / 180;
}

extern const std::unordered_map<std::string, std::function<void(Pose2d &, int)>> simulate;




