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


bool advance_thread(Pose2d &p, int16_t &distance);
bool spin_thread(Pose2d &p, int16_t &distance);


inline float DEG2RAD(const float deg)
{
  return deg* M_PI / 180;
}

extern const std::unordered_map<std::string, std::function<bool(Pose2d &, int16_t &)>> simulate;




