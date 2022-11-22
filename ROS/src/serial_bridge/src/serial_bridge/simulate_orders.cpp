#include "serial_bridge/simulate_orders.hpp"
#include <sstream>      // std::stringstream

const std::unordered_map<std::string, std::function<void(Pose2d &, int)>> simulate = {
  {"spin" , simulate_spin},
  {"advance" , simulate_advance}
};


void simulate_advance(Pose2d &p,int distance)
{
  if (abs(distance) <= MIN_DIST){
    std::stringstream error_msg;
    error_msg << "Dist sent "<< distance << "is below limits " << MIN_DIST << std::endl;
    throw std::invalid_argument(error_msg.str());
  }

  float noise = (rand() % DIST_MAX_NOISE) - (DIST_MAX_NOISE/2);
  distance    = distance + noise;
  
  float d = (float)distance/1000;
  p.x += d * cos(DEG2RAD(p.a));
  p.y += d * sin(DEG2RAD(p.a));
}


void simulate_spin(Pose2d &p,int spin)
{
  if (abs(spin) <= MIN_SPIN){
    std::stringstream error_msg;
    error_msg << "Spin send "<< spin << "is below limits " << MIN_SPIN << std::endl;
    throw std::invalid_argument(error_msg.str());
  }

  float noise = (rand() % SPIN_MAX_NOISE) - (SPIN_MAX_NOISE/2);
  p.a += spin + noise;
  if (p.a >  180) p.a -= 360;
  if (p.a < -180) p.a += 360;
}



