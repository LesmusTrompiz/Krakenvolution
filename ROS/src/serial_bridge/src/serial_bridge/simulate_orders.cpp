#include "serial_bridge/simulate_orders.hpp"
#include <sstream>      // std::stringstream
#include <iostream>

const std::unordered_map<std::string, std::function<bool(Pose2d &, int16_t &)>> simulate = {
  {"spin" , spin_thread},
  {"advance" , advance_thread}
};

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif


bool advance_thread(Pose2d &p, int16_t &distance){

  constexpr float speed     {2000};
  constexpr float TICK_TIME {0.1};
  constexpr float advance_tick {speed * TICK_TIME};
  bool succed {false};

  if(distance > 0){
    distance -= advance_tick;
    if(distance <= 0){
      float d = (float)(distance+advance_tick)/1000;
      p.x += d * cos(DEG2RAD(p.a));
      p.y += d * sin(DEG2RAD(p.a));
      distance = 0;
      return true;
    } 
    else{
      float d = (float)advance_tick/1000;
      p.x += d * cos(DEG2RAD(p.a));
      p.y += d * sin(DEG2RAD(p.a));
      return false;
    }
  }  

  else if(distance < 0){
    distance += advance_tick;
    if(distance > 0){
      p.a -= (advance_tick + distance);
      return true;
    } 
    else{
      p.a -= advance_tick;
      return false;
    }
  }
  return true;
}

bool spin_thread(Pose2d &p, int16_t &spin){

  constexpr float speed     {360};
  constexpr float TICK_TIME {0.1};
  constexpr float spin_tick {speed * TICK_TIME};
  bool succed {false};
  if(spin > 0){
    spin -= spin_tick;
    if(spin <= 0){
      p.a  += (spin_tick + spin);
      spin = 0;
      succed = true;
    } 
    else{
      p.a += spin_tick;
    }
  }  
  else if(spin < 0){
    spin += spin_tick;
    if(spin >= 0){
      p.a -= (spin_tick - spin);
      succed = true;
    } 
    else{
      p.a -= spin_tick;
    }
  }

  if (p.a >  180) p.a -= 360;
  if (p.a < -180) p.a += 360;
  return succed;
}


