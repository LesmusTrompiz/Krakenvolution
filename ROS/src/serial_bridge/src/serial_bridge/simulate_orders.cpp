#include "serial_bridge/simulate_orders.hpp"


void simulate_advance(Pose2d &p,int distance)
{
  float d = (float)distance/1000;
  p.x += d * cos(DEG2RAD(p.a));
  p.y += d * sin(DEG2RAD(p.a));
}

void simulate_spin(Pose2d &p,int spin)
{
  p.a += spin;
  if (p.a >  180) p.a -= 360;
  if (p.a < -180) p.a += 360;
}



