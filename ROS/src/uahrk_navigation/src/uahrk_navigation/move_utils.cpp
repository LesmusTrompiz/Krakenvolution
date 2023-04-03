#include "uahrk_navigation/move_utils.hpp"
#include <stdexcept>
#include <iostream>
int spin_to_goal(const float robot_alfa, const float goal_alfa){
  /**
   * @brief This function calculates the spin
   * needed by the robot to accomplish the 
   * angle goal.
   * 
   * @param robot_alfa Actual rotation of the robot
   * in degrees, the expected range is from 
   * -180 to 180 (ROS STANDARD).
   * @param goal_alfa Goal rotation in degrees 
   * the expected range is from  -180 to 180
   * (ROS STANDARD).
   */
  
  // Check that arguments are in the expected range
  assert(robot_alfa <=  180);
  assert(robot_alfa >= -180);
  assert(goal_alfa  <=  180);
  assert(goal_alfa  >= -180);


  int spin = goal_alfa - robot_alfa;
  if      (spin >  180) return spin - 360;
  else if (spin < -180) return spin + 360;
  else                  return spin;
}

int spin_to_wp(const Pose2d origin, const Pose2d goal){
  /**
   * @brief This function calculates the spin
   * needed by the robot to accomplish the 
   * angle goal.
   * 
   * @param robot_alfa Actual rotation of the robot
   * in degrees, the expected range is from 
   * -180 to 180 (ROS STANDARD).
   * @param goal_alfa Goal rotation in degrees 
   * the expected range is from  -180 to 180
   * (ROS STANDARD).
   */
  
  // Check that arguments are in the expected range
  int spin = (RAD2DEG(atan2(goal.y - origin.y, goal.x - origin.x)) - origin.a);
  if      (spin >  180) return spin - 360;
  else if (spin < -180) return spin + 360;
  else                  return spin;
}

inline float square_dist(const Pose2d &p1, const Pose2d &p2){
  return ((p2.x - p1.x) * (p2.x - p1.x)) + ((p2.y - p1.y) * (p2.y - p1.y));
}

inline float angle_difference(const float &a1, const float &a2){
  float diff = a1 - a2;
  if      (diff >  180) return diff - 360;
  else if (diff < -180) return diff + 360;
  return diff;
}


bool robot_in_distance(const Pose2d &robot, 
                 const Pose2d &goal, 
                 float dist_precision){
  float square_dist_precision = dist_precision * dist_precision;

  return square_dist(robot,goal) < square_dist_precision;
}

bool in_wp(const Pose2d &robot, const Pose2d &goal){
  constexpr float SQUARE_DIST_PRECISION = DIST_PRECISION * DIST_PRECISION;
  return square_dist(robot,goal) < SQUARE_DIST_PRECISION;
}

bool in_wp_and_angle(const Pose2d &robot, const Pose2d &goal){
  return in_wp(robot,goal) && robot_in_angle(robot,goal);
}

bool robot_in_angle(const Pose2d &robot, const Pose2d &goal){
  return abs(angle_difference(robot.a,goal.a)) < ANGLE_PRECISION;
}

bool robot_in_angle(const Pose2d &robot, 
                 const Pose2d &goal, 
                 float angle_precision){
  return abs(angle_difference(robot.a,goal.a)) < angle_precision;
}
inline bool robot_in_goal(const Pose2d &robot, 
                 const Pose2d &goal, 
                 float dist_precision,
                 float angle_precision){
  float square_dist_precision = dist_precision * dist_precision;
  std::cerr << "Dist error " << square_dist(robot,goal) << " square dist precision " << square_dist_precision << " ";
  std::cerr << "Angle error " << abs(angle_difference(robot.a,goal.a)) << " angle precision " << angle_precision;

  return square_dist(robot,goal) < square_dist_precision && abs(angle_difference(robot.a,goal.a)) < angle_precision;
}

std::tuple<std::string, int> calculate_move(
                    const Pose2d &robot_pose, 
                    const Pose2d &goal_pose,
                    const float dist_precision,
                    const float angle_precision){

  auto dist = advance_to_goal(robot_pose, goal_pose);
  auto spin = spin_to_goal(robot_pose.a, goal_pose.a);
  if(dist < (dist_precision * 1000)){
    //if(abs(spin) < angle_precision){
    //  std::stringstream error_msg;
    //  error_msg << "ERROR CALLING CALCULATE MOVE, GOAL ALREDY SATISFIED :";
    //  error_msg << "DIST "  << dist      << " < DIST CONDITION "  << dist_precision  * 1000 << " ";
    //  error_msg << "ANGLE " << abs(spin) << " < ANGLE CONDITION " << angle_precision;
    //  throw std::invalid_argument(error_msg.str());
    //}
    //else{
    return {"spin",spin};
    //}
  }
  else{
    spin = spin_to_wp(robot_pose, goal_pose);
    if ((spin < 5) && (spin > -5)){
      return {"advance",dist};
    }
    else{
      return {"spin",spin};
    }
  }
}
