#pragma once
#include "uahrk_navigation/point2d.hpp"
#include "uahrk_navigation/pose2d.hpp"

extern "C"{
    #include <cassert>
    #include <math.h>
}

int spin_to_goal(float robot_alfa, float goal_alfa);
int spin_to_wp(const Pose2d origin, const Pose2d goal);

bool robot_in_goal(const Pose2d &robot, 
                 const Pose2d &goal, 
                 float dist_precision,
                 float angle_precision);
    
bool robot_in_distance(const Pose2d &robot, 
                 const Pose2d &goal, 
                 float dist_precision);
bool robot_in_angle(const Pose2d &robot, 
                 const Pose2d &goal, 
                 float angle_precision);

std::tuple<std::string, int> calculate_move(
                    const Pose2d &robot_pose, 
                    const Pose2d &goal_pose,
                    const float dist_precision,
                    const float angle_precision);




inline int advance_to_goal(const Pose2d &robot, const Pose2d &goal){
  /**
   * @brief This function calculates the advance
   * needed by the robot to accomplish the xy
   * goal.
   * 
   * @param robot Actual xy position of the robot
   * @param goal Goal xy position of the goal
   */
  
    // Return the eculidean distance in mm
    return sqrt(((robot.x - goal.x) * (robot.x - goal.x)) + ((robot.y - goal.y) * (robot.y - goal.y))) * 1000;
}




