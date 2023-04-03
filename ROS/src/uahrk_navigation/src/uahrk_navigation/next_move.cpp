#include "uahrk_navigation/next_move.hpp"


std::tuple<std::string, int> calculate_move2(
                    const Pose2d &robot_pose, 
                    const Pose2d &goal_pose){
  if(in_wp(robot_pose,goal_pose)){
    if(robot_in_angle(robot_pose, goal_pose)){
      throw_error(robot_pose,goal_pose);
    }
    else{
      auto spin = spin_to_goal(robot_pose.a, goal_pose.a);
      return correct_angle(robot_pose, spin);
    }
  }
  else{
    auto advance_error = calculate_advance_error(robot_pose, goal_pose);
    if(advance_error > DIST_PRECISION){
      auto spin = spin_to_wp(robot_pose, goal_pose);
      return correct_angle(robot_pose, spin);
    }
    else{
      return advance(robot_pose, goal_pose);
    }
  }
}

std::tuple<std::string, int> advance(const Pose2d &robot_pose, const Pose2d &goal_pose){
  auto dist = advance_to_goal(robot_pose, goal_pose);
  if(dist > 0 && dist < 0.1){
    dist = -0.3;
  }
  else if(dist <= 0 && dist > -0.1){
    dist = 0.3;
  }
  return {"advance",dist};
}

std::tuple<std::string, int> correct_angle(const Pose2d &robot_pose, int spin){  
  
  spin = convert_into_fast_spin(spin);
  if(near_wall(robot_pose)){    
    auto safe_angle_range = spin_with_out_wall_collision(robot_pose);
    if (!angle_between_angle_range(safe_angle_range, spin )){
      return escape_from_wall(robot_pose);
    }
  }
  return {"spin",spin};
}

std::tuple<std::string, int> escape_from_wall(const Pose2d &robot_pose){
  if(robot_in_corner(robot_pose)){
    if(robot_advance_goes_outside_corner(robot_pose)){
      return {"advance", -100};
    }
    else{
      return {"advance",  100};
    }
  }
  else{
    auto angle_with_wall = calculate_angle_with_wall(robot_pose);
    if(angle_between_angle_range({-20,20},angle_with_wall)){
      auto spin_with_out_collision = calculate_max_spin_with_out_collision(robot_pose);
      if(angle_between_angle_range({-5,5}, spin_with_out_collision)){
        if(angle_with_wall > 0){
          return {"advance", 100};
        }
        else{
          return {"advance", -100};
        }
      }
      else{
        return {"spin", spin_with_out_collision};
      }
    }
    if(angle_with_wall > 0){
      return {"advance", 300};
    }
    else{
      return {"advance", -300};
    }
  }
}

int calculate_max_spin_with_out_collision(const Pose2d &robot_pose){
  return 90;
}

int calculate_angle_with_wall(const Pose2d &robot_pose){
  return 90;
}

float calculate_advance_error(const Pose2d &robot_pose, const Pose2d &goal_pose){
  return 0.0;
}

int convert_into_fast_spin(int angle){
  if(angle >= -5 && angle < 0){
    angle += -5;
  }
  else if(angle >= 0 && angle <= 5){
    angle +=  5;
  }
  return angle;
}

bool angle_between_angle_range(std::tuple<int, int> angle_range, int angle){
  return true;
}

std::tuple<int, int> spin_with_out_wall_collision(const Pose2d &robot_pose){
  return {0,0};
}

bool robot_in_corner(const Pose2d &robot_pose){
  return false;
}

bool near_wall(const Pose2d &robot_pose){
  return false;
}

bool robot_advance_goes_outside_corner(const Pose2d &robot_pose){
  return true;
}

void throw_error(const Pose2d &robot_pose, const Pose2d &goal_pose){
  std::stringstream error_msg;
  auto dist = advance_to_goal(robot_pose, goal_pose);
  auto spin = spin_to_goal(robot_pose.a, goal_pose.a);
  error_msg << "ERROR CALLING CALCULATE MOVE, GOAL ALREDY SATISFIED :";
  error_msg << "DIST "  << dist      << " < DIST CONDITION "  << DIST_PRECISION  * 1000 << " ";
  error_msg << "ANGLE " << abs(spin) << " < ANGLE CONDITION " << ANGLE_PRECISION;
  throw std::invalid_argument(error_msg.str());
}



