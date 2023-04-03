#pragma once
#include "uahrk_navigation/move_utils.hpp"

std::tuple<std::string, int> calculate_move2(
                    const Pose2d &robot_pose, 
                    const Pose2d &goal_pose);
std::tuple<std::string, int> advance(const Pose2d &robot_pose, const Pose2d &goal_pose);
std::tuple<std::string, int> correct_angle(const Pose2d &robot_pose, int spin);
std::tuple<std::string, int> escape_from_wall(const Pose2d &robot_pose);
int calculate_max_spin_with_out_collision(const Pose2d &robot_pose);
int calculate_angle_with_wall(const Pose2d &robot_pose);
float calculate_advance_error(const Pose2d &robot_pose, const Pose2d &goal_pose);
int convert_into_fast_spin(int angle);
bool angle_between_angle_range(std::tuple<int, int> angle_range, int angle);
std::tuple<int, int> spin_with_out_wall_collision(const Pose2d &robot_pose);
bool robot_in_corner(const Pose2d &robot_pose);
bool near_wall(const Pose2d &robot_pose);
bool robot_advance_goes_outside_corner(const Pose2d &robot_pose);
void throw_error(const Pose2d &robot_pose, const Pose2d &goal_pose);

