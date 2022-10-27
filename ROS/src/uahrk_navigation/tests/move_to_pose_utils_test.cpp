#include <gtest/gtest.h>
#include "uahrk_navigation/MoveToPoseNode.hpp"


TEST(CalculateSpinTest, ZeroTo90){
  /**
   * @test The robot has a 0º orientation 
   * and the goal is at 90º, the robot must
   * spin 90º to admit 
   */
  float robot_alfa = 0.0;
  float goal_alfa  = 90.0;
  int spin  = 0; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, 90);
}


TEST(CalculateSpinTest, ZeroTo180){
  /**
   * @test The robot has a 0º orientation 
   * and the goal is at 180º, the robot must
   * spin 180º to admit 
   */
  float robot_alfa = 0.0;
  float goal_alfa  = 180.0;
  int spin  = 0; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, 180);
}

TEST(CalculateSpinTest, ZeroToMinus180){
  /**
   * @test The robot has a 0º orientation 
   * and the goal is at -180º, the robot must
   * spin 180º to admit 
   */
  float robot_alfa = 0.0;
  float goal_alfa  = -180.0;
  int spin  = 0; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, -180);
}