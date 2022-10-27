#include <gtest/gtest.h>
#include "uahrk_navigation/MoveToPoseNode.hpp"


TEST(CalculateSpinTest, ZeroTo90){
  /**
   * @test The robot has a 0º orientation 
   * and the goal is at 90º, the robot must
   * spin 90º to accomplish the goal.
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
   * spin 180º to accomplish the goal.
   */
  float robot_alfa =    0.0;
  float goal_alfa  =  180.0;
  int   spin       =    0; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, 180);
}

TEST(CalculateSpinTest, ZeroToMinus180){
  /**
   * @test The robot has a 0º orientation 
   * and the goal is at -180º, the robot must
   * spin 180º to accomplish the goal.
   */
  float robot_alfa =    0.0;
  float goal_alfa  = -180.0;
  int   spin       =    0; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, -180);
}

TEST(CalculateSpinTest, Minus170To170){
  /**
   * @test The robot has a -170º orientation 
   * and the goal is at 170º, the robot must
   * spin 20º to accomplish the goal.
   */
  float robot_alfa = -170.0;
  float goal_alfa  = -170.0;
  int   spin       =    0; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, -20);
}

TEST(CalculateSpinTest, Minus100To30){
  /**
   * @test The robot has a -100º orientation 
   * and the goal is at 30º, the robot must
   * spin 130º to accomplish the goal.
   */
  float robot_alfa = -100.0;
  float goal_alfa  =   30.0;
  int   spin       =  130; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, 130);
}

TEST(CalculateSpinTest, Minus180To180){
  /**
   * @test The robot has a -180º orientation 
   * and the goal is at 180º, the robot must
   * spin 0º to accomplish the goal.
   */
  float robot_alfa = -180.0;
  float goal_alfa  =  180.0;
  int   spin       =      0; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, 0);
}