#include <gtest/gtest.h>
#include "uahrk_navigation/move_utils.hpp"



TEST(SpinTest, ZeroTo90){
  /**
   * @test The robot has a 0º orientation 
   * and the goal is at 90º, the robot must
   * spin 90º to accomplish the goal.
   */
  float robot_alfa = 0.0;
  float goal_alfa  = 90.0;
  int spin; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, 90);
}


TEST(SpinTest, ZeroTo180){
  /**
   * @test The robot has a 0º orientation 
   * and the goal is at 180º, the robot must
   * spin 180º to accomplish the goal.
   */
  float robot_alfa =    0.0;
  float goal_alfa  =  180.0;
  int   spin; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, 180);
}

TEST(SpinTest, ZeroToMinus180){
  /**
   * @test The robot has a 0º orientation 
   * and the goal is at -180º, the robot must
   * spin 180º to accomplish the goal.
   */
  float robot_alfa =    0.0;
  float goal_alfa  = -180.0;
  int   spin; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, -180);
}

TEST(SpinTest, Minus170To170){
  /**
   * @test The robot has a -170º orientation 
   * and the goal is at 170º, the robot must
   * spin 20º to accomplish the goal.
   */
  float robot_alfa = -170.0;
  float goal_alfa  =  170.0;
  int   spin; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, -20);
}

TEST(SpinTest, Plus170ToMinus170){
  /**
   * @test The robot has a -170º orientation 
   * and the goal is at 170º, the robot must
   * spin 20º to accomplish the goal.
   */
  float robot_alfa =  170.0;
  float goal_alfa  = -170.0;
  int   spin; 
  spin = spin_to_goal(robot_alfa, goal_alfa);
  ASSERT_EQ(spin, 20);
}

TEST(SpinTest, Minus100To30){
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

TEST(SpinTest, Minus180To180){
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


TEST(NotItRangeInput, RobotAlfa185){
    // The only object is the whole scan
    float robot_alfa =  185.0;
    float goal_alfa  =   40.0;
    EXPECT_DEBUG_DEATH(spin_to_goal(robot_alfa, goal_alfa),"");
}

TEST(NotItRangeInput, RobotAlfaMinus190){
    // The only object is the whole scan
    float robot_alfa = -190.0;
    float goal_alfa  =   40.0;
    EXPECT_DEBUG_DEATH(spin_to_goal(robot_alfa, goal_alfa),"");
}

TEST(NotItRangeInput, GoalAlfa190){
    // The only object is the whole scan
    float robot_alfa =   30.0;
    float goal_alfa  =  190.0;
    EXPECT_DEBUG_DEATH(spin_to_goal(robot_alfa, goal_alfa),"");
}

TEST(NotItRangeInput, GoalAlfaMinus190){
    // The only object is the whole scan
    float robot_alfa =   30.0;
    float goal_alfa  = -190.0;
    EXPECT_DEBUG_DEATH(spin_to_goal(robot_alfa, goal_alfa),"");
}




