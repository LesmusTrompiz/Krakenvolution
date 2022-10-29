#include <gtest/gtest.h>
#include "uahrk_navigation/MoveToPoseNode.hpp"
#include "uahrk_navigation/point2d.hpp"


TEST(AdvanceTest, From00To20){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (2,0), it has to
   * advance 2 coords unit.
   */
  Point2d robot {0,0};
  Point2d goal  {2,0};
  int advance;
  advance = advance_to_goal(robot, goal);
  ASSERT_EQ(advance, 2);
}


TEST(AdvanceTest, From00To00){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (0,0), it has to
   * advance 2 coords unit.
   */
  Point2d robot {0,0};
  Point2d goal  {0,0};
  int advance;
  advance = advance_to_goal(robot, goal);
  ASSERT_EQ(advance, 0);
}

TEST(AdvanceTest, From00To05){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (0,5), it has to
   * advance 5 coords unit.
   */
  Point2d robot {0,0};
  Point2d goal  {0,5};
  int advance;
  advance = advance_to_goal(robot, goal);
  ASSERT_EQ(advance, 5);
}




