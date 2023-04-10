#include <gtest/gtest.h>
#include "uahrk_navigation/next_move.hpp"


TEST(AngleBetweenRange, AllAngleAreIn0360Range){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (2,0), it has to
   * advance 2 coords unit.
   */
  EXPECT_TRUE(angle_between_angle_range({0,360},   0));
  EXPECT_TRUE(angle_between_angle_range({0,360},  50));
  EXPECT_TRUE(angle_between_angle_range({0,360}, 360));
}

TEST(AngleBetweenRange, NoAngleInRange){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (2,0), it has to
   * advance 2 coords unit.
   */
  EXPECT_FALSE(angle_between_angle_range({0,90},  100));
  EXPECT_FALSE(angle_between_angle_range({0,90},  299));
  EXPECT_FALSE(angle_between_angle_range({0,10}, 360));
}

TEST(AngleBetweenRange, AngleInRange){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (2,0), it has to
   * advance 2 coords unit.
   */
  EXPECT_TRUE(angle_between_angle_range({ 90, 125}, 100));
  EXPECT_TRUE(angle_between_angle_range({ 10,  30},  20));
  EXPECT_TRUE(angle_between_angle_range({ 11,  13},  12));
  EXPECT_TRUE(angle_between_angle_range({350, 355}, 352));
}

TEST(AngleBetweenRange, AngleInSuppLimits){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (2,0), it has to
   * advance 2 coords unit.
   */
  EXPECT_TRUE(angle_between_angle_range({ 90, 125}, 125));
  EXPECT_TRUE(angle_between_angle_range({ 10,  30},  30));
  EXPECT_TRUE(angle_between_angle_range({ 11,  13},  13));
  EXPECT_TRUE(angle_between_angle_range({350, 355}, 355));
}

TEST(AngleBetweenRange, AngleInInfLimits){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (2,0), it has to
   * advance 2 coords unit.
   */
  EXPECT_TRUE(angle_between_angle_range({ 90, 125},  90));
  EXPECT_TRUE(angle_between_angle_range({ 10,  30},  10));
  EXPECT_TRUE(angle_between_angle_range({ 11,  13},  11));
  EXPECT_TRUE(angle_between_angle_range({350, 355}, 350));
}

TEST(AngleBetweenRange, AngleRangeShouldBeSorted){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (2,0), it has to
   * advance 2 coords unit.
   */
  EXPECT_DEBUG_DEATH(angle_between_angle_range({ 90, -125},  90), "");
  EXPECT_DEBUG_DEATH(angle_between_angle_range({190,  125},  90), "");
  EXPECT_DEBUG_DEATH(angle_between_angle_range({ 90, -125},  90), "");
  EXPECT_DEBUG_DEATH(angle_between_angle_range({ 90, -125},  90), "");
}

TEST(AngleBetweenRange, AngleInRangeLoop){
  /**
   * @test The robot is in a (0,0) coords
   * and the goal is in (2,0), it has to
   * advance 2 coords unit.
   */

  int min_range,max_range = 0;

  min_range =   0;
  max_range = 180;
  for(int angle=min_range; angle==max_range; angle++){
    EXPECT_TRUE(angle_between_angle_range({min_range, max_range}, angle));
  }

  min_range = -50;
  max_range =  30;
  for(int angle=min_range; angle==max_range; angle++){
    EXPECT_TRUE(angle_between_angle_range({min_range, max_range}, angle));
  }

  min_range = -20;
  max_range =  -3;
  for(int angle=min_range; angle==max_range; angle++){
    EXPECT_TRUE(angle_between_angle_range({min_range, max_range}, angle));
  }
}
