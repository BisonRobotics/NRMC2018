//#include "foo/foo.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <position_controller/position_controller.h>

#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
// Declare a test
TEST(PositionControlTest, canBeInstantiatedWithAVelocity )
{
  float velocity = 10.0f;
  float tolerance = 10.0f;
  PositionController *pos = new PositionController (velocity, tolerance );

  EXPECT_EQ (pos->getVelocity (), velocity);
}

TEST(PositionControlTest, canBeGivenTolerance){
  float velocity = 10.0f;
  float tolerance = 0.01f;
  PositionController *pos = new PositionController (velocity, tolerance);
  EXPECT_EQ(pos->getTolerance (), tolerance);
}


TEST(PositionControlTest, canBeGivenDistanceToTravel){
  float velocity = 10.0f;
  float tolerance = 0.01f;
  PositionController *pos = new PositionController (velocity, tolerance);
  pos->setDistance (3.0f);
  EXPECT_EQ (pos->getDistance (), 3.0f);
}

TEST(PositionControlTest, canBeUpdatedWithPosition){
  float velocity = 10.0f;
  float tolerance = .001f;
  PositionController *pos = new PositionController (velocity, tolerance);
  pos->update (2.0f);
}

TEST(PositionControlTest, shouldTakeAbsValOfVelocity){
  float velocity = 10.0f;
  float tolerance = .001f;
  PositionController *pos = new PositionController (-1.0f*velocity, tolerance);
  EXPECT_EQ (pos->getVelocity(), velocity);
}

TEST(PositionControlTest, shouldBeAbleToReturnMoving){
  float velocity = 10.0f;
  float tolerance = .001f;
  PositionController *pos = new PositionController (velocity, tolerance);
  EXPECT_EQ (pos->isMoving, false);
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
