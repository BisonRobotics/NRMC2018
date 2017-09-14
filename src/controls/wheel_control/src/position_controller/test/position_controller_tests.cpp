
// Bring in gtest
#include <gtest/gtest.h>
#include <position_controller/position_controller.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
// Declare a test

TEST(PositionControlTest, canBeInstantiatedWithAVelocity)
{
  float velocity = 10.0f;
  float tolerance = 10.0f;
  MockVescAccess br;
  MockVescAccess bl;
  MockVescAccess fr;
  MockVescAccess fl;
  PositionController *pos = new PositionController(velocity, tolerance, &fl, &fr, &br, &bl);

  EXPECT_EQ(pos->getVelocity(), velocity);
}

TEST(PositionControlTest, canBeGivenTolerance)
{
  float velocity = 10.0f;
  float tolerance = 0.01f;
  MockVescAccess br;
  MockVescAccess bl;
  MockVescAccess fr;
  MockVescAccess fl;
  PositionController *pos = new PositionController(velocity, tolerance, &fl, &fr, &br, &bl);

  EXPECT_EQ(pos->getTolerance(), tolerance);
}

TEST(PositionControlTest, canBeGivenDistanceToTravel)
{
  float velocity = 10.0f;
  float tolerance = 0.01f;
  MockVescAccess br;
  MockVescAccess bl;
  MockVescAccess fr;
  MockVescAccess fl;
  PositionController *pos = new PositionController(velocity, tolerance, &fl, &fr, &br, &bl);
  pos->setDistance(3.0f);
  EXPECT_EQ(pos->getDistance(), 3.0f);
}

TEST(PositionControlTest, canBeUpdatedWithPosition)
{
  float velocity = 10.0f;
  float tolerance = .001f;
  MockVescAccess br;
  MockVescAccess bl;
  MockVescAccess fr;
  MockVescAccess fl;
  PositionController *pos = new PositionController(velocity, tolerance, &fl, &fr, &br, &bl);
  pos->update(2.0f, 2.0f);
}

TEST(PositionControlTest, shouldTakeAbsValOfVelocity)
{
  float velocity = 10.0f;
  float tolerance = .001f;
  MockVescAccess br;
  MockVescAccess bl;
  MockVescAccess fr;
  MockVescAccess fl;
  PositionController *pos = new PositionController(-1.0f * velocity, tolerance, &fl, &fr, &br, &bl);
  EXPECT_EQ(pos->getVelocity(), velocity);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
