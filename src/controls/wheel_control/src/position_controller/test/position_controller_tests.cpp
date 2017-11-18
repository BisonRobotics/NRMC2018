#include <gtest/gtest.h>
#include <position_controller/position_controller.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;
// Declare a test

TEST(PositionControllerTests, canBeInstantiatedWithAVelocity)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);

  EXPECT_EQ(pos.getVelocity(), velocity);
}

TEST(PositionControllerTests, canBeGivenDistanceToTravel)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);
  pos.update(0.0f, 0.0f);
  pos.setDistance(3.0f);
  EXPECT_EQ(pos.getDistance(), 3.0f);
}

TEST(PositionControllerTests, canBeUpdatedWithPosition)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);
  pos.update(2.0f, 2.0f);
}

TEST(PositionControllerTests, shouldTakeAbsValOfVelocity)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(-1.0f * velocity, &fl, &fr, &br, &bl);
  EXPECT_EQ(pos.getVelocity(), velocity);
}

TEST(PositionControllerTests, shouldMoveAfterGettingGoal)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);
  pos.update(0.0f, 0.0f);
  pos.setDistance(1.0f);
  pos.update(0.0f, 0.0f);
  EXPECT_TRUE(pos.isMoving());
}

TEST(PositionControllerTests, shouldGoUntilPositionReached)
{
  float velocity = 10.0f;
  float tolerance = .001f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);
  pos.update(0.0f, 0.0f);
  pos.setDistance(1.0f);
  pos.update(0.0f, 0.0f);
  EXPECT_TRUE(pos.isMoving());
  pos.update(1.0f, 0.0f);
  EXPECT_FALSE(pos.isMoving());
}

TEST(PositionControllerTests, shouldStopIfGreater)
{
  float velocity = 10.0f;
  float tolerance = .001f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);
  pos.update(0.0f, 0.0f);
  pos.setDistance(1.0f);
  pos.update(0.0f, 0.0f);
  pos.update(3.0f, 0.0f);
  EXPECT_FALSE(pos.isMoving());
}

TEST(PositionControllerTests, takesAbsValOfDistance)
{
  float velocity = 10.0f;
  float tolerance = .001f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);
  pos.update(0.0f, 0.0f);
  pos.setDistance(-1.0f);
  EXPECT_EQ(pos.getDistance(), 1.0f);
}

TEST(PositionControllerTests, exposeMethodToTellDistanceLeft)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);
  pos.update(0.0f, 0.0f);
  pos.setDistance(3.0f);
  pos.update(1.0f, 0.0f);
  EXPECT_EQ(pos.getDistanceRemaining(), 2.0f);
}

TEST(PositionControllerTests, cancancelGoals)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  PositionController pos = PositionController(velocity, &fl, &fr, &br, &bl);
  pos.update(0.0f, 0.0f);
  pos.setDistance(3.0f);
  pos.update(1.0f, 0.0f);
  EXPECT_TRUE(pos.isMoving());
  pos.closeGoal();
  EXPECT_FALSE(pos.isMoving());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
