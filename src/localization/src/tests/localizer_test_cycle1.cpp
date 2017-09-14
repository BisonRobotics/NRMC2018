#include <gtest/gtest.h>
#include <localizer.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
// Declare a test
TEST(MotionTracking, GoesForward)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  Localizer loki(&flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.5));

  loki.updateStateVector(.01);  // 10 ms, 100Hz update
  EXPECT_TRUE(loki.stateVector.xPos >= .0049);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
