#include <gtest/gtest.h>
#include <super_localizer/super_localizer.h>
#include <vesc_access/mock_vesc_access.h>
#include <can_sensors/mock_pos_can_sensor.h>
#include <can_sensors/mock_imu_can_sensor.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;

TEST(SuperLocalizerTests, ForwardWithPos)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  MockPosCanSensor mockPos;
  
  SuperLocalizer loki(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, (Localizer::stateVector_s) SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.3));
  EXPECT_CALL(flvesc, getLinearVelocity());
  EXPECT_CALL(frvesc, getLinearVelocity());
  EXPECT_CALL(brvesc, getLinearVelocity());
  EXPECT_CALL(blvesc, getLinearVelocity());

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  EXPECT_CALL(mockPos, getX())
        .WillOnce(Return(.01f));
  EXPECT_CALL(mockPos, getY())
        .WillOnce(Return(.00f));
  EXPECT_CALL(mockPos, getTheta())
        .WillOnce(Return(.00f));

  loki.updateStateVector(.01f);

  ASSERT_TRUE(loki.getStateVector().x_pos > 0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}