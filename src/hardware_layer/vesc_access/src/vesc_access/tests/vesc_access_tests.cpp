// Bring in gtest
#include <gtest/gtest.h>
#include <vesc_access/vesc_access.h>
#include <vesc_control/mock_vesc.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;
// Declare a test
TEST(VescAccessTests, canSetLinearVelocity)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float linear = 15.0f;
  EXPECT_CALL(vesc, setRpm(linear * transmission_ratio / (output_ratio)));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, 30.0f, 1.0f, &vesc, 1);
  wrap->setLinearVelocity(linear);
}

TEST(VescAccessTests, canSetTorque)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque = 14.0f;
  float torque_ratio = 2.0f;
  EXPECT_CALL(vesc, setCurrent(torque / (torque_ratio * transmission_ratio)));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, 30.0f, torque_ratio, &vesc, 1);
  wrap->setTorque(torque);
}

TEST(VescAccessTests, canSetVelocityLimit)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, velocity_limit, 30.0f, 1.0f, &vesc, 1);
  EXPECT_EQ(wrap->getLinearVelocityLimit(), velocity_limit);
}

TEST(VescAccessTests, canSetTorqueLimit)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc, 1);
  EXPECT_EQ(wrap->getTorqueLimit(), torque_limit);
}

TEST(VescAccessTests, saturatesOnTorqueLimit)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(torque_limit / transmission_ratio));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc, 1);
  wrap->setTorque(20.0f);
}

TEST(VescAccessTests, saturatesOnVelocityLimit)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm((velocity_limit * transmission_ratio / (output_ratio))));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, velocity_limit, 30.0f, 1.0f, &vesc, 1);
  wrap->setLinearVelocity(20.0f);
}

TEST(VescAccessTests, saturatesOnNegativeVelocityLimit)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm(-1.0f * (velocity_limit * transmission_ratio / (output_ratio))));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, velocity_limit, 30.0f, 1.0f, &vesc, 1);
  wrap->setLinearVelocity(-1.0f * 20.0f);
}

TEST(VescAccessTests, saturatesOnNegativeTorqueLimit)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(-1.0f * torque_limit / transmission_ratio));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc, 1);
  wrap->setTorque(-20.0f);
}

TEST(VescAccessTests, allowZeroTorque)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(0.0f));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc, 1);
  wrap->setTorque(0.0f);
}

TEST(VescAccessTests, allowZeroVelocity)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm(0.0f));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc, 1);
  wrap->setLinearVelocity(0.0f);
}

TEST(VescAccessTests, zeroOuputRatioDefaultToOne)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 0.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc, 1);
  EXPECT_EQ(wrap->getOutputRatio(), 1.0f);
}

TEST(VescAccessTests, zeroTransmissionRatioDefaultToOne)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 0.0f;
  float transmission_ratio = 0.0f;
  float torque_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc, 1);
  EXPECT_EQ(wrap->getTransmissionRatio(), 1.0f);
}

TEST(VescAccessTests, zeroTorqueConstantRatioDefaultToOne)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 0.0f;
  float transmission_ratio = 0.0f;
  float torque_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 0.0f, &vesc, 1);
  EXPECT_EQ(wrap->getTransmissionRatio(), 1.0f);
}

TEST(VescAccessTests, canSetPolePairs)
{
  NiceMock<MockVesc> vesc;
  float output_ratio = 1.0f;
  float transmission_ratio = 1.0f;
  float torque_limit = 12.0f;
  ON_CALL(vesc, getRpm()).WillByDefault(Return(400));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 0.0f, &vesc, 4);
  EXPECT_EQ(wrap->getLinearVelocity(), 100.0f);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
