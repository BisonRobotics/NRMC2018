//#include "foo/foo.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <vesc_access/vesc_access.h>
#include <vesc_control/mock_vesc.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;

// Declare a test
TEST(VescAccessTest, canSetLinearVelocity)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float linear = 15.0f;
  EXPECT_CALL(vesc, setRpm(linear / (output_ratio * transmission_ratio)));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, 30.0f, 1.0f, &vesc);
  wrap->setLinearVelocity(linear);
}

TEST(VescAccessTest, canSetTorque)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque = 14.0f;
  float torque_ratio = 2.0f;
  EXPECT_CALL(vesc, setCurrent(torque / torque_ratio));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, 30.0f, torque_ratio, &vesc);
  wrap->setTorque(torque);
}

TEST(VescAccessTest, canSetVelocityLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, velocity_limit, 30.0f, 1.0f, &vesc);
  EXPECT_EQ(wrap->getLinearVelocityLimit(), velocity_limit);
}

TEST(VescAccessTest, canSetTorqueLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc);
  EXPECT_EQ(wrap->getTorqueLimit(), torque_limit);
}

TEST(VescAccessTest, saturatesOnTorqueLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(torque_limit));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc);
  wrap->setTorque(20.0f);
}

TEST(VescAccessTest, saturatesOnVelocityLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm((velocity_limit / (output_ratio * transmission_ratio))));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, velocity_limit, 30.0f, 1.0f, &vesc);
  wrap->setLinearVelocity(20.0f);
}

TEST(VescAccessTest, saturatesOnNegativeVelocityLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm(-1.0f * (velocity_limit / (output_ratio * transmission_ratio))));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, velocity_limit, 30.0f, 1.0f, &vesc);
  wrap->setLinearVelocity(-1.0f * 20.0f);
}

TEST(VescAccessTest, saturatesOnNegativeTorqueLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(-1.0f * torque_limit));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc);
  wrap->setTorque(-20.0f);
}

TEST(VescAccessTest, allowZeroTorque)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(0.0f));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc);
  wrap->setTorque(0.0f);
}

TEST(VescAccessTest, allowZeroVelocity)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm(0.0f));
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc);
  wrap->setLinearVelocity(0.0f);
}

TEST(VescAccessTest, zeroOuputRatioDefaultToOne)
{
  mockVesc vesc;
  float output_ratio = 0.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc);
  EXPECT_EQ(wrap->getOutputRatio(), 1.0f);
}

TEST(VescAccessTest, zeroTransmissionRatioDefaultToOne)
{
  mockVesc vesc;
  float output_ratio = 0.0f;
  float transmission_ratio = 0.0f;
  float torque_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 1.0f, &vesc);
  EXPECT_EQ(wrap->getTransmissionRatio(), 1.0f);
}

TEST(VescAccessTest, zeroTorqueConstantRatioDefaultToOne)
{
  mockVesc vesc;
  float output_ratio = 0.0f;
  float transmission_ratio = 0.0f;
  float torque_limit = 12.0f;
  VescAccess *wrap = new VescAccess(transmission_ratio, output_ratio, 30.0f, torque_limit, 0.0f, &vesc);
  EXPECT_EQ(wrap->getTransmissionRatio(), 1.0f);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}