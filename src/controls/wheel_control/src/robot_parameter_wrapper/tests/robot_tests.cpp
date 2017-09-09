//#include "foo/foo.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <robot_parameter_wrapper/robot_parameter_wrapper.h>
#include <vesc_control/mock_vesc.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;

// Declare a test
TEST(ParamWrapperTest, canSetLinearVelocity)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float linear = 15.0f;
  EXPECT_CALL(vesc, setRpm(linear / (output_ratio * transmission_ratio)));
  RobotParameterWrapper *wrap = new RobotParameterWrapper(transmission_ratio, output_ratio, 30.0f, 30.0f, &vesc);
  wrap->setLinearVelocity(linear);
}

TEST(ParamWrapperTest, canSetTorque)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque = 14.0f;
  EXPECT_CALL(vesc, setCurrent(torque));
  RobotParameterWrapper *wrap = new RobotParameterWrapper(transmission_ratio, output_ratio, 30.0f, 30.0f, &vesc);
  wrap->setTorque(torque);
}

TEST(ParamWrapperTest, canSetVelocityLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  RobotParameterWrapper *wrap =
      new RobotParameterWrapper(transmission_ratio, output_ratio, velocity_limit, 30.0f, &vesc);
  EXPECT_EQ(wrap->getLinearVelocityLimit(), velocity_limit);
}

TEST(ParamWrapperTest, canSetTorqueLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  RobotParameterWrapper *wrap = new RobotParameterWrapper(transmission_ratio, output_ratio, 30.0f, torque_limit, &vesc);
  EXPECT_EQ(wrap->getTorqueLimit(), torque_limit);
}

TEST(ParamWrapperTest, saturatesOnTorqueLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(torque_limit));
  RobotParameterWrapper *wrap = new RobotParameterWrapper(transmission_ratio, output_ratio, 30.0f, torque_limit, &vesc);
  wrap->setTorque(20.0f);
}

TEST(ParamWrapperTest, saturatesOnVelocityLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm((velocity_limit / (output_ratio * transmission_ratio))));
  RobotParameterWrapper *wrap =
      new RobotParameterWrapper(transmission_ratio, output_ratio, velocity_limit, 30.0f, &vesc);
  wrap->setLinearVelocity(20.0f);
}
TEST(ParamWrapperTest, saturatesOnNegativeVelocityLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float velocity_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm(-1.0f*(velocity_limit / (output_ratio * transmission_ratio))));
  RobotParameterWrapper *wrap =
      new RobotParameterWrapper(transmission_ratio, output_ratio, velocity_limit, 30.0f, &vesc);
  wrap->setLinearVelocity(-1.0f*20.0f);
}
TEST(ParamWrapperTest, saturatesOnNegativeTorqueLimit)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(-1.0f*torque_limit));
  RobotParameterWrapper *wrap = new RobotParameterWrapper(transmission_ratio, output_ratio, 30.0f, torque_limit, &vesc);
  wrap->setTorque(-20.0f);
}
TEST(ParamWrapperTest, allowZeroTorque)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setCurrent(0.0f));
  RobotParameterWrapper *wrap = new RobotParameterWrapper(transmission_ratio, output_ratio, 30.0f, torque_limit, &vesc);
  wrap->setTorque(0.0f);
}
TEST(ParamWrapperTest, allowZeroVelocity)
{
  mockVesc vesc;
  float output_ratio = 10.0f;
  float transmission_ratio = 10.0f;
  float torque_limit = 12.0f;
  EXPECT_CALL(vesc, setRpm(0.0f));
  RobotParameterWrapper *wrap = new RobotParameterWrapper(transmission_ratio, output_ratio, 30.0f, torque_limit, &vesc);
  wrap->setLinearVelocity(0.0f);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
