// Bring in my package's API, which is what I'm testing
//#include "foo/foo.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <robot_parameter_wrapper/robotparameterwrapper.h>
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
  EXPECT_CALL (vesc, setRpm(linear/(output_ratio*transmission_ratio) ) );
  RobotParameterWrapper *wrap= new RobotParameterWrapper (transmission_ratio, output_ratio,30.0f, 30.0f, &vesc );
  wrap->setLinearVelocity(linear);

}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
