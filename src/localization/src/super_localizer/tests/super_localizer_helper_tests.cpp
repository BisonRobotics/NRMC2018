#include <gtest/gtest.h>
#include <super_localizer/super_localizer_helper.h>


TEST(SuperLocalizerHelperTests, diffWorks)
{
   Localizer::stateVector_s v1 ={.x_pos = 1,
                                                             .y_pos = 1,
                                                             .theta = 1,
                                                             .x_vel = 1,
                                                             .y_vel = 1,
                                                             .omega = 1,
                                                             .x_accel = 1,
                                                             .y_accel = 1,
                                                             .alpha = 1 };


ASSERT_TRUE(1==2);



}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}