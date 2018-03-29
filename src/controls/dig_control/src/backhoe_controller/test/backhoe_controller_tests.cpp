#include <vesc_access/mock_vesc_access.h>
#include "gtest/gtest.h"
#include "backhoe_controller/backhoe_controller.h"
#include "gmock/gmock.h"
#include "wheel_params/wheel_params.h"

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::_;
using ::testing::Lt;

TEST(backhoe_controller_test, can_be_init)
{

}

TEST(backhoe_controller_test, safety_features_will_not_allow_collision)
{

}

TEST(backhoe_controller_test, wont_set_velocity_past_limit)
{

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
