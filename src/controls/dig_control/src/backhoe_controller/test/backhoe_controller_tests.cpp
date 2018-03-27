#include <vesc_access/mock_vesc_access.h>
#include "gtest/gtest.h"
#include "backhoe_controller/backhoe_controller.h"
#include "gmock/gmock.h"
#include "wheel_params/wheel_params.h"

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::_;

TEST(backhoe_controller_test, can_be_init)
{
  NiceMock <MockVescAccess> shoulder_vesc;
  NiceMock <MockVescAccess> wrist_vesc;
  backhoecontroller::joint_params_t wrist_params = {.minimum_pos=0.0, .maximum_pos=.7,
                                                    .safety_check_pos=.2, .gain=.5, .setpoint_tolerance=.01};
  backhoecontroller::joint_params_t shoulder_params = {.minimum_pos = -1.4, .maximum_pos=1.4,
                                                    .safety_check_pos=SAFE_CENTRAL_ANGLE, .gain=.5,
                                                    .setpoint_tolerance=.01};
  ON_CALL(wrist_vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  BackhoeController backhoeController (shoulder_params, wrist_params,false, &shoulder_vesc, &wrist_vesc);
  backhoeController.init();
  EXPECT_TRUE(backhoeController.getIsInit());
}

TEST(backhoe_controller_test, safety_features_will_not_allow_collision)
{
  NiceMock<MockVescAccess> shoulder_vesc;
  NiceMock<MockVescAccess> wrist_vesc;
  backhoecontroller::joint_params_t wrist_params = {.minimum_pos=0.0, .maximum_pos=.7,
                                                    .safety_check_pos=.2, .gain=.5, .setpoint_tolerance=.01};
  backhoecontroller::joint_params_t shoulder_params = {.minimum_pos = -1.4, .maximum_pos=1.4,
                                                    .safety_check_pos=SAFE_CENTRAL_ANGLE, .gain=.5,
                                                    .setpoint_tolerance=.01};
  BackhoeController backhoeController (shoulder_params, wrist_params, false, &shoulder_vesc, &wrist_vesc);
  ON_CALL(wrist_vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  ON_CALL(shoulder_vesc, getPotPosition()).WillByDefault(Return(0.0));
  backhoeController.init ();
  ON_CALL(wrist_vesc, getLinearVelocity()).WillByDefault(Return(1.0f));
  EXPECT_CALL (shoulder_vesc, setLinearVelocity(_));
  backhoeController.update (.4);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
