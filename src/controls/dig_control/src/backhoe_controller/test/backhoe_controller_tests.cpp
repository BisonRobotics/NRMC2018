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
  NiceMock <MockVescAccess> shoulder_vesc;
  NiceMock <MockVescAccess> wrist_vesc;
  backhoecontroller::joint_params_t wrist_params = {.minimum_pos=0.0, .maximum_pos=.7,
                                                    .safety_check_pos=.2, .gain=.5, .setpoint_tolerance=.01};
  backhoecontroller::joint_params_t shoulder_params = {.minimum_pos = -1.4, .maximum_pos=1.4,
                                                    .safety_check_pos=SAFE_CENTRAL_ANGLE, .gain=.5,
                                                    .setpoint_tolerance=.01};
  ON_CALL(wrist_vesc, getLimitSwitchState()).WillByDefault(Return(nsVescAccess::limitSwitchState::bottomOfMotion));
  EXPECT_CALL(wrist_vesc, setTorque(Lt(0)));
  EXPECT_CALL (wrist_vesc, setLinearVelocity(0));
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
  backhoeController.init ();                // initialize the backhoe
  ON_CALL(wrist_vesc, getLinearVelocity()).WillByDefault(Return(1.0f)); // have the wrist vesc say its going upwards
  ON_CALL (shoulder_vesc, getPotPosition()).WillByDefault(Return(SAFE_CENTRAL_ANGLE));
  backhoeController.update (wrist_params.safety_check_pos+.08);           // set it up so we are in the safety  position
  backhoeController.setShoulderSetpoint(1.0);
  backhoeController.setWristSetpoint(0);
  EXPECT_CALL (shoulder_vesc, setLinearVelocity(0));            // expect that the central drive velocity will get set to 0
  backhoeController.update(1.0);
}

TEST(backhoe_controller_test, wont_set_velocity_past_limit)
{

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
