#include "vesc_access/mock_vesc_access.h"
#include "gtest/gtest.h"
#include "outrigger_controller/outrigger_controller.h"
#include "gmock/gmock.h"
#include "wheel_params/wheel_params.h"

using ::testing::NiceMock;
using ::testing::Return;
using ::testing::_;

TEST(outrigger_controller_test,
     update_modifies_isRetracted_when_called_after_retract_with_a_dt_value_enough_times_to_sum_to_TIME_TO_ACTUATE)
{
  NiceMock<MockVescAccess> l_vesc;
  NiceMock<MockVescAccess> r_vesc;
  OutriggerController oc(&l_vesc, &r_vesc);
  oc.retract();
  double dt = OutriggerController::TIME_TO_ACTUATE / 50.0;
  for (double time = 0; time <= 1.5 * OutriggerController::TIME_TO_ACTUATE; time += dt)
  {
    if (time <= .95 * OutriggerController::TIME_TO_ACTUATE)
    {
      EXPECT_FALSE(oc.isRetracted());
    }
    else if (time >= 1.05 * OutriggerController::TIME_TO_ACTUATE)
    {
      EXPECT_TRUE(oc.isRetracted());
    }
    oc.update(dt);
  }
}

TEST(outrigger_controller_test,
     update_modifies_isDeployed_when_called_after_deploy_with_a_dt_value_enough_times_to_sum_to_TIME_TO_ACTUATE)
{
  NiceMock<MockVescAccess> l_vesc;
  NiceMock<MockVescAccess> r_vesc;
  OutriggerController oc(&l_vesc, &r_vesc);
  oc.deploy();
  double dt = OutriggerController::TIME_TO_ACTUATE / 50.0;
  for (double time = 0; time <= 1.5 * OutriggerController::TIME_TO_ACTUATE; time += dt)
  {
    if (time <= .95 * OutriggerController::TIME_TO_ACTUATE)
    {
      EXPECT_FALSE(oc.isDeployed());
    }
    else if (time >= 1.05 * OutriggerController::TIME_TO_ACTUATE)
    {
      EXPECT_TRUE(oc.isDeployed());
    }
    oc.update(dt);
  }
}

TEST(outrigger_controller_test, retract_calls_setDuty_on_both_vescs)
{
  NiceMock<MockVescAccess> l_vesc;
  NiceMock<MockVescAccess> r_vesc;
  OutriggerController oc(&l_vesc, &r_vesc);
  EXPECT_CALL(l_vesc, setDuty(_)).Times(1);
  EXPECT_CALL(r_vesc, setDuty(_)).Times(1);
  oc.retract();
}

TEST(outrigger_controller_test, deploy_calls_setDuty_on_both_vescs)
{
  NiceMock<MockVescAccess> l_vesc;
  NiceMock<MockVescAccess> r_vesc;
  OutriggerController oc(&l_vesc, &r_vesc);
  EXPECT_CALL(l_vesc, setDuty(_)).Times(1);
  EXPECT_CALL(r_vesc, setDuty(_)).Times(1);
  oc.deploy();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
