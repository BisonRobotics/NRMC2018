#include <gtest/gtest.h>
#include <waypoint_controller/waypoint_controller.h>
#include <vesc_access/mock_vesc_access.h>
#include <sim_robot/sim_robot.h>
#include <gmock/gmock.h>
#include <vector>
#include <utility>

#define _USE_MATH_DEFINES
#include <cmath>

using ::testing::NiceMock;

TEST(WaypointControllerTests, instantiateAndAddWaypointReturnsPoints)
{
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  pose wcInitial = {.x = 0, .y = 0, .theta = 0 };
  pose theWay = {.x = 3, .y = 1, .theta = M_PI_2 };
  WaypointController wc = WaypointController(.5f, .5f, wcInitial, &fl, &fr, &br, &bl, .02, waypoint_default_gains);
  std::vector<std::pair<double, double> > returnPoints = wc.addWaypoint(theWay, wcInitial);
  EXPECT_TRUE(returnPoints.size() > 0);
}

TEST(WaypointControllerTests, updateReturnsAStatus)
{
  SimRobot sim(.5f, 0, 0, 0, 16);

  iVescAccess *fl = (sim.getFLVesc());
  iVescAccess *fr = (sim.getFRVesc());
  iVescAccess *br = (sim.getBRVesc());
  iVescAccess *bl = (sim.getBLVesc());

  pose wcInitial = {.x = 0, .y = 0, .theta = 0 };
  pose theWay = {.x = 3, .y = 0, .theta = 0 };
  WaypointController wc = WaypointController(.5f, .5f, wcInitial, fl, fr, br, bl, .01, waypoint_default_gains);
  WaypointController::Status returnStatus = wc.update(sim.getStates(), .01);

  EXPECT_TRUE(
      returnStatus == WaypointController::Status::ALLGOOD || returnStatus == WaypointController::Status::GOALREACHED ||
      returnStatus == WaypointController::Status::OVERSHOT || returnStatus == WaypointController::Status::OFFPATH ||
      returnStatus == WaypointController::Status::CANTPLAN || returnStatus == WaypointController::Status::ISSTUCK);
}

TEST(WaypointControllerTests, ableToAddWaypoint_FrontThenRight)
{
  SimRobot sim(.5f, 0, 0, 0, 16);
  /*
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  */
  iVescAccess *fl = (sim.getFLVesc());
  iVescAccess *fr = (sim.getFRVesc());
  iVescAccess *br = (sim.getBRVesc());
  iVescAccess *bl = (sim.getBLVesc());

  pose wcInitial = {.x = 0, .y = 0, .theta = 0 };
  pose theWay = {.x = 3, .y = 0, .theta = 0 };
  WaypointController wc = WaypointController(.5f, .5f, wcInitial, fl, fr, br, bl, .01, waypoint_default_gains);

  wc.addWaypoint(theWay, wcInitial);
  pose currPose;
  for (int loop = 0; loop < 1000; loop++)
  {
    currPose.x = sim.getX();
    currPose.y = sim.getY();
    currPose.theta = sim.getTheta();
    wc.update(sim.getStates(), .01);
    sim.update(.01);
    ASSERT_NEAR(wc.getETpEstimate(), 0, 4) << "loop index: " << loop << "\nSim Robot Pose:\n"
                                           << "X: " << sim.getX() << "\nY: " << sim.getY() << "\nTh: " << sim.getTheta()
                                           << "\n";  // angle stay within .5 rad (~30deg)
    ASSERT_NEAR(wc.getEPpEstimate(), 0, .3) << "loop index: " << loop << "\nSim Robot Pose:\n"
                                            << "X: " << sim.getX() << "\nY: " << sim.getY()
                                            << "\nTh: " << sim.getTheta() << "\n";  // path error below 10cm
  }
}

TEST(WaypointControllerTests, ableToAddWaypoint_BackAndLeft2Turn)
{
  pose wcInitial = {.x = 3, .y = 0, .theta = 0 };
  pose theWay = {.x = 1, .y = .5, .theta = 0 };

  SimRobot sim(.5f, wcInitial.x, wcInitial.y, wcInitial.theta, 16);

  iVescAccess *fl = (sim.getFLVesc());
  iVescAccess *fr = (sim.getFRVesc());
  iVescAccess *br = (sim.getBRVesc());
  iVescAccess *bl = (sim.getBLVesc());

  WaypointController wc = WaypointController(.5f, .5f, wcInitial, fl, fr, br, bl, .01, waypoint_default_gains);

  wc.addWaypoint(theWay, wcInitial);
  pose currPose;
  for (int loop = 0; loop < 1000; loop++)
  {
    currPose.x = sim.getX();
    currPose.y = sim.getY();
    currPose.theta = sim.getTheta();
    wc.update(sim.getStates(), .01);
    sim.update(.01);
    ASSERT_NEAR(wc.getETpEstimate(), 0, 4) << "loop index: " << loop << "\nSim Robot Pose:\n"
                                           << "X: " << sim.getX() << "\nY: " << sim.getY() << "\nTh: " << sim.getTheta()
                                           << "\n";  // angle stay within .5 rad (~30deg)
    ASSERT_NEAR(wc.getEPpEstimate(), 0, .3) << "loop index: " << loop << "\nSim Robot Pose:\n"
                                            << "X: " << sim.getX() << "\nY: " << sim.getY()
                                            << "\nTh: " << sim.getTheta() << "\n";  // path error below 10cm
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
