#include <gtest/gtest.h>
#include <waypoint_controller/waypoint_controller.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>
#include <vector>
#include <utility>

#define PI 3.1415926539f
#define APPROX(A, B, T) ((A > B - T && A < B + T) ? true : false)

using ::testing::NiceMock;

TEST(WaypointControllerTests, instantiateAndAddWaypointReturnsPoints)
{
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  pose wcInitial = {.x = 0, .y = 0, .theta = 0 };
  pose theWay = {.x = 3, .y = 1, .theta = PI / 2.0f };
  WaypointController wc = WaypointController(.5f, .5f, wcInitial, &fl, &fr, &br, &bl);
  std::vector<std::pair<float, float> > returnPoints = wc.addWaypoint(theWay, wcInitial);
  EXPECT_TRUE(returnPoints.size() > 0);
}

TEST(WaypointControllerTests, updateReturnsAStatus)
{
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  pose wcInitial = {.x = 0, .y = 0, .theta = 0 };
  pose theWay = {.x = 3, .y = 1, .theta = PI / 2.0f };
  WaypointController wc = WaypointController(.5f, .5f, wcInitial, &fl, &fr, &br, &bl);
  WaypointController::Status returnStatus = wc.update(wcInitial, .01);

  EXPECT_TRUE(returnStatus == WaypointController::Status::ALLGOOD ||
              returnStatus == WaypointController::Status::GOALREACHED ||
              returnStatus == WaypointController::Status::ALLBAD);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
