
// Bring in gtest
#include <gtest/gtest.h>
#include <teleop_interface/teleop_interface.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;
// Declare a test

TEST(TeleopControllerTests, canBeInstantiatedWithAVelocity)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  TeleopInterface tele = TeleopInterface(velocity, &fl, &fr, &br, &bl);

  EXPECT_EQ(tele.getVelocity(), velocity);
}

TEST(TeleopControllerTests, startsWithNoVelocity)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  EXPECT_CALL(br, setLinearVelocity(0.0f));
  EXPECT_CALL(bl, setLinearVelocity(0.0f));
  EXPECT_CALL(fr, setLinearVelocity(0.0f));
  EXPECT_CALL(fl, setLinearVelocity(0.0f));

  TeleopInterface tele = TeleopInterface(velocity, &fl, &fr, &br, &bl);
}

TEST(TeleopControllerTests, takesAbsOfMaxVelocity)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;
  TeleopInterface tele = TeleopInterface(-1.0f * velocity, &fl, &fr, &br, &bl);

  EXPECT_EQ(tele.getVelocity(), velocity);
}

TEST(TeleopControllerTests, canBeUpdated)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  EXPECT_CALL(br, setLinearVelocity(10.0f));
  EXPECT_CALL(bl, setLinearVelocity(10.0f));
  EXPECT_CALL(fr, setLinearVelocity(10.0f));
  EXPECT_CALL(fl, setLinearVelocity(10.0f));
  TeleopInterface tele = TeleopInterface(velocity, &fl, &fr, &br, &bl);

  tele.update(1.0f, 1.0f);
}
/*

TEST(TeleopControllerTests, saturatesVelocities)
{
  float velocity = 10.0f;
  NiceMock<MockVescAccess> br;
  NiceMock<MockVescAccess> bl;
  NiceMock<MockVescAccess> fr;
  NiceMock<MockVescAccess> fl;

  EXPECT_CALL (br, setLinearVelocity(10.0f));
  EXPECT_CALL (bl, setLinearVelocity(10.0f));
  EXPECT_CALL (fr, setLinearVelocity(10.0f));
  EXPECT_CALL (fl, setLinearVelocity(10.0f));
  TeleopInterface tele = TeleopInterface(velocity, &fl, &fr, &br, &bl);

  tele.update (2.0f, 2.0f);
}
*/
// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
