#include <gtest/gtest.h>
#include <super_localizer/super_loclizer.h>
#include <vesc_access/mock_vesc_access.h>
#include <gmock/gmock.h>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;


TEST(SuperLocalizerTests, Forward)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  SuperLocalizer loki(&flvesc, &frvesc, &brvesc, &blvesc);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.5));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));

  ASSERT_TRUE(FALSE);
  
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}