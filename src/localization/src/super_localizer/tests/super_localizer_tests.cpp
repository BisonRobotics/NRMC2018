#include <gtest/gtest.h>
#include <super_localizer/super_localizer.h>
#include <vesc_access/mock_vesc_access.h>
#include <can_sensors/mock_pos_can_sensor.h>
#include <can_sensors/mock_imu_can_sensor.h>
#include <gmock/gmock.h>

#include <random>
#include <cmath>

#define POSTOL .03f
#define RADTOL .05f

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Gt;
using ::testing::NiceMock;

TEST(SuperLocalizerTests, residualAccelIsZero){
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosCanSensor> mockPos;
  SuperLocalizer loki (.5f, 0.0f, 0.0f, 0.0f, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
//  LocalizerInterface::stateVector = loki.getResidual();

}




TEST(SuperLocalizerTests, ForwardWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosCanSensor> mockPos;
  Localizer posspoof(.5f, 0.0f,0.0f,0.0f,&flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0.0f,0.0f,0.0f,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(0.3f));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));
  EXPECT_TRUE (loki.getHavePosition());
  EXPECT_FALSE (loki.getHaveImu());
  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f,.1f);

  for (int iter = 0; iter< 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta ()).WillByDefault(Return(posspoof.getStateVector().theta + normnum (generator)));

      loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(posspoof.getStateVector().x_vel, .3f, POSTOL)
              << "Xvel = " << posspoof.getStateVector().x_vel;
  EXPECT_NEAR(posspoof.getStateVector().y_vel, 0.0f, POSTOL)
              << "Yvel = " << posspoof.getStateVector().y_vel;
  EXPECT_NEAR(posspoof.getStateVector().omega, 0.0f, RADTOL)
              << "Omega = " << posspoof.getStateVector().omega;

  EXPECT_NEAR(posspoof.getStateVector().x_pos, .15f, POSTOL)
              << "Xpos = " << posspoof.getStateVector().x_pos;
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 0.0f, POSTOL)
              << "Ypos = " << posspoof.getStateVector().y_pos;
  EXPECT_NEAR(posspoof.getStateVector().theta, 0.0f, RADTOL)
              << "Theta = " << posspoof.getStateVector().theta;

  EXPECT_NEAR(loki.getStateVector().x_vel, .3f, POSTOL)
              << "Xvel = " << loki.getStateVector().x_vel;
  EXPECT_NEAR(loki.getStateVector().y_vel, 0.0f, POSTOL)
              << "Yvel = " << loki.getStateVector().y_vel;
  EXPECT_NEAR(loki.getStateVector().omega, 0.0f, RADTOL)
              << "Omega = " << loki.getStateVector().omega;

  EXPECT_NEAR(loki.getStateVector().x_pos,.15f, POSTOL)
              << "Xpos = " << loki.getStateVector().x_pos;
  EXPECT_NEAR(loki.getStateVector().y_pos, 0.00f, POSTOL)
              << "Ypos = " << loki.getStateVector().y_pos;
  EXPECT_NEAR(loki.getStateVector().theta, 0.0f, RADTOL)
              << "Theta = " << loki.getStateVector().theta;
}

TEST(SuperLocalizerTests, ForwardLeftWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosCanSensor> mockPos;
  Localizer posspoof(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.1));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f,.1f);

  for (int iter = 0; iter< 50; iter++)
  {
      posspoof.updateStateVector(.01f);
      ON_CALL (mockPos, getX()).WillByDefault(Return (posspoof.getStateVector().x_pos + normnum(generator)));
      ON_CALL (mockPos, getY()).WillByDefault(Return (posspoof.getStateVector().y_pos + normnum(generator)));
      ON_CALL (mockPos, getTheta()).WillByDefault(Return (posspoof.getStateVector().theta + normnum(generator)));
      loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, .099f, POSTOL)
              << "Xpos = " << loki.getStateVector().x_pos;
  EXPECT_NEAR(loki.getStateVector().y_pos, 0.01f, POSTOL)
              << "Ypos = " << loki.getStateVector().y_pos;
  EXPECT_NEAR(loki.getStateVector().theta, .2f, RADTOL)
              << "Theta = " << loki.getStateVector().theta;

  EXPECT_NEAR(posspoof.getStateVector().x_pos, .099f, POSTOL)
              << "Xpos = " << posspoof.getStateVector().x_pos;
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 0.01f, POSTOL)
              << "Ypos = " << posspoof.getStateVector().y_pos;
  EXPECT_NEAR(posspoof.getStateVector().theta, .2f, RADTOL)
              << "Theta = " << posspoof.getStateVector().theta;
}

TEST(SuperLocalizerTests, ForwardRightWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosCanSensor> mockPos;
  Localizer posspoof(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f,.1f);

  for (int iter = 0; iter< 50; iter++)
  {
      posspoof.updateStateVector(.01f);
      ON_CALL (mockPos, getX()).WillByDefault(Return (posspoof.getStateVector().x_pos + normnum(generator)));
      ON_CALL (mockPos, getY()).WillByDefault(Return (posspoof.getStateVector().y_pos + normnum(generator)));
      ON_CALL (mockPos, getTheta()).WillByDefault(Return (posspoof.getStateVector().theta + normnum(generator)));
      loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, .099f, POSTOL)
              << "Xpos = " << loki.getStateVector().x_pos;
  EXPECT_NEAR(loki.getStateVector().y_pos, -0.01f, POSTOL)
              << "Ypos = " << loki.getStateVector().y_pos;
  EXPECT_NEAR(loki.getStateVector().theta, -.2f, RADTOL)
              << "Theta = " << loki.getStateVector().theta;

  EXPECT_NEAR(posspoof.getStateVector().x_pos, .099f, POSTOL)
              << "Xpos = " << posspoof.getStateVector().x_pos;
  EXPECT_NEAR(posspoof.getStateVector().y_pos, -0.01f, POSTOL)
              << "Ypos = " << posspoof.getStateVector().y_pos;
  EXPECT_NEAR(posspoof.getStateVector().theta, -.2f, RADTOL)
              << "Theta = " << posspoof.getStateVector().theta;
}

TEST(SuperLocalizerTests, BackwardWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosCanSensor> mockPos;
  Localizer posspoof(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f,.1f);

  for (int iter = 0; iter< 50; iter++)
  {
      posspoof.updateStateVector(.01f);
      ON_CALL (mockPos, getX()).WillByDefault(Return (posspoof.getStateVector().x_pos + normnum(generator)));
      ON_CALL (mockPos, getY()).WillByDefault(Return (posspoof.getStateVector().y_pos + normnum(generator)));
      ON_CALL (mockPos, getTheta()).WillByDefault(Return (posspoof.getStateVector().theta + normnum(generator)));
      loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, -.15f, POSTOL)
              << "Xpos = " << loki.getStateVector().x_pos;
  EXPECT_NEAR(loki.getStateVector().y_pos, 0.0f, POSTOL)
              << "Ypos = " << loki.getStateVector().y_pos;
  EXPECT_NEAR(loki.getStateVector().theta, 0.0f, RADTOL)
              << "Theta = " << loki.getStateVector().theta;

  EXPECT_NEAR(posspoof.getStateVector().x_pos, -.15f, POSTOL)
              << "Xpos = " << posspoof.getStateVector().x_pos;
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 0.00f, POSTOL)
              << "Ypos = " << posspoof.getStateVector().y_pos;
  EXPECT_NEAR(posspoof.getStateVector().theta, 0.00f, RADTOL)
              << "Theta = " << posspoof.getStateVector().theta;
}


TEST(SuperLocalizerTests, BackwardLeftWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosCanSensor> mockPos;
  Localizer posspoof(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.1));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f,.1f);

  for (int iter = 0; iter< 50; iter++)
  {
      posspoof.updateStateVector(.01f);
      ON_CALL (mockPos, getX()).WillByDefault(Return (posspoof.getStateVector().x_pos + normnum(generator)));
      ON_CALL (mockPos, getY()).WillByDefault(Return (posspoof.getStateVector().y_pos + normnum(generator)));
      ON_CALL (mockPos, getTheta()).WillByDefault(Return (posspoof.getStateVector().theta + normnum(generator)));
      loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, -.0993, POSTOL)
              << "Xpos = " << loki.getStateVector().x_pos;
  EXPECT_NEAR(loki.getStateVector().y_pos, 0.01f, POSTOL)
              << "Ypos = " << loki.getStateVector().y_pos;
  EXPECT_NEAR(loki.getStateVector().theta, -.2f, RADTOL)
              << "Theta = " << loki.getStateVector().theta;

  EXPECT_NEAR(posspoof.getStateVector().x_pos, -.0993f, POSTOL)
              << "Xpos = " << posspoof.getStateVector().x_pos;
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 0.01f, POSTOL)
              << "Ypos = " << posspoof.getStateVector().y_pos;
  EXPECT_NEAR(posspoof.getStateVector().theta, -.2f, RADTOL)
              << "Theta = " << posspoof.getStateVector().theta;
}


TEST(SuperLocalizerTests, BackwardRightWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosCanSensor> mockPos;
  Localizer posspoof(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f,.1f);

  for (int iter = 0; iter< 50; iter++)
  {
      posspoof.updateStateVector(.01f);
      ON_CALL (mockPos, getX()).WillByDefault(Return (posspoof.getStateVector().x_pos + normnum(generator)));
      ON_CALL (mockPos, getY()).WillByDefault(Return (posspoof.getStateVector().y_pos + normnum(generator)));
      ON_CALL (mockPos, getTheta()).WillByDefault(Return (posspoof.getStateVector().theta + normnum(generator)));
      loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, -.0993, POSTOL)
              << "Xpos = " << loki.getStateVector().x_pos;
  EXPECT_NEAR(loki.getStateVector().y_pos, -0.01f, POSTOL)
              << "Ypos = " << loki.getStateVector().y_pos;
  EXPECT_NEAR(loki.getStateVector().theta, .2f, RADTOL)
              << "Theta = " << loki.getStateVector().theta;

  EXPECT_NEAR(posspoof.getStateVector().x_pos, -.0993f, POSTOL)
              << "Xpos = " << posspoof.getStateVector().x_pos;
  EXPECT_NEAR(posspoof.getStateVector().y_pos, -0.01f, POSTOL)
              << "Ypos = " << posspoof.getStateVector().y_pos;
  EXPECT_NEAR(posspoof.getStateVector().theta, .2f, RADTOL)
              << "Theta = " << posspoof.getStateVector().theta;
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
