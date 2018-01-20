#include <gtest/gtest.h>
#include <super_localizer/super_localizer.h>
#include <vesc_access/mock_vesc_access.h>
#include <sensor_access/mock_pos_sensor.h>
#include <sensor_access/mock_imu_sensor.h>
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

TEST(SuperLocalizerTests, CanTurnInPlacecw)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 10.0f, 10.0f, 0.0f, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 10.0f, 10.0f, 0.0f, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos,
                      SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(1.0f));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-1.0f));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-1.0f));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(1.0f));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));
  EXPECT_TRUE(loki.getHavePosition());
  EXPECT_FALSE(loki.getHaveImu());
  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .05f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));

    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(posspoof.getStateVector().x_vel, 0.0f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_vel, 0.0f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().omega, -4.0f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, 10.f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 10.0f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, -2.04f, RADTOL);

  EXPECT_NEAR(loki.getStateVector().x_vel, 0.0f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_vel, 0.0f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().omega, -4.0f, RADTOL);

  EXPECT_NEAR(loki.getStateVector().x_pos, 10.0f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 10.00f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, -2.04f, RADTOL);
}

TEST(SuperLocalizerTests, ForwardWithPosInit)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 10.0f, 10.0f, 0.0f, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 10.0f, 10.0f, 0.0f, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos,
                      SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(0.3f));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));
  EXPECT_TRUE(loki.getHavePosition());
  EXPECT_FALSE(loki.getHaveImu());
  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(posspoof.getStateVector().x_vel, 0.3f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_vel, 0.0f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().omega, 0.0f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, 10.15f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 10.0f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, 0.0f, RADTOL);

  EXPECT_NEAR(loki.getStateVector().x_vel, 0.3f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_vel, 0.0f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().omega, 0.0f, RADTOL);

  EXPECT_NEAR(loki.getStateVector().x_pos, 10.15f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 10.00f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, 0.0f, RADTOL);
}

TEST(SuperLocalizerTests, ForwardLeftWithPosInit)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 10.0f, 10.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 10.0f, 10.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.1));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, 10.101f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 10.01f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, .204f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, 10.101f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 10.01f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, .204f, RADTOL);
}

TEST(SuperLocalizerTests, ForwardRightWithPosInit)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 5.0f, 5.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 5.0f, 5.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, 5.101f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 5.0f - 0.01f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, -.204f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, 5.0f + .101f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 5.0f - 0.01f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, -.204f, RADTOL);
}

TEST(SuperLocalizerTests, BackwardWithPosInit)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 10.0f, 10.f, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 10.0f, 10.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, 10.0f - .153f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 10.0f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, 0.0f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, 10.0f - .153f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 10.00f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, 0.00f, RADTOL);
}

TEST(SuperLocalizerTests, BackwardLeftWithPosInit)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 10.0f, 10.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 10.0f, 10.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.1));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, 10.0f - .101f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 10.0104f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, -.204f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, 10.0f - .101f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 10.0104f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, -.204f, RADTOL);
}

TEST(SuperLocalizerTests, BackwardRightWithPosInit)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 10.0f, 10.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 10.0f, 10.0f, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, 10.0f - .1013, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 10.0f - 0.0104f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, .204f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, 10.0f - .1013f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 10.0f - 0.0104f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, .204f, RADTOL);
}

TEST(SuperLocalizerTests, ForwardWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 0.0f, 0.0f, 0.0f, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0.0f, 0.0f, 0.0f, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos,
                      SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(0.3f));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(0.3f));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));
  EXPECT_TRUE(loki.getHavePosition());
  EXPECT_FALSE(loki.getHaveImu());
  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(posspoof.getStateVector().x_vel, .3f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_vel, 0.0f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().omega, 0.0f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, .15f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 0.0f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, 0.0f, RADTOL);

  EXPECT_NEAR(loki.getStateVector().x_vel, .3f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_vel, 0.0f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().omega, 0.0f, RADTOL);

  EXPECT_NEAR(loki.getStateVector().x_pos, .15f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 0.00f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, 0.0f, RADTOL);
}

TEST(SuperLocalizerTests, ForwardLeftWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.1));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, .099f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 0.01f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, .2f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, .099f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 0.01f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, .2f, RADTOL);
}

TEST(SuperLocalizerTests, ForwardRightWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(.1));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, .099f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, -0.01f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, -.2f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, .099f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, -0.01f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, -.2f, RADTOL);
}

TEST(SuperLocalizerTests, BackwardWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, -.15f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 0.0f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, 0.0f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, -.15f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 0.00f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, 0.00f, RADTOL);
}

TEST(SuperLocalizerTests, BackwardLeftWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.1));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, -.101, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, 0.0104f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, -.2f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, -.101f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, 0.0104f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, -.2f, RADTOL);
}

TEST(SuperLocalizerTests, BackwardRightWithPos)
{
  NiceMock<MockVescAccess> flvesc, frvesc, brvesc, blvesc;
  NiceMock<MockPosSensor> mockPos;

  Localizer posspoof(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc);

  SuperLocalizer loki(.5f, 0, 0, 0, &flvesc, &frvesc, &brvesc, &blvesc, &mockPos, SuperLocalizer_default_gains);
  ON_CALL(flvesc, getLinearVelocity()).WillByDefault(Return(-.3));
  ON_CALL(frvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(brvesc, getLinearVelocity()).WillByDefault(Return(-.1));
  ON_CALL(blvesc, getLinearVelocity()).WillByDefault(Return(-.3));

  ON_CALL(mockPos, receiveData()).WillByDefault(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f, .1f);

  for (int iter = 0; iter < 50; iter++)
  {
    posspoof.updateStateVector(.01f);
    ON_CALL(mockPos, getX()).WillByDefault(Return(posspoof.getStateVector().x_pos + normnum(generator)));
    ON_CALL(mockPos, getY()).WillByDefault(Return(posspoof.getStateVector().y_pos + normnum(generator)));
    ON_CALL(mockPos, getTheta()).WillByDefault(Return(posspoof.getStateVector().theta + normnum(generator)));
    loki.updateStateVector(.01f);
  }

  EXPECT_NEAR(loki.getStateVector().x_pos, -.101, POSTOL);
  EXPECT_NEAR(loki.getStateVector().y_pos, -0.0104f, POSTOL);
  EXPECT_NEAR(loki.getStateVector().theta, .204f, RADTOL);

  EXPECT_NEAR(posspoof.getStateVector().x_pos, -.101f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().y_pos, -0.0104f, POSTOL);
  EXPECT_NEAR(posspoof.getStateVector().theta, .204f, RADTOL);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
