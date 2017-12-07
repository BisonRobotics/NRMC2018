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

TEST(SuperLocalizerTests, ForwardWithPos)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  MockPosCanSensor mockPos;
  Localizer posspoof(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc);
  
  SuperLocalizer loki(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, (Localizer::stateVector_s) SuperLocalizer_default_gains);
  EXPECT_CALL(flvesc, getLinearVelocity()).WillRepeatedly(Return(.3));
  EXPECT_CALL(frvesc, getLinearVelocity()).WillRepeatedly(Return(.3));
  EXPECT_CALL(brvesc, getLinearVelocity()).WillRepeatedly(Return(.3));
  EXPECT_CALL(blvesc, getLinearVelocity()).WillRepeatedly(Return(.3));
  //EXPECT_CALL(flvesc, getLinearVelocity());
  //EXPECT_CALL(frvesc, getLinearVelocity());
  //EXPECT_CALL(brvesc, getLinearVelocity());
  //EXPECT_CALL(blvesc, getLinearVelocity());

  EXPECT_CALL(mockPos, receiveData()).WillRepeatedly(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f,.0002f);

  for (int iter = 0; iter < 50; iter++) //.5 seconds at dt = .01
  {
      EXPECT_CALL(mockPos, getX())
            .WillOnce(Return(posspoof.getStateVector().x_pos + normnum(generator)))
		    .RetiresOnSaturation();
      EXPECT_CALL(mockPos, getY())
            .WillOnce(Return(posspoof.getStateVector().y_pos + normnum(generator)))
    		.RetiresOnSaturation();
      EXPECT_CALL(mockPos, getTheta())
            .WillOnce(Return(posspoof.getStateVector().theta + normnum(generator)))
    		.RetiresOnSaturation();
			posspoof.updateStateVector(.01f);
  }

  for (int iter = 0; iter< 50; iter++) 
  {
      loki.updateStateVector(.01f);
  }

  ASSERT_NEAR(loki.getStateVector().x_pos,.15f, POSTOL) 
              << "Xpos = " << loki.getStateVector().x_pos;
  ASSERT_NEAR(loki.getStateVector().y_pos, 0.00f, POSTOL) 
              << "Ypos = " << loki.getStateVector().y_pos;
  ASSERT_NEAR(loki.getStateVector().theta, 0.0f, RADTOL) 
              << "Theta = " << loki.getStateVector().theta;

  ASSERT_NEAR(posspoof.getStateVector().x_pos, .15f, POSTOL) 
              << "Xpos = " << posspoof.getStateVector().x_pos;
  ASSERT_NEAR(posspoof.getStateVector().y_pos, 0.0f, POSTOL) 
              << "Ypos = " << posspoof.getStateVector().y_pos;
  ASSERT_NEAR(posspoof.getStateVector().theta, 0.0f, RADTOL) 
              << "Theta = " << posspoof.getStateVector().theta;
}

TEST(SuperLocalizerTests, ForwardLeftWithPos)
{
  MockVescAccess flvesc, frvesc, brvesc, blvesc;
  MockPosCanSensor mockPos;
  Localizer posspoof(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc);
  
  SuperLocalizer loki(.5f, 0,0,0,&flvesc, &frvesc, &brvesc, &blvesc, &mockPos, (Localizer::stateVector_s) SuperLocalizer_default_gains);
  EXPECT_CALL(flvesc, getLinearVelocity()).WillRepeatedly(Return(.1));
  EXPECT_CALL(frvesc, getLinearVelocity()).WillRepeatedly(Return(.3));
  EXPECT_CALL(brvesc, getLinearVelocity()).WillRepeatedly(Return(.3));
  EXPECT_CALL(blvesc, getLinearVelocity()).WillRepeatedly(Return(.1));
  //EXPECT_CALL(flvesc, getLinearVelocity());
  //EXPECT_CALL(frvesc, getLinearVelocity());
  //EXPECT_CALL(brvesc, getLinearVelocity());
  //EXPECT_CALL(blvesc, getLinearVelocity());

  EXPECT_CALL(mockPos, receiveData()).WillRepeatedly(Return(ReadableSensors::ReadStatus::READ_SUCCESS));

  std::default_random_engine generator;
  std::normal_distribution<float> normnum(0.0f,.0002f);

  for (int iter = 0; iter < 50; iter++) //.5 seconds at dt = .01
  {
      EXPECT_CALL(mockPos, getX())
            .WillOnce(Return(posspoof.getStateVector().x_pos + normnum(generator)))
		    .RetiresOnSaturation();
      EXPECT_CALL(mockPos, getY())
            .WillOnce(Return(posspoof.getStateVector().y_pos + normnum(generator)))
    		.RetiresOnSaturation();
      EXPECT_CALL(mockPos, getTheta())
            .WillOnce(Return(posspoof.getStateVector().theta + normnum(generator)))
    		.RetiresOnSaturation();
			posspoof.updateStateVector(.01f);
  }

  for (int iter = 0; iter< 50; iter++) 
  {
      loki.updateStateVector(.01f);
  }

  ASSERT_NEAR(loki.getStateVector().x_pos, .099f, POSTOL) 
              << "Xpos = " << loki.getStateVector().x_pos;
  ASSERT_NEAR(loki.getStateVector().y_pos, 0.01f, POSTOL) 
              << "Ypos = " << loki.getStateVector().y_pos;
  ASSERT_NEAR(loki.getStateVector().theta, .2f, RADTOL) 
              << "Theta = " << loki.getStateVector().theta;

  ASSERT_NEAR(posspoof.getStateVector().x_pos, .099f, POSTOL) 
              << "Xpos = " << posspoof.getStateVector().x_pos;
  ASSERT_NEAR(posspoof.getStateVector().y_pos, 0.01f, POSTOL) 
              << "Ypos = " << posspoof.getStateVector().y_pos;
  ASSERT_NEAR(posspoof.getStateVector().theta, .2f, RADTOL) 
              << "Theta = " << posspoof.getStateVector().theta;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}