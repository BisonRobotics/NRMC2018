// Bring in da gtest
#include <gtest/gtest.h>

/*class testSensor: public can_sensor {
  public:
  typedef struct testData_S {
    int data;
  } testData;
  recieveData(*testData)
  {

  }
}*/

// I do declare a test!
TEST(TestSuite, testCase1)
{
  // init test can_sensor  object
  // testSensor mySensor(22);
  ASSERT_TRUE(true);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
