/**
 * @file unit_tests.cpp
 * @brief The unit tests for the parking node
 * @author PSAF
 * @date 2022-06-01
 */
#include "gtest/gtest.h"


TEST(ParkingSampleTest, Test1) {
  EXPECT_EQ(1, 1);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
