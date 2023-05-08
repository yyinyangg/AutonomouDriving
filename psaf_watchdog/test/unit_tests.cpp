/**
 * @file unit_tests.cpp
 * @brief The unit tests for the watchdog node.
 * @author PSAF
 * @date 2022-06-01
 * @todo implement unit tests
 */
#include "gtest/gtest.h"


TEST(WatchdogSampleTest, Test1) {
  EXPECT_EQ(1, 1);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
