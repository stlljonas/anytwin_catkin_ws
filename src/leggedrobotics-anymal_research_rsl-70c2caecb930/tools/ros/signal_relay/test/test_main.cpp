#include <gtest/gtest.h>

// ros
#include <ros/ros.h>

using ::testing::InitGoogleTest;

int main(int argc, char** argv) {
  InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "signal_relay", ros::init_options::NoSigintHandler);
  int val = RUN_ALL_TESTS();

  return val;
}
