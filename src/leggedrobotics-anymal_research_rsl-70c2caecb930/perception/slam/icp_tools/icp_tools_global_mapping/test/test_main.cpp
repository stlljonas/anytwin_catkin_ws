
#include <gtest/gtest.h>

// ros
#include <ros/ros.h>

using ::testing::InitGoogleTest;

int main(int argc, char** argv) {
  InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "icp_tools_global_mapping", ros::init_options::NoSigintHandler);

  return RUN_ALL_TESTS();
}
