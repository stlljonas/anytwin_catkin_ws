/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Main test file
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

using ::testing::InitGoogleTest;

/* RUN TESTS */
int main(int argc, char** argv) {
  ros::init(argc, argv, "geometry_utils_ros");
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
