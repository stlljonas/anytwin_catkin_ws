/*
 *  Created on: Dec 1, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>

int main(int argc, char** argv)
{
  sleep(10);
  ros::init(argc, argv, "free_gait_anymal_common_test");
  testing::InitGoogleTest(&argc, argv);
  srand((int) time(0));
  return RUN_ALL_TESTS();
}
