/*!
* @file    test_main.cpp
* @author  Perry Franklin
* @date    Jun 3, 2017
*/

#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>

/// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "collisions_fcl_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
