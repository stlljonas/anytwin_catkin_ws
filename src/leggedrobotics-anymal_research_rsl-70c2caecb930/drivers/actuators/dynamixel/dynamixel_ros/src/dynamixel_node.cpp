/** \file dynamixel_node.cpp
    \brief This file is the ROS node for Dynamixel devices.
  */

#include <ros/ros.h>

#include "dynamixel_ros/DynamixelNode.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamixel", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");
  try {
    dynamixel_ros::DynamixelNode node(nh);
    node.spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception");
    return 1;
  }
  return 0;
}
