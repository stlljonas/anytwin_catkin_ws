/** \file actuated_lidar_node.cpp
    \brief This file is the ROS node for the actuated lidar.
  */

// ros
#include <ros/ros.h>

// actuated_lidar
#include "actuated_lidar/ActuatedLidar.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "actuated_lidar");
  ros::NodeHandle nodeHandle("~");

  try {
    actuated_lidar::ActuatedLidar node(nodeHandle);
    ros::spin();
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
