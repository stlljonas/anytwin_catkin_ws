/** \file actuated_lidar_master_node.cpp
    \brief This file is the ROS node for the actuated lidar master.
  */

// ros
#include <ros/ros.h>

// actuated_lidar
#include "actuated_lidar/ActuatedLidarMaster.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "actuated_lidar_master");
  ros::NodeHandle nodeHandle("~");

  try {
    actuated_lidar::ActuatedLidarMaster node(nodeHandle);
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
