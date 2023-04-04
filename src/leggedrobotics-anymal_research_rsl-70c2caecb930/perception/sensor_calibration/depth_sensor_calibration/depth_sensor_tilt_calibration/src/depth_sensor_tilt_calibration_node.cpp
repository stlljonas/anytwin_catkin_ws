/*
 * depth_sensor_tilt_calibration_node.cpp
 *
 *  Created on: Nov 27, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "depth_sensor_tilt_calibration/DepthSensorTiltCalibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_sensor_tilt_calibration");

  ros::NodeHandle nodeHandle("~");

  depth_sensor_tilt_calibration::DepthSensorTiltCalibration depthSensorTiltCalibration(nodeHandle);

  ros::spin();
  return 0;
}
