/*
 * elevation_map_processing_node.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: Aravind Elanjimattathil Vijayan
 *   Institute: ETH Zurich
 */

#include <ros/ros.h>
#include "elevation_map_processing/ElevationMapProcessingRos.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_map_processing");
  ros::NodeHandle nodeHandle("~");
  elevation_map_processing::ElevationMapProcessingRos elevationMapProcessingRos(nodeHandle);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
