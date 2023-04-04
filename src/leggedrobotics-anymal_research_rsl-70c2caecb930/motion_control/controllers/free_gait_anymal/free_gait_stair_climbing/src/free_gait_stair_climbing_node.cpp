/*
 * free_gait_stair_climbing_node.cpp
 *
 *  Created on: Aug 30, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include <free_gait_stair_climbing/StairClimbing.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "free_gait_stair_climbing");

  ros::NodeHandle nodeHandle("~");
  free_gait_stair_climbing::StairClimbing stairClimbing(nodeHandle);

  ros::spin();
  return 0;
}
