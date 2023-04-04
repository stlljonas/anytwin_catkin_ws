/*
 * locomotion_planner_node.cpp
 *
 *  Created on: Mar 4, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <locomotion_planner/LocomotionPlannerRos.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "locomotion_planner");

  ros::NodeHandle nodeHandle("~");
  locomotion_planner::LocomotionPlannerRos locomotionPlanner(nodeHandle);

  ros::spin();
  return 0;
}
