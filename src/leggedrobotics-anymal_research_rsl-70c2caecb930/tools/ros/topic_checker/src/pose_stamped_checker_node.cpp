/*
 * pose_stamped_checker_node.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// ros
#include <geometry_msgs/PoseStamped.h>

// topic checker
#include "topic_checker/TopicChecker.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_stamped_checker_node");
  ros::NodeHandle nodeHandle("~");

  topic_checker::TopicChecker<geometry_msgs::PoseStamped> topicChecker(nodeHandle);

  ros::spin();
  return 0;
}
