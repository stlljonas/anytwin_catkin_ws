/*
 * twist_stamped_checker_node.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// ros
#include <geometry_msgs/TwistStamped.h>

// topic checker
#include "topic_checker/TopicChecker.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_stamped_checker_node");
  ros::NodeHandle nodeHandle("~");

  topic_checker::TopicChecker<geometry_msgs::TwistStamped> topicChecker(nodeHandle);

  ros::spin();
  return 0;
}
