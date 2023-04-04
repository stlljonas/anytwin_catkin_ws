/*
 * tf_forward_node.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// tf
#include <tf/tfMessage.h>

// topic forward
#include "topic_forward/TopicForward.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_forward_node");
  ros::NodeHandle nh("~");

  topic_forward::TopicForward<tf::tfMessage> topicForward(nh);

  ros::spin();
  return 0;
}
