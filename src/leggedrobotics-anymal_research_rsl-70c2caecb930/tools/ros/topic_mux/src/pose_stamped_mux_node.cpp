/*
 * pose_stamped_mux_node.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// ros
#include <geometry_msgs/PoseStamped.h>

// topic mux
#include "topic_mux/TopicMux.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_stamped_mux_node");
  ros::NodeHandle nodeHandle("~");

  topic_mux::TopicMux<geometry_msgs::PoseStamped> topicMux(nodeHandle);

  ros::spin();
  return 0;
}
