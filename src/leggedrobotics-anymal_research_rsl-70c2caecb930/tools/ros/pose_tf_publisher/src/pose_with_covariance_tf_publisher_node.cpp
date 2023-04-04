/*
 * pose_with_covariance_tf_publisher_node.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// ros
#include <ros/ros.h>

// pose tf publisher
#include "pose_tf_publisher/PoseTfPublisher.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_tf_publisher");
  ros::NodeHandle nodeHandle("~");

  pose_tf_publisher::PoseTfPublisher<geometry_msgs::PoseWithCovarianceStamped> tfPublisher(nodeHandle);

  ros::spin();
  return 0;
}
