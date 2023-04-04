/*
 * StateCheckerPublisher.cpp
 *
 *  Created on: Sept 1, 2019
 *      Author: Fabian Jenelten
 */


// anymal ctrl dynamic gaits ros.
#include "motion_generation_ros/StateCheckerPublisher.hpp"

namespace anymal_ctrl_dynamic_gaits_ros {

bool StateCheckerPublisher::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  nodeHandle_ = nodeHandle;
  publisher_ = nodeHandle_.advertise<decltype(isSafe_)>(topic, 1);
  return true;
}

bool StateCheckerPublisher::shutdown() {
  publisher_.shutdown();
  return true;
}

bool StateCheckerPublisher::update(bool isSafe) {
  isSafe_.data = static_cast<float>(isSafe);
  return true;
}

bool StateCheckerPublisher::publish() {
  loco_ros::publishMsg(publisher_, isSafe_);
  return true;
}

unsigned int StateCheckerPublisher::getNumSubscribers() const {
  return publisher_.getNumSubscribers();
}

} /* namespace anymal_ctrl_dynamic_gaits_ros */
