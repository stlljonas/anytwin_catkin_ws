/*
 * FinalComBox.cpp
 *
 *  Created on: Feb 1, 2017
 *      Author: Dario Bellicoso
 */


// anymal ctrl dynamic gaits ros
#include "motion_generation_ros/FinalComBox.hpp"

namespace anymal_ctrl_dynamic_gaits_ros {

bool FinalComBox::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  nodeHandle_ = nodeHandle;
  publisher_ = nodeHandle.advertise<decltype(box_)>(topic, 1);

  box_.action = visualization_msgs::Marker::ADD;
  box_.type = visualization_msgs::Marker::CUBE;
  box_.header.frame_id = "odom";
  box_.ns = "final_com_box";
  box_.color.a = 0.5;

  return true;
}

bool FinalComBox::shutdown() {
  publisher_.shutdown();
  return true;
}

bool FinalComBox::update(const zmp::MotionPlan& motionPlan) {
  if (!motionPlan.didOptimizationSucceeded()) { return true; }

  if (getNumSubscribers() > 0u) {
    if (motionPlan.getComFinalBox().finalMaxStateBox_.size()>0u) {
      const loco::RotationQuaternion& orientationPlaneToWorld = motionPlan.getVirtualPlaneFrame().getPosePlaneToWorld().getRotation();
      const loco::Position& positionWorldToBoxInWorldFrame = motionPlan.getVirtualPlaneFrame().getPosePlaneToWorld().transform(
          motionPlan.getComFinalBox().positionPlaneToCenterInPlaneFrame_);

      box_.scale.x = 2.0*motionPlan.getComFinalBox().finalMaxStateBox_[zmp::CogDim::x];
      box_.scale.y = 2.0*motionPlan.getComFinalBox().finalMaxStateBox_[zmp::CogDim::y];
      box_.scale.z = 2.0*motionPlan.getComFinalBox().finalMaxStateBox_[zmp::CogDim::z];

      box_.pose.position.x = positionWorldToBoxInWorldFrame.x();
      box_.pose.position.y = positionWorldToBoxInWorldFrame.y();
      box_.pose.position.z = positionWorldToBoxInWorldFrame.z();
      box_.pose.orientation.w = orientationPlaneToWorld.w();
      box_.pose.orientation.x = orientationPlaneToWorld.x();
      box_.pose.orientation.y = orientationPlaneToWorld.y();
      box_.pose.orientation.z = orientationPlaneToWorld.z();
    }
  }

  return true;
}

bool FinalComBox::publish() {
  loco_ros::publishMsg(publisher_, box_);
  return true;
}

unsigned int FinalComBox::getNumSubscribers() const {
  return publisher_.getNumSubscribers();
}

} /* namespace anymal_ctrl_dynamic_gaits_ros */
