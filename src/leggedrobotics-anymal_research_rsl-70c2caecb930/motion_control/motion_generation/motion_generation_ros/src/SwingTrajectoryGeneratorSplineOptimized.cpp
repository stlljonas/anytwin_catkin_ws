/*
 * SwingTrajectoryGeneratorSplineOptimized.hpp
 *
 *  Created on: Jan. 17, 2018
 *      Author: Fabian Jenelten
 */

#include "motion_generation_ros/SwingTrajectoryGeneratorSplineOptimized.hpp"

namespace anymal_ctrl_dynamic_gaits_ros {


SwingTrajectoryGeneratorSplineOptimized::SwingTrajectoryGeneratorSplineOptimized():
    colors_(),
    numOfSamples_(20u)
{
  publisherRefs_.push_back(plannedSwingTrajectory0_.first);
  publisherRefs_.push_back(plannedSwingTrajectory1_.first);
  publisherRefs_.push_back(plannedSwingTrajectory2_.first);
  publisherRefs_.push_back(plannedSwingTrajectory3_.first);
}

SwingTrajectoryGeneratorSplineOptimized::~SwingTrajectoryGeneratorSplineOptimized() {

}

bool SwingTrajectoryGeneratorSplineOptimized::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  bool success = true;
  nodeHandle_ = nodeHandle;
  success &= initPublishers(topic);
  success &= initMsgs();
  return true;
}

void SwingTrajectoryGeneratorSplineOptimized::setColorVector(const std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>>& colors) {
  colors_ = colors;
}

bool SwingTrajectoryGeneratorSplineOptimized::initPublishers(const std::string& topic) {
  // Initialize publishers.
  plannedSwingTrajectory0_.first = nodeHandle_.advertise<decltype(plannedSwingTrajectory0_.second)>(topic + "planned_swing_trajectory0_markers", 1, true);
  plannedSwingTrajectory1_.first = nodeHandle_.advertise<decltype(plannedSwingTrajectory1_.second)>(topic + "planned_swing_trajectory1_markers", 1, true);
  plannedSwingTrajectory2_.first = nodeHandle_.advertise<decltype(plannedSwingTrajectory2_.second)>(topic + "planned_swing_trajectory2_markers", 1, true);
  plannedSwingTrajectory3_.first = nodeHandle_.advertise<decltype(plannedSwingTrajectory3_.second)>(topic + "planned_swing_trajectory3_markers", 1, true);
  return true;
}

bool SwingTrajectoryGeneratorSplineOptimized::initMsgs() {
  // Initialize markers.
  plannedSwingTrajectory0_.second.markers.clear();
  plannedSwingTrajectory1_.second.markers.clear();
  plannedSwingTrajectory2_.second.markers.clear();
  plannedSwingTrajectory3_.second.markers.clear();

  for (int k=0u; k<numOfSamples_; ++k) {
    plannedSwingTrajectory0_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "planned_swing_trajectory0", k, 0.005));
    plannedSwingTrajectory1_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "planned_swing_trajectory1", k, 0.005));
    plannedSwingTrajectory2_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "planned_swing_trajectory2", k, 0.005));
    plannedSwingTrajectory3_.second.markers.push_back(loco_ros::getInitializedCubeMarker("odom", "planned_swing_trajectory3", k, 0.005));
  }

  return true;
}


bool SwingTrajectoryGeneratorSplineOptimized::update(const std::vector<sto::MotionPlan>& motionPlan)
{
  if (isNotSubscribed() || motionPlan.size()!=4u) {
    return true;
  }

  const ros::Time stamp = ros::Time::now();
  loco::Position positionWorldToDesiredFootInWorldFrame;



  if (plannedSwingTrajectory0_.first.getNumSubscribers() > 0u && motionPlan[0].didOptimizationSucceeded()) {
    unsigned int legId = 0u;
    const double containerDuration = motionPlan[legId].getContainerDuration() - motionPlan[legId].getSanceDuration(); // We plot only the swing trajectory
    const double dt = containerDuration / (numOfSamples_ - 1);
    double splineTime = 0.0;

    for (unsigned int k=0u; k<numOfSamples_; ++k) {

      // Select color (depending on active spline).
      const unsigned int activeSplineId = motionPlan[legId].getActiveSplineId(splineTime);
      const loco::Vector& colors = colors_[activeSplineId];

      // Get position at sample time.
      motionPlan[legId].getPositionWorldToDesiredFootInWorldFrameAtTime(positionWorldToDesiredFootInWorldFrame, splineTime);

      // Plot.
      plannedSwingTrajectory0_.second.markers[k].header.stamp = stamp;
      plannedSwingTrajectory0_.second.markers[k].pose.position.x = positionWorldToDesiredFootInWorldFrame.x();
      plannedSwingTrajectory0_.second.markers[k].pose.position.y = positionWorldToDesiredFootInWorldFrame.y();
      plannedSwingTrajectory0_.second.markers[k].pose.position.z = positionWorldToDesiredFootInWorldFrame.z();

      plannedSwingTrajectory0_.second.markers[k].color.r = colors.x();
      plannedSwingTrajectory0_.second.markers[k].color.g = colors.y();
      plannedSwingTrajectory0_.second.markers[k].color.b = colors.z();

      splineTime += dt;
    }
  }

  if (plannedSwingTrajectory1_.first.getNumSubscribers() > 0u && motionPlan[1].didOptimizationSucceeded()) {
    unsigned int legId = 1u;
    const double containerDuration = motionPlan[legId].getContainerDuration() - motionPlan[legId].getSanceDuration(); // We plot only the swing trajectory
    const double dt = containerDuration / (numOfSamples_ - 1);
    double splineTime = 0.0;
    for (unsigned int k=0u; k<numOfSamples_; ++k) {

      // Select color (depending on active spline).
      const unsigned int activeSplineId = motionPlan[legId].getActiveSplineId(splineTime);
      const loco::Vector& colors = colors_[activeSplineId];

      // Get position at sample time.
      motionPlan[legId].getPositionWorldToDesiredFootInWorldFrameAtTime(positionWorldToDesiredFootInWorldFrame, splineTime);

      // Plot.
      plannedSwingTrajectory1_.second.markers[k].header.stamp = stamp;
      plannedSwingTrajectory1_.second.markers[k].pose.position.x = positionWorldToDesiredFootInWorldFrame.x();
      plannedSwingTrajectory1_.second.markers[k].pose.position.y = positionWorldToDesiredFootInWorldFrame.y();
      plannedSwingTrajectory1_.second.markers[k].pose.position.z = positionWorldToDesiredFootInWorldFrame.z();

      plannedSwingTrajectory1_.second.markers[k].color.r = colors.x();
      plannedSwingTrajectory1_.second.markers[k].color.g = colors.y();
      plannedSwingTrajectory1_.second.markers[k].color.b = colors.z();

      splineTime += dt;
    }
  }

  if (plannedSwingTrajectory2_.first.getNumSubscribers() > 0u && motionPlan[2].didOptimizationSucceeded()) {
    unsigned int legId = 2u;
    const double containerDuration = motionPlan[legId].getContainerDuration() - motionPlan[legId].getSanceDuration(); // We plot only the swing trajectory
    const double dt = containerDuration / (numOfSamples_ - 1);
    double splineTime = 0.0;
    for (unsigned int k=0u; k<numOfSamples_; ++k) {

      // Select color (depending on active spline).
      const unsigned int activeSplineId = motionPlan[legId].getActiveSplineId(splineTime);
      const loco::Vector& colors = colors_[activeSplineId];

      // Get position at sample time.
      motionPlan[legId].getPositionWorldToDesiredFootInWorldFrameAtTime(positionWorldToDesiredFootInWorldFrame, splineTime);

      // Plot.
      plannedSwingTrajectory2_.second.markers[k].header.stamp = stamp;
      plannedSwingTrajectory2_.second.markers[k].pose.position.x = positionWorldToDesiredFootInWorldFrame.x();
      plannedSwingTrajectory2_.second.markers[k].pose.position.y = positionWorldToDesiredFootInWorldFrame.y();
      plannedSwingTrajectory2_.second.markers[k].pose.position.z = positionWorldToDesiredFootInWorldFrame.z();

      plannedSwingTrajectory2_.second.markers[k].color.r = colors.x();
      plannedSwingTrajectory2_.second.markers[k].color.g = colors.y();
      plannedSwingTrajectory2_.second.markers[k].color.b = colors.z();

      splineTime += dt;
    }
  }

  if (plannedSwingTrajectory3_.first.getNumSubscribers() > 0u && motionPlan[3].didOptimizationSucceeded()) {
    unsigned int legId = 3u;
    const double containerDuration = motionPlan[legId].getContainerDuration() - motionPlan[legId].getSanceDuration(); // We plot only the swing trajectory
    const double dt = containerDuration / (numOfSamples_ - 1);
    double splineTime = 0.0;

    for (unsigned int k=0u; k<numOfSamples_; ++k) {

      // Select color (depending on active spline).
      const unsigned int activeSplineId = motionPlan[legId].getActiveSplineId(splineTime);
      const loco::Vector& colors = colors_[activeSplineId];

      // Get position at sample time.
      motionPlan[legId].getPositionWorldToDesiredFootInWorldFrameAtTime(positionWorldToDesiredFootInWorldFrame, splineTime);

      // Plot.
      plannedSwingTrajectory3_.second.markers[k].header.stamp = stamp;
      plannedSwingTrajectory3_.second.markers[k].pose.position.x = positionWorldToDesiredFootInWorldFrame.x();
      plannedSwingTrajectory3_.second.markers[k].pose.position.y = positionWorldToDesiredFootInWorldFrame.y();
      plannedSwingTrajectory3_.second.markers[k].pose.position.z = positionWorldToDesiredFootInWorldFrame.z();

      plannedSwingTrajectory3_.second.markers[k].color.r = colors.x();
      plannedSwingTrajectory3_.second.markers[k].color.g = colors.y();
      plannedSwingTrajectory3_.second.markers[k].color.b = colors.z();

      splineTime += dt;
    }
  }

  return true;
}

bool SwingTrajectoryGeneratorSplineOptimized::publish() {
  loco_ros::publishMsg(plannedSwingTrajectory0_);
  loco_ros::publishMsg(plannedSwingTrajectory1_);
  loco_ros::publishMsg(plannedSwingTrajectory2_);
  loco_ros::publishMsg(plannedSwingTrajectory3_);
  return true;
}


} /* namespace loco_ros */
