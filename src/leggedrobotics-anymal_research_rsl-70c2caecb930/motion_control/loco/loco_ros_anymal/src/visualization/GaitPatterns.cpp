/*
 * GaitPatterns.hpp
 *
 *  Created on: Dec 7, 2015
 *      Author: Christian Gehring, Dario Bellicoso
 */

// anymal_description
#include <anymal_description/LegEnum.hpp>

// loco ros anymal
#include "loco_ros_anymal/visualization/GaitPatterns.hpp"

// loco ros
#include <loco_ros/loco_ros.hpp>

namespace loco_ros_anymal {

GaitPatterns::GaitPatterns() {
  publisherRefs_.push_back(publisher_);
}

bool GaitPatterns::initialize(ros::NodeHandle& nodeHandle, const std::string& topic) {
  publisher_ = nodeHandle.advertise<anymal_msgs::GaitPatterns>(topic, 1);
  return true;
}

bool GaitPatterns::updateGaitPatternFlightPhases(const loco::GaitPatternFlightPhases* gaitPattern) {
  if (isNotSubscribed()) {
    return true;
  }

  if (gaitPattern == nullptr) {
    return true;
  }

  gaitPatternsMessage_.header.stamp = ros::Time::now();
  gaitPatternsMessage_.phase = gaitPattern->getStridePhase();
  gaitPatternsMessage_.patterns.clear();

  anymal_msgs::GaitPattern pattern;
  pattern.duration = gaitPattern->getStrideDuration();

  for (int i=0; i<gaitPattern->getNumberOfLegs(); ++i) {
    pattern.liftoff_phases[i] = gaitPattern->getFootLiftOffPhase(i);
    pattern.touchdown_phases[i] = gaitPattern->getFootTouchDownPhase(i);
  }

  gaitPatternsMessage_.patterns.push_back(pattern);

  return true;
}

bool GaitPatterns::updateContactSchedule(const loco::contact_schedule::ContactScheduleAnymalBase* contactSchedule, double horizon) {
  if (isNotSubscribed()) {
    return true;
  }

  if (contactSchedule == nullptr || horizon<=0.0) {
    return true;
  }

  gaitPatternsMessage_.header.stamp = ros::Time::now();
  gaitPatternsMessage_.phase = 0.0;
  gaitPatternsMessage_.patterns.clear();

  anymal_msgs::GaitPattern pattern;
  pattern.duration = horizon;

  for (const auto legId : anymal_description::LegEnumIterator()) {
    pattern.liftoff_phases[static_cast<unsigned int>(legId)]   = std::fmin(1.0, contactSchedule->getTimeUntilNextSwing(legId) / horizon);
    pattern.touchdown_phases[static_cast<unsigned int>(legId)] = std::fmin(1.0, contactSchedule->getTimeUntilNextStance(legId) / horizon);
  }

  gaitPatternsMessage_.patterns.push_back(pattern);

  return true;
}

bool GaitPatterns::updateStridePhase(const double phase) {
  gaitPatternsMessage_.phase = phase;
  return true;
}

bool GaitPatterns::publish() {
  loco_ros::publishMsg(publisher_, gaitPatternsMessage_);
  return true;
}

bool GaitPatterns::visualizeGaitPatternFlightPhases() {
  return publish();
}

} /* namespace loco_ros_anymal */
