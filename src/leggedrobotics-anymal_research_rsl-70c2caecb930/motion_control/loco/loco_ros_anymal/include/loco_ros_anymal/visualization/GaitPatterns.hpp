/*
 * GaitPatterns.hpp
 *
 *  Created on: Dec 7, 2015
 *      Author: Christian Gehring, Dario Bellicoso
 */
#pragma once

// loco
#include <loco/gait_pattern/GaitPatternFlightPhases.hpp>
#include "loco/gait_pattern/contact_schedules.hpp"

// ros
#include <ros/ros.h>
#include <anymal_msgs/GaitPatterns.h>

// loco ros
#include <loco_ros/visualization/ModuleRos.hpp>

namespace loco_ros_anymal {

class GaitPatterns : public ModuleRos {
 public:
  GaitPatterns();
  virtual ~GaitPatterns() = default;

  bool initialize(ros::NodeHandle& nodeHandle, const std::string& topic);

  // Display a periodic gait.
  bool updateGaitPatternFlightPhases(const loco::GaitPatternFlightPhases* gaitPattern);

  // Display any gait within a predefined prediction horizon horizon.
  bool updateContactSchedule(const loco::contact_schedule::ContactScheduleAnymalBase* contactSchedule, double horizon = 2.0);
  bool publish() override;

  bool visualizeGaitPatternFlightPhases();

  bool updateStridePhase(const double phase);

 protected:
   anymal_msgs::GaitPatterns gaitPatternsMessage_;
   ros::Publisher publisher_;
};

} /* namespace loco_ros_anymal */
