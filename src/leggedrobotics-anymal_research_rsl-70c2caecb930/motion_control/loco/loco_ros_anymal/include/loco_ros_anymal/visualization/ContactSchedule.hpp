/*
 * ContactSchedule.hpp
 *
 *  Created on: Jan 17, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/gait_pattern/GaitPatternBase.hpp>
#include <loco/common/legs/Legs.hpp>

// loco ros
#include <loco_ros/loco_ros.hpp>

// ros
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

namespace loco_ros_anymal {

class ContactSchedule {
 public:
  ContactSchedule() {
    mapIdToLegName_.insert(std::pair<int, std::string>(0, std::string{"lf"}));
    mapIdToLegName_.insert(std::pair<int, std::string>(1, std::string{"rf"}));
    mapIdToLegName_.insert(std::pair<int, std::string>(2, std::string{"lh"}));
    mapIdToLegName_.insert(std::pair<int, std::string>(3, std::string{"rh"}));

    shouldBeGrounded_.resize(4);
    swingPhase_.resize(4);
    stancePhase_.resize(4);
    swingDuration_.resize(4);
    stanceDuration_.resize(4);
    limbStrategy_.resize(4);
  };
  virtual ~ContactSchedule() = default;

  bool initialize(ros::NodeHandle& nodeHandle) {
    for (unsigned int k=0; k<4; ++k) {
      shouldBeGrounded_[k].first = nodeHandle_.advertise<decltype(shouldBeGrounded_[k].second)>("/loco_ros/contact_schedule/should_be_grounded_"+mapIdToLegName_[k], 1);
      swingPhase_[k].first = nodeHandle_.advertise<decltype(swingPhase_[k].second)>("/loco_ros/contact_schedule/swing_phase_"+mapIdToLegName_[k], 1);
      stancePhase_[k].first = nodeHandle_.advertise<decltype(stancePhase_[k].second)>("/loco_ros/contact_schedule/stance_phase_"+mapIdToLegName_[k], 1);
      swingDuration_[k].first = nodeHandle_.advertise<decltype(swingDuration_[k].second)>("/loco_ros/contact_schedule/swing_duration_"+mapIdToLegName_[k], 1);
      stanceDuration_[k].first = nodeHandle_.advertise<decltype(stanceDuration_[k].second)>("/loco_ros/contact_schedule/stance_duration_"+mapIdToLegName_[k], 1);
      limbStrategy_[k].first = nodeHandle_.advertise<decltype(limbStrategy_[k].second)>("/loco_ros/contact_schedule/limb_strategy_"+mapIdToLegName_[k], 1);
    }

    return true;
  }

  bool shutdown() {
    for (unsigned int k=0; k<4; ++k) {
      shouldBeGrounded_[k].first.shutdown();
      swingPhase_[k].first.shutdown();
      stancePhase_[k].first.shutdown();
      swingDuration_[k].first.shutdown();
      stanceDuration_[k].first.shutdown();
      limbStrategy_[k].first.shutdown();
    }
    return true;
  }

  bool update(const loco::GaitPatternBase& gaitPattern, const loco::Legs& legs) {
    for (unsigned int k=0; k<4; ++k) {
      shouldBeGrounded_[k].second.data = gaitPattern.shouldBeLegGrounded(k);
      swingPhase_[k].second.data = gaitPattern.getSwingPhaseForLeg(k);
      stancePhase_[k].second.data = gaitPattern.getStancePhaseForLeg(k);
      swingDuration_[k].second.data = gaitPattern.getSwingDuration(k, gaitPattern.getStrideDuration());
      stanceDuration_[k].second.data = gaitPattern.getStanceDuration(k, gaitPattern.getStrideDuration());
      limbStrategy_[k].second.data = static_cast<unsigned int>(legs.get(k).getLimbStrategy().getLimbStrategyEnum());
    }

    return true;
  }

  bool publish() {
    for (unsigned int k=0; k<4; ++k) {
      loco_ros::publishMsg(shouldBeGrounded_[k]);
      loco_ros::publishMsg(swingPhase_[k]);
      loco_ros::publishMsg(stancePhase_[k]);
      loco_ros::publishMsg(swingDuration_[k]);
      loco_ros::publishMsg(stanceDuration_[k]);
      loco_ros::publishMsg(limbStrategy_[k]);
    }
    return true;
  }

protected:
  ros::NodeHandle nodeHandle_;

  std::vector<std::pair<ros::Publisher, std_msgs::Bool>> shouldBeGrounded_;
  std::vector<std::pair<ros::Publisher, std_msgs::Float64>> swingPhase_;
  std::vector<std::pair<ros::Publisher, std_msgs::Float64>> stancePhase_;
  std::vector<std::pair<ros::Publisher, std_msgs::Float64>> swingDuration_;
  std::vector<std::pair<ros::Publisher, std_msgs::Float64>> stanceDuration_;
  std::vector<std::pair<ros::Publisher, std_msgs::Int32>> limbStrategy_;

  std::map<int, std::string> mapIdToLegName_;

};

} /* namespace loco_ros */

