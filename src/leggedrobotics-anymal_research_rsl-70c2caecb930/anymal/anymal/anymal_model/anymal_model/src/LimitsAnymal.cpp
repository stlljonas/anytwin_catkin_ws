/*
 * LimitsAnymal.cpp
 *
 *  Created on: Nov 7, 2018
 *      Author: Yvain de Viragh
 */

// model
#include <anymal_model/LimitsAnymal.hpp>

// signal_logger
#include "signal_logger/signal_logger.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// stl
#include <string>

namespace anymal_model {

bool LimitsAnymal::update(const JointPositions& /*jointPositions*/, const JointVelocities& /*jointVelocities*/) {
  MELO_WARN_STREAM("[LimitsAnymal::update]: Functionality not supported. Returning false!");

  return false;
}

void LimitsAnymal::init() {}

void LimitsAnymal::printLimits() const {
  std::stringstream message;

  message << "[LimitsAnymal::printLimits] Joint and actuator limits:";

  for (const auto& jointKey : AD::getJointKeys()) {
    const auto& limit = jointLimits_[jointKey.getEnum()];
    message << "\nJoint: " + std::string(jointKey.getName()) + "\n* minPosition: " + std::to_string(limit.minPosition_) +
                   "\n* maxPosition: " + std::to_string(limit.maxPosition_) + "\n* minVelocity: " + std::to_string(limit.minVelocity_) +
                   "\n* maxVelocity: " + std::to_string(limit.maxVelocity_) + "\n* minEffort: " + std::to_string(limit.minEffort_) +
                   "\n* maxEffort: " + std::to_string(limit.maxEffort_) +
                   "\n* minCommandEffort: " + std::to_string(limit.minCommandEffort_) +
                   "\n* maxCommandEffort: " + std::to_string(limit.maxCommandEffort_);
  }

  for (const auto& actuatorKey : AD::getActuatorKeys()) {
    const auto& limit = actuatorLimits_[actuatorKey.getEnum()];
    message << "\nActuator: " + std::string(actuatorKey.getName()) + "\n* minPosition: " + std::to_string(limit.minPosition_) +
                   "\n* maxPosition: " + std::to_string(limit.maxPosition_) + "\n* minVelocity: " + std::to_string(limit.minVelocity_) +
                   "\n* maxVelocity: " + std::to_string(limit.maxVelocity_) + "\n* minEffort: " + std::to_string(limit.minEffort_) +
                   "\n* maxEffort: " + std::to_string(limit.maxEffort_) +
                   "\n* minCommandEffort: " + std::to_string(limit.minCommandEffort_) +
                   "\n* maxCommandEffort: " + std::to_string(limit.maxCommandEffort_) + "\n* minGearVelocity: "
            << std::to_string(limit.minGearVelocity_) + "\n* maxGearVelocity: "
            << std::to_string(limit.maxGearVelocity_) + "\n* minCurrent: " << std::to_string(limit.minCurrent_) + "\n* maxCurrent: "
            << std::to_string(limit.maxCurrent_);
  }

  MELO_INFO_STREAM(message.str());
}

const LimitsAnymal::ActuatorLimitEntry& LimitsAnymal::getActuatorLimitEntry(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator];
}

void LimitsAnymal::setActuatorLimitEntry(ActuatorEnum actuator, ActuatorLimitEntry entry) {
  actuatorLimits_[actuator] = entry;
}

const LimitsAnymal::JointLimitEntry& LimitsAnymal::getJointLimitEntry(JointEnum joint) const {
  return jointLimits_[joint];
}

void LimitsAnymal::setJointLimitEntry(JointEnum joint, JointLimitEntry entry) {
  jointLimits_[joint] = entry;
}

double LimitsAnymal::getActuatorMinPosition(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].minPosition_;
}

double LimitsAnymal::getActuatorMaxPosition(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].maxPosition_;
}

double LimitsAnymal::getActuatorMinVelocity(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].minVelocity_;
}

double LimitsAnymal::getActuatorMaxVelocity(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].maxVelocity_;
}

double LimitsAnymal::getActuatorMinEffort(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].minEffort_;
}

double LimitsAnymal::getActuatorMaxEffort(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].maxEffort_;
}

double LimitsAnymal::getActuatorMinCommandEffort(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].minCommandEffort_;
}

double LimitsAnymal::getActuatorMaxCommandEffort(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].maxCommandEffort_;
}

double LimitsAnymal::getActuatorMinGearVelocity(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].minGearVelocity_;
}

double LimitsAnymal::getActuatorMaxGearVelocity(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].maxGearVelocity_;
}

double LimitsAnymal::getActuatorMinCurrent(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].minCurrent_;
}

double LimitsAnymal::getActuatorMaxCurrent(ActuatorEnum actuator) const {
  return actuatorLimits_[actuator].maxCurrent_;
}

void LimitsAnymal::setActuatorMinPosition(ActuatorEnum actuator, double minPosition) {
  actuatorLimits_[actuator].minPosition_ = minPosition;
}

void LimitsAnymal::setActuatorMaxPosition(ActuatorEnum actuator, double maxPosition) {
  actuatorLimits_[actuator].maxPosition_ = maxPosition;
}

void LimitsAnymal::setActuatorMinVelocity(ActuatorEnum actuator, double minVelocity) {
  actuatorLimits_[actuator].minVelocity_ = minVelocity;
}

void LimitsAnymal::setActuatorMaxVelocity(ActuatorEnum actuator, double maxVelocity) {
  actuatorLimits_[actuator].maxVelocity_ = maxVelocity;
}

void LimitsAnymal::setActuatorMinEffort(ActuatorEnum actuator, double minEffort) {
  actuatorLimits_[actuator].minEffort_ = minEffort;
}

void LimitsAnymal::setActuatorMaxEffort(ActuatorEnum actuator, double maxEffort) {
  actuatorLimits_[actuator].maxEffort_ = maxEffort;
}

void LimitsAnymal::setActuatorMinCommandEffort(ActuatorEnum actuator, double minCommandEffort) {
  actuatorLimits_[actuator].minCommandEffort_ = minCommandEffort;
}

void LimitsAnymal::setActuatorMaxCommandEffort(ActuatorEnum actuator, double maxCommandEffort) {
  actuatorLimits_[actuator].maxCommandEffort_ = maxCommandEffort;
}

void LimitsAnymal::setActuatorMinGearVelocity(ActuatorEnum actuator, double minGearVelocity) {
  actuatorLimits_[actuator].minGearVelocity_ = minGearVelocity;
}

void LimitsAnymal::setActuatorMaxGearVelocity(ActuatorEnum actuator, double maxGearVelocity) {
  actuatorLimits_[actuator].maxGearVelocity_ = maxGearVelocity;
}

void LimitsAnymal::setActuatorMinCurrent(ActuatorEnum actuator, double minCurrent) {
  actuatorLimits_[actuator].minCurrent_ = minCurrent;
}

void LimitsAnymal::setActuatorMaxCurrent(ActuatorEnum actuator, double maxCurrent) {
  actuatorLimits_[actuator].maxCurrent_ = maxCurrent;
}

double LimitsAnymal::getJointMinPosition(JointEnum joint) const {
  return jointLimits_[joint].minPosition_;
}

double LimitsAnymal::getJointMaxPosition(JointEnum joint) const {
  return jointLimits_[joint].maxPosition_;
}

double LimitsAnymal::getJointMinVelocity(JointEnum joint) const {
  return jointLimits_[joint].minVelocity_;
}

double LimitsAnymal::getJointMaxVelocity(JointEnum joint) const {
  return jointLimits_[joint].maxVelocity_;
}

double LimitsAnymal::getJointMinEffort(JointEnum joint) const {
  return jointLimits_[joint].minEffort_;
}

double LimitsAnymal::getJointMaxEffort(JointEnum joint) const {
  return jointLimits_[joint].maxEffort_;
}

double LimitsAnymal::getJointMinCommandEffort(JointEnum joint) const {
  return jointLimits_[joint].minCommandEffort_;
}

double LimitsAnymal::getJointMaxCommandEffort(JointEnum joint) const {
  return jointLimits_[joint].maxCommandEffort_;
}

void LimitsAnymal::setJointMinPosition(JointEnum joint, double minPosition) {
  jointLimits_[joint].minPosition_ = minPosition;
}

void LimitsAnymal::setJointMaxPosition(JointEnum joint, double maxPosition) {
  jointLimits_[joint].maxPosition_ = maxPosition;
}

void LimitsAnymal::setJointMinVelocity(JointEnum joint, double minVelocity) {
  jointLimits_[joint].minVelocity_ = minVelocity;
}

void LimitsAnymal::setJointMaxVelocity(JointEnum joint, double maxVelocity) {
  jointLimits_[joint].maxVelocity_ = maxVelocity;
}

void LimitsAnymal::setJointMinEffort(JointEnum joint, double minEffort) {
  jointLimits_[joint].minEffort_ = minEffort;
}

void LimitsAnymal::setJointMaxEffort(JointEnum joint, double maxEffort) {
  jointLimits_[joint].maxEffort_ = maxEffort;
}

void LimitsAnymal::setJointMinCommandEffort(JointEnum joint, double minCommandEffort) {
  jointLimits_[joint].minCommandEffort_ = minCommandEffort;
}

void LimitsAnymal::setJointMaxCommandEffort(JointEnum joint, double maxCommandEffort) {
  jointLimits_[joint].maxCommandEffort_ = maxCommandEffort;
}

}  // namespace anymal_model
