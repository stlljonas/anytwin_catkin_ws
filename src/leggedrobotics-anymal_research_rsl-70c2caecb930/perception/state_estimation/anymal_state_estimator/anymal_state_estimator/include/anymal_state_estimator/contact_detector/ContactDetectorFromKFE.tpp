/*
 * ContactDetectorFromKFE.cpp
 *
 *  Created on: Nov 28, 2017
 *      Author: markusta
 */

#include <anymal_state_estimator/contact_detector/ContactDetectorFromKFE.hpp>

namespace anymal_state_estimator {

template <typename ConcreteDescription_, typename RobotState_>
ContactDetectorFromKFE<ConcreteDescription_, RobotState_>::ContactDetectorFromKFE(const std::string& name,
                                                                                  const RobotModel& robotModel,
                                                                                  const double lowerThreshold,
                                                                                  const double upperThreshold,
                                                                                  const unsigned int legId)
    : ContactDetectorBase(name),
      robotModel_(robotModel),
      lowerThreshold_(lowerThreshold),
      upperThreshold_(upperThreshold),
      legId_(legId),
      kfeJointUInt_(RD::mapKeyEnumToKeyId(RD::JointNodeEnum::KFE)) {}

template <typename ConcreteDescription_, typename RobotState_>
bool ContactDetectorFromKFE<ConcreteDescription_, RobotState_>::initialize(double dt) {
  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
bool ContactDetectorFromKFE<ConcreteDescription_, RobotState_>::advance(double dt) {
  detectContact();
  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
void ContactDetectorFromKFE<ConcreteDescription_, RobotState_>::detectContact() {
  double kfeTorque = computeKfeTorque();

  if (kfeTorque > upperThreshold_ || kfeTorque < lowerThreshold_) {
    state_ = ContactState::CLOSED;

  } else {
    state_ = ContactState::OPEN;
  }
}

template <typename ConcreteDescription_, typename RobotState_>
double ContactDetectorFromKFE<ConcreteDescription_, RobotState_>::computeKfeTorque() {
  return robotModel_.getState().getJointTorques().vector()[kfeJointUInt_ + legId_ * RD::getNumDofLimb()];
}

}  // namespace anymal_state_estimator
