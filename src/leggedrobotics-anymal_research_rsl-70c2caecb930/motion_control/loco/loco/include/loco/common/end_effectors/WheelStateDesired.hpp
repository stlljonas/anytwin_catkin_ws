/*
 * WheelStateDesired.hpp
 *
 *  Created on: Jan 12, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include <loco/common/end_effectors/FootBaseStateDesired.hpp>
#include <loco/common/joints/DesiredJointStates.hpp>
#include <loco/common/typedefs.hpp>

namespace loco {

class WheelStateDesired : public FootBaseStateDesired {
 public:
  explicit WheelStateDesired(const unsigned int indexInLimbJoints) : FootBaseStateDesired(), indexInLimbJoints_(indexInLimbJoints) {}

  ~WheelStateDesired() override = default;

  void setWheelJointPosition(JointPosition jointPosition) { desiredJointStatesLimb_->setJointPosition(indexInLimbJoints_, jointPosition); }

  const JointPosition& getWheelJointPosition() const { return desiredJointStatesLimb_->getJointPositions()(indexInLimbJoints_); }

  void setWheelJointVelocity(JointVelocity jointVelocity) { desiredJointStatesLimb_->setJointVelocity(indexInLimbJoints_, jointVelocity); }

  const JointVelocity& getWheelJointVelocity() const { return desiredJointStatesLimb_->getJointVelocities()(indexInLimbJoints_); }

  void setWheelJointTorque(JointTorque jointTorque) { desiredJointStatesLimb_->setJointTorque(indexInLimbJoints_, jointTorque); }

  const JointTorque& getWheelJointTorque() const { return desiredJointStatesLimb_->getJointTorques()(indexInLimbJoints_); }

  void setWheelJointControlMode(JointControlMode jointControlMode) {
    desiredJointStatesLimb_->setJointControlMode(indexInLimbJoints_, jointControlMode);
  }

  const JointControlMode& getWheelJointControlMode() const { return desiredJointStatesLimb_->getJointControlModes()(indexInLimbJoints_); }

 protected:
  const unsigned int indexInLimbJoints_;
};

} /* namespace loco */
