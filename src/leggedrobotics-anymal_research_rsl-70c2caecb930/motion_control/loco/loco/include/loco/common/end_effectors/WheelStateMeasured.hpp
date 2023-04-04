/*
 * WheelStateMeasured.hpp
 *
 *  Created on: Jan 12, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include <loco/common/end_effectors/FootBaseStateMeasured.hpp>
#include <loco/common/joints/MeasuredJointStates.hpp>
#include <loco/common/typedefs.hpp>

namespace loco {

class WheelStateMeasured : public FootBaseStateMeasured {
 public:
  explicit WheelStateMeasured(const unsigned int indexInLimbJoints) : FootBaseStateMeasured(), indexInLimbJoints_(indexInLimbJoints) {}

  ~WheelStateMeasured() override = default;

  void setWheelJointPosition(JointPosition jointPosition) { measuredJointStatesLimb_->setJointPosition(indexInLimbJoints_, jointPosition); }

  const JointPosition& getWheelJointPosition() const { return measuredJointStatesLimb_->getJointPositions()(indexInLimbJoints_); }

  void setWheelJointVelocity(JointVelocity jointVelocity) { measuredJointStatesLimb_->setJointVelocity(indexInLimbJoints_, jointVelocity); }

  const JointVelocity& getWheelJointVelocity() const { return measuredJointStatesLimb_->getJointVelocities()(indexInLimbJoints_); }

  void setWheelJointTorque(JointTorque jointTorque) { measuredJointStatesLimb_->setJointTorque(indexInLimbJoints_, jointTorque); }

  const JointTorque& getWheelJointTorque() const { return measuredJointStatesLimb_->getJointTorques()(indexInLimbJoints_); }

  void setWheelJointControlMode(JointControlMode jointControlMode) {
    measuredJointStatesLimb_->setJointControlMode(indexInLimbJoints_, jointControlMode);
  }

  const JointControlMode& getWheelJointControlMode() const { return measuredJointStatesLimb_->getJointControlModes()(indexInLimbJoints_); }

 protected:
  const unsigned int indexInLimbJoints_;
};

} /* namespace loco */
