/*
 * ArmBase.cpp
 *
 *  Created on: Jan 9, 2016
 *      Author: Gabriel Hottiger
 */

// loco
#include "loco/common/arms/ArmBase.hpp"

namespace loco {

ArmBase::ArmBase(const std::string& name, const unsigned int numDofLimb, LimbPropertiesPtr&& properties, EndEffectorBasePtr&& endeffector)
    : LimbBase(name, numDofLimb, std::move(properties), std::move(endeffector)),
      logNameSpace_(std::string{"/loco/"} + name + std::string{"/"}) {}

ArmBase::ArmBase(const std::string& name, const unsigned int numDofLimb, LimbPropertiesPtr&& properties, EndEffectorBasePtr&& endeffector,
                 LimbStateMeasuredPtr&& stateMeasured, LimbStateDesiredPtr&& stateDesired)
    : LimbBase(name, numDofLimb, std::move(properties), std::move(endeffector), std::move(stateMeasured), std::move(stateDesired)) {}

void ArmBase::print(std::ostream& out) const {
  out << "name: " << getName() << std::endl;
  out << "PositionWorldToGripperInWorldFrame: " << getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()
      << std::endl;
}

bool ArmBase::addVariablesToLog(const std::string& ns) const {
  LimbBase::addVariablesToLog(std::string{"/loco/"} + getName());

  // Log task space references.
  signal_logger::add(
      getEndEffector().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Origin).getPositionWorldToEndEffectorInWorldFrame(),
      "positionWorldToDesiredEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(
      getEndEffector().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Origin).getLinearVelocityEndEffectorInWorldFrame(),
      "linearVelocityDesiredEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(
      getEndEffector().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Origin).getLinearAccelerationEndEffectorInWorldFrame(),
      "linearAccelerationDesiredEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(
      getEndEffector().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Contact).getPositionWorldToEndEffectorInWorldFrame(),
      "positionWorldToDesiredEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(
      getEndEffector().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Contact).getLinearVelocityEndEffectorInWorldFrame(),
      "linearVelocityDesiredEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(
      getEndEffector().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Contact).getLinearAccelerationEndEffectorInWorldFrame(),
      "linearAccelerationDesiredEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(getEndEffector().getStateDesired().getForceAtEndEffectorInWorldFrame(), "forceDesiredAtEEInWorldFrame", logNameSpace_);
  signal_logger::add(getEndEffector().getStateDesired().getTorqueAtEndEffectorInWorldFrame(), "desiredTorqueAtEEInWorldFrame",
                     logNameSpace_);

  // Log task space measurements.
  signal_logger::add(
      getEndEffector().getStateMeasured(TimePoint::Now, EndEffectorContactEnum::Origin).getPositionWorldToEndEffectorInWorldFrame(),
      "positionWorldToEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(
      getEndEffector().getStateMeasured(TimePoint::Now, EndEffectorContactEnum::Origin).getLinearVelocityEndEffectorInWorldFrame(),
      "linearVelocityEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(
      getEndEffector().getStateMeasured(TimePoint::Now, EndEffectorContactEnum::Contact).getPositionWorldToEndEffectorInWorldFrame(),
      "positionWorldToEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(
      getEndEffector().getStateMeasured(TimePoint::Now, EndEffectorContactEnum::Contact).getLinearVelocityEndEffectorInWorldFrame(),
      "linearVelocityEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame(), "positionWorldToArmBaseInWorldFrame", logNameSpace_);
  signal_logger::add(getLimbStateMeasured().getLinearVelocityLimbBaseInWorldFrame(), "linearVelocityArmBaseInWorldFrame", logNameSpace_);
  signal_logger::add(getEndEffector().getStateMeasured().getForceAtEndEffectorInWorldFrame(), "forceAtEEInWorldFrame", logNameSpace_);
  signal_logger::add(getEndEffector().getStateMeasured().getTorqueAtEndEffectorInWorldFrame(), "torqueAtEEInWorldFrame", logNameSpace_);

  // Add event flags to log.
  limbStrategy_->addVariablesToLog(logNameSpace_);

  return true;
}

} /* namespace loco */
