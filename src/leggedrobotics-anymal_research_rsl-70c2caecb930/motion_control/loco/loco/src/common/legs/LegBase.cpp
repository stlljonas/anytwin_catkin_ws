/*
 * LegBase.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include "loco/common/legs/LegBase.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

namespace loco {

LegBase::LegBase(const std::string& name, const unsigned int numDofLimb, LegPropertiesPtr&& properties, FootBasePtr&& foot)
    : LimbBase(name, numDofLimb, std::move(properties), std::move(foot)),
      stateSwitcher_(new loco::StateSwitcher()),
      contactSchedule_(new loco::ContactSchedule()),
      stateTouchDown_(),
      stateLiftOff_(),
      logNameSpace_(std::string{"/loco/"} + name + std::string{"/"}),
      didTouchDownAtLeastOnceDuringStance_(false),
      didLiftOffAtLeastOnceDuringSwing_(false),
      didSetLostContactPositionForPhase_(false),
      positionWorldToLostContactPositionInWorldFrame_(),
      positionWorldToLastOrCurrentContactInWorldFrame_() {}

LegStateTouchDown* LegBase::getStateTouchDown() {
  return &stateTouchDown_;
}

const LegStateTouchDown& LegBase::getStateTouchDown() const {
  return stateTouchDown_;
}

LegStateLiftOff* LegBase::getStateLiftOff() {
  return &stateLiftOff_;
}

const LegStateLiftOff& LegBase::getStateLiftOff() const {
  return stateLiftOff_;
}

LegProperties* LegBase::getLegPropertiesPtr() {
  return dynamic_cast<LegProperties*>(limbProperties_.get());
}

const LegProperties& LegBase::getLegProperties() const {
  return dynamic_cast<const LegProperties&>(*limbProperties_);
}

FootBase* LegBase::getFootPtr() {
  return dynamic_cast<FootBase*>(endEffector_.get());
}

const FootBase& LegBase::getFoot() const {
  return dynamic_cast<const FootBase&>(*endEffector_);
}

ContactSchedule* LegBase::getContactSchedulePtr() {
  return contactSchedule_.get();
}

const ContactSchedule& LegBase::getContactSchedule() const {
  return *contactSchedule_;
}

StateSwitcher* LegBase::getStateSwitcherPtr() {
  return stateSwitcher_.get();
}

const StateSwitcher& LegBase::getStateSwitcher() const {
  return *stateSwitcher_;
}

void LegBase::setDesiredJointControlModeToLeg(loco::ControlMode controlMode) {
  JointControlModes controlModes = getInitializedJointControlModes();
  controlModes.setConstant(controlMode);
  limbStateDesired_->setJointControlModes(controlModes);
}

bool LegBase::isAnyJointInControlMode(loco::ControlMode controlMode) const {
  return (limbStateDesired_->getJointControlModes().array() == controlMode).any();
}

bool LegBase::didTouchDownAtLeastOnceDuringStance() const {
  return didTouchDownAtLeastOnceDuringStance_;
}

void LegBase::setDidTouchDownAtLeastOnceDuringStance(bool didTouchDownAtLeastOnceDuringStance) {
  didTouchDownAtLeastOnceDuringStance_ = didTouchDownAtLeastOnceDuringStance;
}

bool LegBase::didLiftOffAtLeastOnceDuringSwing() const {
  return didLiftOffAtLeastOnceDuringSwing_;
}

void LegBase::setDidLiftOffAtLeastOnceDuringSwing(bool didLiftOffAtLeastOnceDuringSwing) {
  didLiftOffAtLeastOnceDuringSwing_ = didLiftOffAtLeastOnceDuringSwing;
}

void LegBase::setPositionWorldToLostContactPositionInWorldFrame(const Position& positionWorldToLostContactPositionInWorldFrame) {
  positionWorldToLostContactPositionInWorldFrame_ = positionWorldToLostContactPositionInWorldFrame;
}

const Position& LegBase::getPositionWorldToLostContactPositionInWorldFrame() const {
  return positionWorldToLostContactPositionInWorldFrame_;
}

void LegBase::setPositionWorldToLastOrCurrentContactInWorldFrame(const Position& positionWorldToLastOrCurrentContactInWorldFrame) {
  positionWorldToLastOrCurrentContactInWorldFrame_ = positionWorldToLastOrCurrentContactInWorldFrame;
}

const Position& LegBase::getPositionWorldToLastOrCurrentContactInWorldFrame() const {
  return positionWorldToLastOrCurrentContactInWorldFrame_;
}

void LegBase::setDidSetLostContactPositionForPhase(bool didSetLostContactPositionForPhase) {
  didSetLostContactPositionForPhase_ = didSetLostContactPositionForPhase;
}

bool LegBase::didSetLostContactPositionForPhase() const {
  return didSetLostContactPositionForPhase_;
}

void LegBase::print(std::ostream& out) const {
  out << "name: " << getName() << std::endl;
  out << "swing phase: " << getContactSchedule().getSwingPhase() << std::endl;
  out << "stance phase: " << getContactSchedule().getStancePhase() << std::endl;
  out << "swing duration: " << getContactSchedule().getSwingDuration() << std::endl;
  out << "stance duration: " << getContactSchedule().getStanceDuration() << std::endl;

  out << "is grounded: " << (getContactSchedule().isGrounded() ? "yes" : "no") << std::endl;
  out << "should be grounded: " << (getContactSchedule().shouldBeGrounded() ? "yes" : "no") << std::endl;
  out << "was grounded: " << (getContactSchedule().wasGrounded() ? "yes" : "no") << std::endl;
  out << "is slipping: " << (getContactSchedule().isSlipping() ? "yes" : "no") << std::endl;

  out << "did touchdown: " << (getStateTouchDown().isNow() ? "yes" : "no") << std::endl;
  out << "       early?: " << (getStateTouchDown().lastStateWasEarly() ? "yes" : "no") << std::endl;
  out << "        late?: " << (getStateTouchDown().lastStateWasLate() ? "yes" : "no") << std::endl;
  out << "most recent occurred at time: " << getStateTouchDown().stateChangedAtTime() << std::endl;

  out << "did liftoff: " << (getStateLiftOff().isNow() ? "yes" : "no") << std::endl;
  out << "most recent occurred at time: " << getStateLiftOff().stateChangedAtTime() << std::endl;

  out << "did touchdown: " << (getStateTouchDown().isNow() ? "yes" : "no") << std::endl;

  out << "PositionWorldToFootInWorldFrame: " << getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() << std::endl;
}

bool LegBase::addVariablesToLog(const std::string& ns) const {
  LimbBase::addVariablesToLog(std::string{"/loco/"} + getName());

  // Log leg state flags.
  signal_logger::add(didTouchDownAtLeastOnceDuringStance_, "didTouchDownAtLeastOnceDuringStance", logNameSpace_);

  // Log task space references.
  signal_logger::add(getFoot().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Origin).getPositionWorldToEndEffectorInWorldFrame(),
                     "positionWorldToDesiredEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(getFoot().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Origin).getLinearVelocityEndEffectorInWorldFrame(),
                     "linearVelocityDesiredEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(
      getFoot().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Origin).getLinearAccelerationEndEffectorInWorldFrame(),
      "linearAccelerationDesiredEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(getFoot().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Contact).getPositionWorldToEndEffectorInWorldFrame(),
                     "positionWorldToDesiredEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(getFoot().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Contact).getLinearVelocityEndEffectorInWorldFrame(),
                     "linearVelocityDesiredEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(
      getFoot().getStateDesired(TimePoint::Now, EndEffectorContactEnum::Contact).getLinearAccelerationEndEffectorInWorldFrame(),
      "linearAccelerationDesiredEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(getFoot().getStateDesired().getForceAtEndEffectorInWorldFrame(), "forceDesiredAtEEInWorldFrame", logNameSpace_);

  // Log task space measurements.
  signal_logger::add(getFoot().getStateMeasured(TimePoint::Now, EndEffectorContactEnum::Origin).getPositionWorldToEndEffectorInWorldFrame(),
                     "positionWorldToEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(getFoot().getStateMeasured(TimePoint::Now, EndEffectorContactEnum::Origin).getLinearVelocityEndEffectorInWorldFrame(),
                     "linearVelocityEEOriginInWorldFrame", logNameSpace_);
  signal_logger::add(
      getFoot().getStateMeasured(TimePoint::Now, EndEffectorContactEnum::Contact).getPositionWorldToEndEffectorInWorldFrame(),
      "positionWorldToEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(getFoot().getStateMeasured(TimePoint::Now, EndEffectorContactEnum::Contact).getLinearVelocityEndEffectorInWorldFrame(),
                     "linearVelocityEEContactInWorldFrame", logNameSpace_);
  signal_logger::add(getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame(), "positionWorldToHipInWorldFrame", logNameSpace_);
  signal_logger::add(getLimbStateMeasured().getLinearVelocityLimbBaseInWorldFrame(), "linearVelocityHipInWorldFrame", logNameSpace_);
  signal_logger::add(getFoot().getStateMeasured().getForceAtEndEffectorInWorldFrame(), "forceAtEEInWorldFrame", logNameSpace_);
  signal_logger::add(getPositionWorldToLostContactPositionInWorldFrame(), "posWorldToLostContactInWorldFrame", logNameSpace_);

  // Log foothold reference.
  signal_logger::add(getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame(), "positionWorldToDesiredFootholdInWorldFrame",
                     logNameSpace_);

  // Add event flags to log.
  stateSwitcher_->addVariablesToLog(logNameSpace_);
  limbStrategy_->addVariablesToLog(logNameSpace_);
  contactSchedule_->addVariablesToLog(logNameSpace_);

  return true;
}

} /* namespace loco */
