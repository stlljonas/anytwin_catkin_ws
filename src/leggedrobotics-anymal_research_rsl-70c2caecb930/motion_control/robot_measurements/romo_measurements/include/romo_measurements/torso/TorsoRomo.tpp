/*!
 * @file	  TorsoRomo.tpp
 * @author	Gabriel Hottiger
 * @date	  Nov, 2017
 */

// romo_measurements
#include "romo_measurements/torso/TorsoRomo.hpp"

// message_logger
#include "message_logger/message_logger.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_>
TorsoRomo<ConcreteDescription_,RobotState_>::TorsoRomo(const std::string& name, const RobotModel& model)
  : TorsoRomo(name, model, loco::TorsoPropertiesPtr(new TorsoPropertiesRomo<ConcreteDescription_, RobotState_>(model)))
{
}

template <typename ConcreteDescription_, typename RobotState_>
TorsoRomo<ConcreteDescription_,RobotState_>::TorsoRomo(const std::string& name,
                                                       const RobotModel& model,
                                                       loco::TorsoPropertiesPtr&& properties)
  : loco::TorsoBase(name, std::move(properties)),
    model_(model)
{
}

template <typename ConcreteDescription_, typename RobotState_>
bool TorsoRomo<ConcreteDescription_,RobotState_>::initialize(double dt) {
  // Init properties
  if(!this->getPropertiesPtr()->initialize(dt)) {
    MELO_WARN_STREAM("[TorsoRomo]: Torso could not initialize properties!");
    return false;
  }

  // Advance once
  if (!this->advance(dt)) {
    MELO_WARN_STREAM("[TorsoRomo]: Torso could not advance during initialization!");
    return false;
  }

  // Initialize motion references.
//  this->getDesiredStatePtr()->setTargetPoint(loco::TorsoStateDesired::TargetPoint::BASE);

  this->getDesiredStatePtr()->setPositionControlToTargetInControlFrame(
    this->getMeasuredState().inControlFrame().getPositionControlToBaseInControlFrame());
  this->getDesiredStatePtr()->setPositionWorldToBaseInWorldFrame(
    this->getMeasuredState().getPositionWorldToBaseInWorldFrame());

  this->getDesiredStatePtr()->setOrientationControlToBase(
    this->getMeasuredState().inControlFrame().getOrientationControlToBase());
  this->getDesiredStatePtr()->setOrientationEulerAnglesZyxBaseToWorld(
    this->getMeasuredState().getOrientationEulerAnglesZyxBaseToWorld());

  this->getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(loco::LinearVelocity());
  this->getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(loco::LocalAngularVelocity());

  this->getDesiredStatePtr()->setLinearAccelerationTargetInControlFrame(loco::LinearAcceleration());
  this->getDesiredStatePtr()->setAngularAccelerationTargetInControlFrame(loco::AngularAcceleration());

  this->getDesiredStatePtr()->setPositionErrorInControlFrame(loco::Position());
  this->getDesiredStatePtr()->setLinearVelocityErrorInControlFrame(loco::LinearVelocity());
  this->getDesiredStatePtr()->setAngularVelocityErrorInControlFrame(loco::LocalAngularVelocity());

  return true;
}

template <typename ConcreteDescription_, typename RobotState_>
bool TorsoRomo<ConcreteDescription_,RobotState_>::advance(double dt) {
  // Properties
  if(!this->getPropertiesPtr()->advance(dt)) {
    MELO_WARN_STREAM("[TorsoRomo]: Torso could not advance properties!");
    return false;
  }

  // Pose
  const loco::RotationQuaternion& q_WB(model_.getState().getOrientationBaseToWorld());
  this->getMeasuredStatePtr()->setPositionWorldToBaseInWorldFrame(
    model_.getState().getPositionWorldToBaseInWorldFrame());
  this->getMeasuredStatePtr()->setPositionWorldToCenterOfMassInWorldFrame(
      loco::Position(model_.getPositionWorldToBodyCom(BodyEnum::BASE, CoordinateFrameEnum::WORLD)));
  this->getMeasuredStatePtr()->setOrientationWorldToBase(q_WB.inverted());

  // Twist
  this->getMeasuredStatePtr()->setLinearVelocityBaseInBaseFrame(q_WB.inverseRotate(
    model_.getState().getLinearVelocityBaseInWorldFrame()));
  this->getMeasuredStatePtr()->setAngularVelocityBaseInBaseFrame(
    model_.getState().getAngularVelocityBaseInBaseFrame());
  this->getMeasuredStatePtr()->setLinearVelocityCenterOfMassInBaseFrame(
      loco::LinearVelocity(model_.getLinearVelocityWorldToBodyCom(
          BodyEnum::BASE, CoordinateFrameEnum::BASE)));

  return true;
}

} /* namespace romo_measurements */
