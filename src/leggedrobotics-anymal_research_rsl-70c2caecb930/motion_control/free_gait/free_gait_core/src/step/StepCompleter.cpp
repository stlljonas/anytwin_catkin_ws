/*
 * StepCompleter.hpp
 *
 *  Created on: Mar 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// message logger
#include <message_logger/message_logger.hpp>

#include "free_gait_core/step/StepCompleter.hpp"
#include "free_gait_core/leg_motion/leg_motion.hpp"

namespace free_gait {

StepCompleter::StepCompleter(const StepParameters& parameters, const AdapterBase& adapter)
    : parameters_(parameters),
      adapter_(adapter)
{
}

StepCompleter::~StepCompleter()
{
}

bool StepCompleter::complete(const State& state, const StepQueue& queue, Step& step)
{
  for (auto& legMotion : step.legMotions_) {
    setParameters(*legMotion.second);
    legMotion.second->hasContactAtStart_ = adapter_.isLegGrounded(legMotion.first);
    switch (legMotion.second->getType()) {
      case LegMotionBase::Type::Footstep:
        setParameters(dynamic_cast<Footstep&>(*legMotion.second));
        break;
      case LegMotionBase::Type::EndEffectorTarget:
        setParameters(dynamic_cast<EndEffectorTarget&>(*legMotion.second));
        break;
      case LegMotionBase::Type::LegMode:
        setParameters(dynamic_cast<LegMode&>(*legMotion.second));
        break;
      default:
        break;
    }
    switch (legMotion.second->getTrajectoryType()) {
      case LegMotionBase::TrajectoryType::EndEffector:
        if (!complete(state, step, dynamic_cast<EndEffectorMotionBase&>(*legMotion.second))) return false;
        break;
      case LegMotionBase::TrajectoryType::Joints:
        if (!complete(state, step, dynamic_cast<JointMotionBase&>(*legMotion.second))) return false;
        break;
      default:
        throw std::runtime_error("StepCompleter::complete() could not complete leg motion of this type.");
        break;
    }
  }

  if (step.baseMotion_) {
    switch (step.baseMotion_->getType()) {
      case BaseMotionBase::Type::Auto:
        setParameters(dynamic_cast<BaseAuto&>(*step.baseMotion_));
        break;
      case BaseMotionBase::Type::Target:
        setParameters(dynamic_cast<BaseTarget&>(*step.baseMotion_));
        break;
      case BaseMotionBase::Type::Trajectory:
        setParameters(dynamic_cast<BaseTrajectory&>(*step.baseMotion_));
        break;
      default:
        break;
    }
    if (!complete(state, step, queue, *(step.baseMotion_))) return false;
  }

  return true;
}

bool StepCompleter::complete(const State& state, const Step& step, EndEffectorMotionBase& endEffectorMotion) const
{
  // Input.
//  ControlSetup controlSetupIn = state.getControlSetup(endEffectorMotion.getLimb());
//  bool positionIn = controlSetupIn.at(ControlLevel::Position);
//  bool velocityIn = controlSetupIn.at(ControlLevel::Velocity);
//  bool accelerationIn = controlSetupIn.at(ControlLevel::Acceleration);
//  bool effortIn = controlSetupIn.at(ControlLevel::Effort);

  // Output.
  ControlSetup controlSetupOut = endEffectorMotion.getControlSetup();
  const bool positionOut = controlSetupOut.at(ControlLevel::Position);
  const bool velocityOut = controlSetupOut.at(ControlLevel::Velocity);
  const bool accelerationOut = controlSetupOut.at(ControlLevel::Acceleration);
  const bool effortOut = controlSetupOut.at(ControlLevel::Effort);

  if (positionOut) {
    const std::string& frameId = endEffectorMotion.getFrameId(ControlLevel::Position);
    if (!adapter_.frameIdExists(frameId)) {
      MELO_ERROR_STREAM("Could not find frame '" << frameId << "' for free gait leg motion!")
      return false;
    }
    Position startPositionInBaseFrame = adapter_.getPositionBaseToFootInBaseFrame(
        endEffectorMotion.getLimb(), state.getJointPositionsForLimb(endEffectorMotion.getLimb()));
    Position startPosition = adapter_.transformPosition("base", frameId, startPositionInBaseFrame);
    endEffectorMotion.updateStartPosition(startPosition);
  }

  if (velocityOut) {
    const std::string& frameId = endEffectorMotion.getFrameId(ControlLevel::Velocity);
    if (!adapter_.frameIdExists(frameId)) {
      MELO_ERROR_STREAM("Could not find frame '" << frameId << "' for free gait leg motion!")
      return false;
    }

    const JointVelocitiesLeg& jointVelocities = state.getJointVelocitiesForLimb(endEffectorMotion.getLimb());
    const LinearVelocity& startVelocity =
        adapter_.getEndEffectorLinearVelocityFromJointVelocities(
            endEffectorMotion.getLimb(), jointVelocities, frameId);
    endEffectorMotion.updateStartVelocity(startVelocity);
  }

  // Set initial force.
  if (effortOut) {
    const auto& frameId = endEffectorMotion.getFrameId(ControlLevel::Effort);
    if (!adapter_.frameIdExists(frameId)) {
      MELO_ERROR_STREAM("Could not find frame '" << frameId << "' for free gait leg motion!")
      return false;
    }

    auto startForceInWorldFrame = state.getEndEffectorForceInWorldFrame(endEffectorMotion.getLimb());

    const auto startForce = adapter_.transformForce(adapter_.getWorldFrameId(), frameId, startForceInWorldFrame);
    endEffectorMotion.updateStartEndEffectorForce(startForce);
  }

  return endEffectorMotion.prepareComputation(state, step, adapter_);
}

bool StepCompleter::complete(const State& state, const Step& step, JointMotionBase& jointMotion) const
{
  // Input.
  ControlSetup controlSetupIn = state.getControlSetup(jointMotion.getLimb());
  const bool positionIn = controlSetupIn.at(ControlLevel::Position);
  const bool velocityIn = controlSetupIn.at(ControlLevel::Velocity);
  const bool accelerationIn = controlSetupIn.at(ControlLevel::Acceleration);
  const bool effortIn = controlSetupIn.at(ControlLevel::Effort);

  // Output.
  ControlSetup controlSetupOut = jointMotion.getControlSetup();
  const bool positionOut = controlSetupOut.at(ControlLevel::Position);
  const bool velocityOut = controlSetupOut.at(ControlLevel::Velocity);
  const bool accelerationOut = controlSetupOut.at(ControlLevel::Acceleration);
  const bool effortOut = controlSetupOut.at(ControlLevel::Effort);

  // Check for special mode transitions.
  if (positionIn && !effortIn && positionOut && effortOut) {
    JointPositionsLeg startPosition = state.getJointPositionsForLimb(jointMotion.getLimb());
    jointMotion.updateStartPosition(startPosition);
    JointEffortsLeg startEffort = state.getJointEffortsForLimb(jointMotion.getLimb());
    startEffort.setZero();
    jointMotion.updateStartEfforts(startEffort);
  } else if (positionIn && effortIn && !positionOut && effortOut) {
    JointEffortsLeg startEffort = adapter_.getJointEffortsForLimb(jointMotion.getLimb());
    jointMotion.updateStartEfforts(startEffort);
  } //else if (positionIn && effortIn && positionOut && !effortOut) {
    // TODO: Decrease torque to zero in special step.
  //} else if (!positionIn && effortIn && positionOut && !effortOut) {
    // TODO: Decrease compression on leg in special step.
  // }
  else {

    // Standard transitions.
    if (positionOut) {
      JointPositionsLeg startPosition = state.getJointPositionsForLimb(jointMotion.getLimb());
      jointMotion.updateStartPosition(startPosition);
    }
    if (velocityOut) {
      JointVelocitiesLeg startVelocity = state.getJointVelocitiesForLimb(jointMotion.getLimb());
      jointMotion.updateStartVelocity(startVelocity);
    }
    if (accelerationOut) {
      JointAccelerationsLeg startAcceleration = state.getJointAccelerationsForLimb(jointMotion.getLimb());
      jointMotion.updateStartAcceleration(startAcceleration);
    }
    if (effortOut) {
      JointEffortsLeg startEffort = state.getJointEffortsForLimb(jointMotion.getLimb());
      jointMotion.updateStartEfforts(startEffort);
    }

  }
  return jointMotion.prepareComputation(state, step, adapter_);
}

bool StepCompleter::complete(const State& state, const Step& step, const StepQueue& queue, BaseMotionBase& baseMotion) const
{
  if (baseMotion.getControlSetup().at(ControlLevel::Position)) {
    const std::string& frameId = baseMotion.getFrameId(ControlLevel::Position);
    if (!adapter_.frameIdExists(frameId)) {
      MELO_ERROR_STREAM("Could not find frame '" << frameId << "' for free gait base motion!")
      return false;
    }
    Pose startPoseInWorld(state.getPositionWorldToBaseInWorldFrame(), state.getOrientationBaseToWorld());
    Pose startPose = adapter_.transformPose(adapter_.getWorldFrameId(), frameId, startPoseInWorld);
    baseMotion.updateStartPose(startPose);
  }
  if (baseMotion.getControlSetup().at(ControlLevel::Velocity)) {
    // TODO
  }
  return baseMotion.prepareComputation(state, step, queue, adapter_);
}

void StepCompleter::setParameters(LegMotionBase& legMotion) const
{
  if (legMotion.surfaceNormal_) {
    if (*(legMotion.surfaceNormal_) == Vector::Zero())
      legMotion.surfaceNormal_.reset(nullptr);
  }
}

void StepCompleter::setParameters(Footstep& footstep) const
{
  const auto& parameters = parameters_.footstepParameters;

  if (footstep.profileHeight_ == 0.0)
    footstep.profileHeight_ = parameters.profileHeight;
  if (footstep.profileType_.empty())
    footstep.profileType_ = parameters.profileType;
  if (footstep.averageVelocity_ == 0.0)
    footstep.averageVelocity_ = parameters.averageVelocity;

  footstep.liftOffSpeed_ = parameters.liftOffSpeed;
  footstep.touchdownSpeed_ = parameters.touchdownSpeed;
  footstep.minimumDuration_ = parameters.minimumDuration;
}

void StepCompleter::setParameters(EndEffectorTarget& endEffectorTarget) const
{
  const auto& parameters = parameters_.endEffectorTargetParameters;

  // If neither average velocity nor duration are set, take average velocity.
  if (endEffectorTarget.averageVelocity_ == 0.0 && endEffectorTarget.duration_ == 0.0) {
    endEffectorTarget.averageVelocity_ = parameters.averageVelocity;
    endEffectorTarget.useAverageVelocity_ = true;
  }
  endEffectorTarget.useAverageVelocity_ = (endEffectorTarget.averageVelocity_ > 0.0 && endEffectorTarget.duration_ <= 0.0);

  endEffectorTarget.minimumDuration_ = parameters.minimumDuration;
}

void StepCompleter::setParameters(LegMode& legMode) const
{
  const auto& parameters = parameters_.legModeParameters;

  if (legMode.duration_ == 0.0)
    legMode.duration_ = parameters.duration;
  if (legMode.frameId_.empty())
    legMode.frameId_ = parameters.frameId;
}

void StepCompleter::setParameters(BaseAuto& baseAuto) const
{
  baseAuto.frameId_ =  adapter_.getWorldFrameId();

  const auto& parameters = parameters_.baseAutoParameters;

  if (baseAuto.height_) {
    if (*(baseAuto.height_) == 0.0)
      baseAuto.height_.reset(nullptr);
  }
  if (baseAuto.averageLinearVelocity_ == 0.0)
    baseAuto.averageLinearVelocity_ = parameters.averageLinearVelocity;
  if (baseAuto.averageAngularVelocity_ == 0.0)
    baseAuto.averageAngularVelocity_ = parameters.averageAngularVelocity;
  if (baseAuto.supportMargin_ == 0.0)
    baseAuto.supportMargin_ = parameters.supportMargin;
  if (baseAuto.centerOfMassTolerance_ == 0.0)
    baseAuto.centerOfMassTolerance_ = parameters.centerOfMassTolerance;
  if (baseAuto.legLengthTolerance_ == 0.0)
    baseAuto.legLengthTolerance_ = parameters.legLengthTolerance;
  if (baseAuto.minLimbLengthScale_ == 0.0)
    baseAuto.minLimbLengthScale_ = parameters.minLimbLengthScale;
  if (baseAuto.maxLimbLengthAtClosingContactScale_ == 0.0)
    baseAuto.maxLimbLengthAtClosingContactScale_ = parameters.maxLimbLengthAtClosingContactScale;
  if (baseAuto.maxLimbLengthAtOpeningContactScale_ == 0.0)
    baseAuto.maxLimbLengthAtOpeningContactScale_ = parameters.maxLimbLengthAtOpeningContactScale;
  baseAuto.minimumDuration_ = parameters.minimumDuration;

  baseAuto.nominalPlanarStanceInBaseFrame_.clear();
  baseAuto.nominalPlanarStanceInBaseFrame_ = parameters.nominalPlanarStanceInBaseFrame;
}

void StepCompleter::setParameters(BaseTarget& baseTarget) const
{
  const auto& parameters = parameters_.baseTargetParameters;
  if (baseTarget.averageLinearVelocity_ == 0.0)
    baseTarget.averageLinearVelocity_ = parameters.averageLinearVelocity;
  if (baseTarget.averageAngularVelocity_ == 0.0)
    baseTarget.averageAngularVelocity_ = parameters.averageAngularVelocity;
  baseTarget.minimumDuration_ = parameters.minimumDuration;
}

void StepCompleter::setParameters(BaseTrajectory& baseTrajectory) const
{
//  const auto& parameters = parameters_.baseAutoParameters_;
//
//  if (baseAuto.controllerType_.empty())
//    baseAuto.controllerType_ = parameters.controllerType;
}

} /* namespace */

