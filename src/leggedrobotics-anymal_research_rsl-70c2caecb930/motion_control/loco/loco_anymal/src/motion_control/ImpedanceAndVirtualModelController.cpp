/*
 * ImpedanceAndVirtualModelController.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// loco
#include <loco_anymal/motion_control/ImpedanceAndVirtualModelController.hpp>
#include <loco/common/loco_common.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// robot utils
#include <robot_utils/math/math.hpp>

// message logger
#include <message_logger/message_logger.hpp>

using namespace message_logger::color;

namespace loco {

ImpedanceAndVirtualModelController::ImpedanceAndVirtualModelController(
    loco_anymal::WholeBodyAnymal& wholeBody,
    anymal_model::AnymalModel& anymalModel,
    anymal_model::AnymalModel& anymalModelDesired,
    ContactForceDistributionInterface& contactForceDistribution)
    : VirtualModelControllerContactInvariantDamper(wholeBody, contactForceDistribution),
      anymalModel_(anymalModel),
      anymalModelDesired_(anymalModelDesired),
      timeSinceInitialization_(0.0),
      interpolationDuration_(0.0)
{
   jacobianTranspose_.activateIsSettingJointPositionsFromDesiredBase(false);
}

bool ImpedanceAndVirtualModelController::loadParameters(const TiXmlHandle& handle)
{
  const TiXmlHandle impedanceControllerHandle = tinyxml_tools::getChildHandle(handle, "ImpedanceController");
  const TiXmlHandle regainContactHandle = tinyxml_tools::getChildHandle(impedanceControllerHandle, "Interpolation");
  tinyxml_tools::loadParameter(interpolationDuration_, regainContactHandle, "duration", 0.0);
  return VirtualModelControllerContactInvariantDamper::loadParameters(handle);
}

bool ImpedanceAndVirtualModelController::initialize(double dt)
{
  timeSinceInitialization_ = 0.0;
  return VirtualModelControllerContactInvariantDamper::initialize(dt);
}

bool ImpedanceAndVirtualModelController::setControlModeForLimbs() {
  for (auto limb : limbs_) {
    setControlModeForLimb(limb, loco::ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS,
                         nonSupportLegControlMode_);
  }
  return true;
}

bool ImpedanceAndVirtualModelController::advance(double dt)
{
  timeSinceInitialization_ += dt;

  bool success = true;

  success &= setControlModeForLimbs();
  success &= computeError();
  success &= computeGravityCompensation();
  success &= computeVirtualForce(dt);
  success &= computeVirtualTorque(dt);

  if (!contactForceDistribution_.computeForceDistribution(virtualForceInBaseFrame_, virtualTorqueInBaseFrame_)) {
    MELO_WARN("[VirtualModelController::advance] Computation of contact force distribution returned false!");
    return false;
  }

  // Set torques from desired contact forces
  if(!jacobianTranspose_.advance(dt)) {
    MELO_WARN("Loco: jacobianTranspose_->advance() returned false!");
    return false;
  }

  setJointPositionsFromDesiredBase();
  setJointVelocitiesFromDesiredBase();
  return success;
}

// TODO This should be merged with the method from Jacobian Transpose.
void ImpedanceAndVirtualModelController::setJointPositionsFromDesiredBase()
{
  for (auto leg: *wholeBody_.getLegsPtr()) {
    if (!((leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support)
        || (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant))) {
      continue;
    }

    const Position positionWorldToFootInWorldFrame = leg->getStateTouchDown()->getPositionWorldToFootInWorldFrame();

    // Measured state.
    const RotationQuaternion orientationWorldToMeasuredBase = torso_.getMeasuredState().getOrientationWorldToBase();
    const Position positionWorldToMeasuredBaseInWorldFrame = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
    const Position positionMeasuredBaseToFootInBaseFrame = orientationWorldToMeasuredBase.rotate(positionWorldToFootInWorldFrame-positionWorldToMeasuredBaseInWorldFrame);

    // Desired state.
    const RotationQuaternion orientationWorldToDesiredBase(torso_.getDesiredState().getOrientationEulerAnglesZyxBaseToWorld().inverted());
    const Position positionWorldToDesiredBaseInWorldFrame = torso_.getDesiredState().getPositionWorldToBaseInWorldFrame();
    const Position positionDesiredBaseToFootInBaseFrame = orientationWorldToDesiredBase.rotate(positionWorldToFootInWorldFrame-positionWorldToDesiredBaseInWorldFrame);

    // Interpolation.
    const double timeSinceTouchdown = timeSinceInitialization_ - leg->getStateTouchDown()->stateChangedAtTime();
    double interpolationFactor = robot_utils::mapTo01Range(timeSinceTouchdown, 0.0, interpolationDuration_);
    if (timeSinceInitialization_ < interpolationDuration_) {
      interpolationFactor = 1.0; // Stable start after initialization.
    }

    const Position positionBaseToFootInBaseFrame = (1.0 - interpolationFactor) * positionMeasuredBaseToFootInBaseFrame
                                                    + interpolationFactor * positionDesiredBaseToFootInBaseFrame;

    Eigen::VectorXd desiredJoints;
    anymalModel_.getLimbJointPositionsFromLimbEnumIteratively(
        desiredJoints, positionBaseToFootInBaseFrame.vector(),
        AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt()));

    leg->getLimbStateDesiredPtr()->setJointPositions(desiredJoints);
  }
}

void ImpedanceAndVirtualModelController::setJointVelocitiesFromDesiredBase() {
  for (auto leg: *wholeBody_.getLegsPtr()) {
    if (!((leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support)
       || (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant))) {
      continue;
    }

    // TODO Add correct velocities.
    loco::JointVelocities jointVelocities;
    leg->populateJointVelocities(jointVelocities);
    jointVelocities.setZero();
    leg->getLimbStateDesiredPtr()->setJointVelocities(jointVelocities);
  }
}

} /* namespace loco */
