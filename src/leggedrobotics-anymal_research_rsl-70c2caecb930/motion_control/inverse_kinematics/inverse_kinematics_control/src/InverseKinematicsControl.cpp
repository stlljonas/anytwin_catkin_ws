/**
 * @file        InverseKinematicsControl.cpp
 * @authors     Fabian Jenelten
 * @date        Mar 23, 2020
 * @affiliation ETH RSL
 */

// inverse_control.
#include "inverse_kinematics_control/InverseKinematicsControl.hpp"

// loco.
#include <loco/common/loco_common.hpp>

// message logger.
#include <message_logger/message_logger.hpp>

// tinyxml tools.
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace inverse_kinematics_control {

InverseKinematicsControl::InverseKinematicsControl(loco::WholeBody& wholeBody, bool useAsTrackingController, bool useVelocityTracking,
                                                   bool useDesiredTorsoState)
    : Base(wholeBody),
      useAsTrackingController_(useAsTrackingController),
      useVelocityTracking_(useVelocityTracking),
      useDesiredTorsoState_(useDesiredTorsoState),
      limbControlMode_(loco::JointControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS),
      desiredJointPositions_(),
      desiredJointVelocities_(),
      desiredJointTorques_() {}

bool InverseKinematicsControl::initialize(double dt) {
  constexpr double filterTimeConstant = 0.003;

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto branchEnum = AD::mapKeyEnumToKeyEnum<AD::ContactEnum, AD::BranchEnum>(contactEnum);
    const auto limbEnum = AD::mapEnums<AD::LimbEnum>(branchEnum);
    const auto contactId = contactKey.getId();
    const auto& limb = wholeBody_.getLimbs().get(contactId);

    desiredJointPositions_[contactEnum].reset(
        new basic_filters::FirstOrderFilter<Eigen::MatrixXd>(dt, filterTimeConstant, 1.0, limb.getLimbStateMeasured().getJointPositions()));

    desiredJointVelocities_[contactEnum].reset(new basic_filters::FirstOrderFilter<Eigen::MatrixXd>(
        dt, filterTimeConstant, 1.0, 0.0 * limb.getLimbStateMeasured().getJointVelocities()));

    desiredJointTorques_[contactEnum].setZero(AD::getNumDofLimb(limbEnum));
  }

  return true;
}

bool InverseKinematicsControl::advance(double dt) {
  if (!computeControlReferences(dt)) {
    MELO_WARN_STREAM("Failed to compute control reference signals.")
    return false;
  }

  if (!setControlReferencesToLimbs()) {
    MELO_WARN_STREAM("Failed to set control reference signals.")
    return false;
  }

  if (!setControlModeForLimbs()) {
    MELO_WARN_STREAM("Failed to set control mode.")
    return false;
  }

  return true;
}

bool InverseKinematicsControl::loadParameters(const TiXmlHandle& /*handle*/) {
  return true;
}

bool InverseKinematicsControl::setControlModeForLimbs() {
  if (!useAsTrackingController_) {
    return true;
  }

  for (auto limb : *wholeBody_.getLimbsPtr()) {
    if (!loco::setControlModeForLimb(limb, limbControlMode_, limbControlMode_)) {
      MELO_WARN_STREAM("Failed to set control mode for limb " << limb->getName() << ".")
      return false;
    }
  }

  return true;
}

bool InverseKinematicsControl::computeControlReferences(double /*dt*/) {
  // Get torso state.
  loco::RotationQuaternion orientationWorldToBase;
  loco::Position positionWorldToBaseInWorldFrame;
  loco::LinearVelocity linearVelocityBaseInBaseFrame;
  loco::LocalAngularVelocity angularVelocityBaseInBaseFrame;

  if (useDesiredTorsoState_) {
    const loco::RotationQuaternion& orientationControlToBase = torso_.getDesiredState().getOrientationControlToBase();
    const loco::RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
    orientationWorldToBase = orientationControlToBase * orientationWorldToControl;
    positionWorldToBaseInWorldFrame = torso_.getDesiredState().getPositionWorldToBaseInWorldFrame();
    linearVelocityBaseInBaseFrame = orientationControlToBase.rotate(torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
    angularVelocityBaseInBaseFrame = orientationControlToBase.rotate(torso_.getDesiredState().getAngularVelocityBaseInControlFrame());
  } else {
    orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
    positionWorldToBaseInWorldFrame = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
    linearVelocityBaseInBaseFrame = torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame();
    angularVelocityBaseInBaseFrame = torso_.getMeasuredState().getAngularVelocityBaseInBaseFrame();
  }

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactId = contactKey.getId();
    const auto contactEnum = contactKey.getEnum();

    auto limb = wholeBody_.getLimbsPtr()->getPtr(contactId);
    auto endEffector = limb->getEndEffectorPtr();
    auto footDesiredState = endEffector->getStateDesiredPtr();

    // Compute joint positions through inverse kinematics.
    const loco::Position positionBaseToEndEffectorInBaseFrame =
        orientationWorldToBase.rotate(footDesiredState->getPositionWorldToEndEffectorInWorldFrame() - positionWorldToBaseInWorldFrame);

    if (desiredJointPositions_[contactEnum] == nullptr) {
      MELO_WARN_STREAM("Joint position filter is null pointer for limb " << limb->getName() << ".")
      return false;
    }
    desiredJointPositions_[contactEnum]->advance(
        endEffector->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(positionBaseToEndEffectorInBaseFrame));

    // Compute joint velocities through inverse differential kinematics (zero if not in velocity tracking mode).
    if (useVelocityTracking_) {
      const loco::LinearVelocity linearVelocityEndEffectorInBaseFrame =
          orientationWorldToBase.rotate(footDesiredState->getLinearVelocityEndEffectorInWorldFrame());
      const loco::LinearVelocity linearVelocityBaseToEndEffectorInBaseFrame =
          linearVelocityEndEffectorInBaseFrame - linearVelocityBaseInBaseFrame -
          loco::LinearVelocity(
              angularVelocityBaseInBaseFrame.toImplementation().cross(positionBaseToEndEffectorInBaseFrame.toImplementation()));

      if (desiredJointVelocities_[contactEnum] == nullptr) {
        MELO_WARN_STREAM("Joint velocity filter is null pointer for limb " << limb->getName() << ".")
        return false;
      }
      desiredJointVelocities_[contactEnum]->advance(
          endEffector->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(linearVelocityBaseToEndEffectorInBaseFrame));
    }
  }

  return true;
}

bool InverseKinematicsControl::setControlReferencesToLimbs() {
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto contactId = contactKey.getId();
    auto limb = wholeBody_.getLimbsPtr()->getPtr(contactId);

    if (desiredJointPositions_[contactEnum] == nullptr || desiredJointVelocities_[contactEnum] == nullptr) {
      MELO_WARN_STREAM("Joint position or velocity filter is null pointer for limb " << limb->getName() << ".")
      return false;
    }

    // Set desired joint positions and velocities.
    limb->getLimbStateDesiredPtr()->setJointPositions(desiredJointPositions_[contactEnum]->getFilteredValue());
    limb->getLimbStateDesiredPtr()->setJointVelocities(desiredJointVelocities_[contactEnum]->getFilteredValue());

    // Overwrite joint torques if in tracking controller mode.
    if (useAsTrackingController_) {
      limb->getLimbStateDesiredPtr()->setJointTorques(desiredJointTorques_[contactEnum]);
    }
  }

  return true;
}

} /* namespace inverse_kinematics_control */
