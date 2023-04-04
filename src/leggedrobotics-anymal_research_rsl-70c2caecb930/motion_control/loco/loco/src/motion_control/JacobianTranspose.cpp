/*!
 * @file     JacobianTranspose.cpp
 * @author   Christian Gehring, Péter Fankhauser, Dario Bellicoso
 * @date     Aug, 2015
 * @brief
 */

// loco
#include "loco/motion_control/JacobianTranspose.hpp"
#include "loco/common/limbs/LimbLinkGroup.hpp"

namespace loco {

JacobianTranspose::JacobianTranspose(WholeBody& wholeBody, const bool isSettingJointPositionsFromDesiredBase)
    : MotionControllerBase(wholeBody), isSettingJointPositionsFromDesiredBase_(isSettingJointPositionsFromDesiredBase) {}

bool JacobianTranspose::loadParameters(const TiXmlHandle& handle) {
  return true;
}

bool JacobianTranspose::addVariablesToLog(const std::string& ns) const {
  return true;
}

bool JacobianTranspose::initialize(double dt) {
  return true;
}

bool JacobianTranspose::advance(double dt) {
  for (auto limb : *wholeBody_.getLimbsPtr()) {
    /*
     * Torque set points should be updated only is limb is support limb.
     */
    JointTorques jointTorques = limb->getInitializedJointTorques();
    if ((limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
      // Force at end effector represents the force applied by the environment on the robot.
      // So the end effector should push the ground by the opposite amount.
      computeJointTorquesFromForceAtEndEffectorInWorldFrame(jointTorques, limb,
                                                            -limb->getEndEffector().getStateDesired().getForceAtEndEffectorInWorldFrame());
      limb->getLimbStateDesiredPtr()->setJointTorques(jointTorques);
      if (isSettingJointPositionsFromDesiredBase_) {
        setJointPositionsFromDesiredBase(limb);
      }
    }
  }

  return true;
}

void JacobianTranspose::computeJointTorquesFromForceAtEndEffectorInWorldFrame(
    JointTorques& jointTorques, LimbBase* limb, const Force& desiredContactForceAtEndEffectorInWorldFrame) const {
  const LinearAcceleration gravitationalAccelerationInWorldFrame = torso_.getProperties().getGravity();
  const LinearAcceleration gravitationalAccelerationInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(gravitationalAccelerationInWorldFrame);
  const TranslationJacobian& jacobian = limb->getEndEffector().getStateMeasured().getTranslationJacobianBaseToEndEffectorInBaseFrame();

  const Force contactForceInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(desiredContactForceAtEndEffectorInWorldFrame);
  jointTorques = JointTorques(jacobian.transpose() * contactForceInBaseFrame.toImplementation()) +
                 limb->getLimbStateMeasured().getGravityJointTorques();
}

void JacobianTranspose::setJointPositionsFromDesiredBase(LimbBase* limb) {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const RotationQuaternion& orientationControlToDesiredBase = torso_.getDesiredState().getOrientationControlToBase();
  const RotationQuaternion orientationWorldToDesiredBase = orientationControlToDesiredBase * orientationWorldToControl;

  Position positionControlToDesiredBaseInControlFrame = torso_.getDesiredState().getPositionControlToTargetInControlFrame();
  Position positionWorldToDesiredBaseInWorldFrame = orientationWorldToControl.inverseRotate(positionControlToDesiredBaseInControlFrame);

  Position positionWorldToFootInWorldFrame = limb->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  Position positionDesiredBaseToFootInBaseFrame =
      orientationWorldToDesiredBase.rotate(positionWorldToFootInWorldFrame - positionWorldToDesiredBaseInWorldFrame);
  JointPositions desiredJoints =
      limb->getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(positionDesiredBaseToFootInBaseFrame);
  limb->getLimbStateDesiredPtr()->setJointPositions(desiredJoints);
}

void JacobianTranspose::setJointPositionsFromDesiredBase() {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const RotationQuaternion& orientationControlToDesiredBase = torso_.getDesiredState().getOrientationControlToBase();
  const RotationQuaternion orientationWorldToDesiredBase = orientationControlToDesiredBase * orientationWorldToControl;

  Position positionControlToDesiredBaseInControlFrame = torso_.getDesiredState().getPositionControlToTargetInControlFrame();
  Position positionWorldToDesiredBaseInWorldFrame = orientationWorldToControl.inverseRotate(positionControlToDesiredBaseInControlFrame);

  for (auto limb : *wholeBody_.getLimbsPtr()) {
    if ((limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
      Position positionWorldToFootInWorldFrame = limb->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
      Position positionDesiredBaseToFootInBaseFrame =
          orientationWorldToDesiredBase.rotate(positionWorldToFootInWorldFrame - positionWorldToDesiredBaseInWorldFrame);
      JointPositions desiredJoints =
          limb->getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(positionDesiredBaseToFootInBaseFrame);
      limb->getLimbStateDesiredPtr()->setJointPositions(desiredJoints);
    }
  }
}

void JacobianTranspose::activateIsSettingJointPositionsFromDesiredBase(const bool activate) {
  isSettingJointPositionsFromDesiredBase_ = activate;
}

} /* namespace loco */
