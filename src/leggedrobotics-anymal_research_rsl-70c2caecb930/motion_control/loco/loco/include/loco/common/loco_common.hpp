/*
 * loco_common.hpp
 *
 *  Created on: Sep 18, 2015
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/WholeBody.hpp>
#include <loco/common/typedefs.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace loco {

inline static void setJointPositionsFromDesiredBase(const TorsoBase& torso, Legs* legs) {
  const RotationQuaternion& orientationWorldToControl = torso.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const RotationQuaternion& orientationControlToDesiredBase = torso.getDesiredState().getOrientationControlToBase();
  const RotationQuaternion orientationWorldToDesiredBase = orientationControlToDesiredBase * orientationWorldToControl;

  const Position& positionControlToDesiredBaseInControlFrame = torso.getDesiredState().getPositionControlToTargetInControlFrame();
  const Position positionWorldToDesiredBaseInWorldFrame =
      orientationWorldToControl.inverseRotate(positionControlToDesiredBaseInControlFrame);

  for (auto leg : *legs) {
    if ((leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
      const Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
      const Position positionDesiredBaseToFootInBaseFrame =
          orientationWorldToDesiredBase.rotate(positionWorldToFootInWorldFrame - positionWorldToDesiredBaseInWorldFrame);
      leg->getLimbStateDesiredPtr()->setJointPositions(
          leg->getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(positionDesiredBaseToFootInBaseFrame));
    }
  }
}

inline static void setJointPositionsFromMeasuredBase(const TorsoBase& torso, Legs* legs) {
  for (auto leg : *legs) {
    const Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame();
    leg->getLimbStateDesiredPtr()->setJointPositions(leg->getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(
        torso.getMeasuredState().getOrientationWorldToBase().rotate(positionWorldToFootInWorldFrame -
                                                                    torso.getMeasuredState().getPositionWorldToBaseInWorldFrame())));

    const LinearVelocity& linearVelocityDesFootInWorldFrame = leg->getFoot().getStateDesired().getLinearVelocityEndEffectorInWorldFrame();
    leg->getLimbStateDesiredPtr()->setJointVelocities(
        leg->getEndEffectorPtr()->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(
            torso.getMeasuredState().getOrientationWorldToBase().rotate(linearVelocityDesFootInWorldFrame) -
            torso.getMeasuredState().getLinearVelocityBaseInBaseFrame() -
            LinearVelocity(torso.getMeasuredState().getAngularVelocityBaseInBaseFrame().toImplementation().cross(
                leg->getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame().toImplementation()))));
  }
}

inline static bool setControlModeForLeg(LegBase* leg, const ControlMode supportLegControlMode, const ControlMode nonSupportLegControlMode) {
  JointControlModes desiredJointControlModes = leg->getInitializedJointControlModes();
  if ((leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
      (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
    desiredJointControlModes.setConstant(supportLegControlMode);
  } else {
    desiredJointControlModes.setConstant(nonSupportLegControlMode);
  }
  leg->getLimbStateDesiredPtr()->setJointControlModes(desiredJointControlModes);

  return true;
}

inline static bool setControlModeForLimb(LimbBase* limb, const ControlMode supportLegControlMode,
                                         const ControlMode nonSupportLegControlMode) {
  JointControlModes desiredJointControlModes = limb->getInitializedJointControlModes();
  if ((limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
      (limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
    desiredJointControlModes.setConstant(supportLegControlMode);
  } else {
    desiredJointControlModes.setConstant(nonSupportLegControlMode);
  }
  limb->getLimbStateDesiredPtr()->setJointControlModes(desiredJointControlModes);

  return true;
}

inline static void getMixedComBasePositionError(const WholeBody& wholeBody, Position& positionErrorInControlFrame,
                                                const Position& positionControlToDesiredTorsoidInControlFrame) {
  const RotationQuaternion& orientationWorldToControl =
      wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const Position positionControlToWholeBodyComInControlFrame =
      orientationWorldToControl.rotate(wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame() -
                                       wholeBody.getTorso().getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame());
  const Position positionControlToTorsoidInControlFrame =
      Position(positionControlToWholeBodyComInControlFrame.x(), positionControlToWholeBodyComInControlFrame.y(),
               wholeBody.getTorso().getMeasuredState().inControlFrame().getPositionControlToBaseInControlFrame().z());

  positionErrorInControlFrame = positionControlToDesiredTorsoidInControlFrame - positionControlToTorsoidInControlFrame;
}

inline static void getMixedComBaseLinearVelocityError(const WholeBody& wholeBody, LinearVelocity& linearVelocityErrorInControlFrame,
                                                      const LinearVelocity& linearVelocityDesiredTorsoidInControlFrame) {
  const RotationQuaternion& orientationWorldToControl =
      wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const RotationQuaternion& orientationControlToBase =
      wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationControlToBase();
  const LinearVelocity linearVelocityWholeBodyComInControlFrame =
      orientationWorldToControl.rotate(wholeBody.getWholeBodyStateMeasured().getLinearVelocityWholeBodyCenterOfMassInWorldFrame());
  const LinearVelocity linearVelocityBaseInControlFrame =
      orientationControlToBase.inverseRotate(wholeBody.getTorso().getMeasuredState().getLinearVelocityBaseInBaseFrame());
  const LinearVelocity linearVelocityTorsoidInControlFrame(
      linearVelocityWholeBodyComInControlFrame.x(), linearVelocityWholeBodyComInControlFrame.y(), linearVelocityBaseInControlFrame.z());
  linearVelocityErrorInControlFrame = linearVelocityDesiredTorsoidInControlFrame - linearVelocityTorsoidInControlFrame;
}

inline static void getBasePositionError(const WholeBody& wholeBody, Position& positionErrorInControlFrame,
                                        const Position& positionControlToDesiredBaseInControlFrame) {
  positionErrorInControlFrame = positionControlToDesiredBaseInControlFrame -
                                wholeBody.getTorso().getMeasuredState().inControlFrame().getPositionControlToBaseInControlFrame();
}

inline static void getBaseLinearVelocityError(const WholeBody& wholeBody, LinearVelocity& linearVelocityErrorInControlFrame,
                                              const LinearVelocity& linearVelocityDesiredBaseInControlFrame) {
  const RotationQuaternion& orientationControlToBase =
      wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationControlToBase();
  linearVelocityErrorInControlFrame =
      linearVelocityDesiredBaseInControlFrame -
      orientationControlToBase.inverseRotate(wholeBody.getTorso().getMeasuredState().getLinearVelocityBaseInBaseFrame());
}

inline static void getComPositionError(const WholeBody& wholeBody, Position& positionErrorInControlFrame,
                                       const Position& positionControlToDesiredComInControlFrame) {
  positionErrorInControlFrame = positionControlToDesiredComInControlFrame -
                                wholeBody.getWholeBodyStateMeasured().getPositionControlToWholeBodyCenterOfMassInControlFrame();
}

inline static void getComVelocityError(const WholeBody& wholeBody, LinearVelocity& linearVelocityErrorInControlFrame,
                                       const LinearVelocity& linearVelocityDesiredComInControlFrame) {
  linearVelocityErrorInControlFrame =
      linearVelocityDesiredComInControlFrame - wholeBody.getWholeBodyStateMeasured().getLinearVelocityWholeBodyCenterOfMassInControlFrame();
}

inline static void getTorsoComPositionError(const WholeBody& wholeBody, Position& positionErrorInControlFrame,
                                            const Position& positionControlToDesiredTorsoInControlFrame) {
  const RotationQuaternion& orientationWorldToControl =
      wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const Position positionControlToTorsoInWorldFrame =
      wholeBody.getTorso().getMeasuredState().getPositionWorldToCenterOfMassInWorldFrame() -
      wholeBody.getTorso().getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame();
  positionErrorInControlFrame =
      positionControlToDesiredTorsoInControlFrame - orientationWorldToControl.rotate(positionControlToTorsoInWorldFrame);
}

inline static void getTorsoComVelocityError(const WholeBody& wholeBody, LinearVelocity& linearVelocityErrorInControlFrame,
                                            const LinearVelocity& linearVelocityDesiredTorsoInControlFrame) {
  const RotationQuaternion& orientationControlToBase =
      wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationControlToBase();
  linearVelocityErrorInControlFrame =
      linearVelocityDesiredTorsoInControlFrame -
      orientationControlToBase.inverseRotate(wholeBody.getTorso().getMeasuredState().getLinearVelocityCenterOfMassInBaseFrame());
}

inline static bool getErrorInControlFrame(const WholeBody& wholeBody, Position& positionErrorInControlFrame,
                                          LinearVelocity& linearVelocityErrorInControlFrame,
                                          LocalAngularVelocity& angularVelocityErrorInControlFrame,
                                          const Position& positionControlToDesiredTargetInControlFrame,
                                          const LinearVelocity& linearVelocityDesiredTargetInControlFrame,
                                          const LocalAngularVelocity& angularVelocityDesiredBaseInControlFrame) {
  switch (wholeBody.getTorso().getDesiredState().getTargetPoint()) {
    case (TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ): {
      getMixedComBasePositionError(wholeBody, positionErrorInControlFrame, positionControlToDesiredTargetInControlFrame);
      getMixedComBaseLinearVelocityError(wholeBody, linearVelocityErrorInControlFrame, linearVelocityDesiredTargetInControlFrame);
    } break;

    case (TorsoStateDesired::TargetPoint::WBCOM): {
      getComPositionError(wholeBody, positionErrorInControlFrame, positionControlToDesiredTargetInControlFrame);
      getComVelocityError(wholeBody, linearVelocityErrorInControlFrame, linearVelocityDesiredTargetInControlFrame);
    } break;

    case (TorsoStateDesired::TargetPoint::BASE): {
      getBasePositionError(wholeBody, positionErrorInControlFrame, positionControlToDesiredTargetInControlFrame);
      getBaseLinearVelocityError(wholeBody, linearVelocityErrorInControlFrame, linearVelocityDesiredTargetInControlFrame);
    } break;

    case (TorsoStateDesired::TargetPoint::TORSOCOM): {
      getTorsoComPositionError(wholeBody, positionErrorInControlFrame, positionControlToDesiredTargetInControlFrame);
      getTorsoComVelocityError(wholeBody, linearVelocityErrorInControlFrame, linearVelocityDesiredTargetInControlFrame);
    } break;

    default: {
      MELO_FATAL("[loco_common::getErrorInControlFrame] Warning: Undefined target point.");
      return false;
    } break;
  }

  const RotationQuaternion& orientationControlToBase =
      wholeBody.getTorso().getMeasuredState().inControlFrame().getOrientationControlToBase();
  angularVelocityErrorInControlFrame =
      angularVelocityDesiredBaseInControlFrame -
      orientationControlToBase.inverseRotate(wholeBody.getTorso().getMeasuredState().getAngularVelocityBaseInBaseFrame());

  return true;
}

inline static void getPositionBaseToTargetPointInWorldFrame(const WholeBody& wholeBody, Position& positionBaseToTargetPointInWorldFrame) {
  switch (wholeBody.getTorso().getDesiredState().getTargetPoint()) {
    case (TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ): {
      positionBaseToTargetPointInWorldFrame = wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame() -
                                              wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
      positionBaseToTargetPointInWorldFrame.z() = 0.0;
    } break;

    case (TorsoStateDesired::TargetPoint::WBCOM): {
      // const Position& positionWorldToBaseInWorldFrame = wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
      positionBaseToTargetPointInWorldFrame = wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame() -
                                              wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
    } break;

    case (TorsoStateDesired::TargetPoint::BASE): {
      positionBaseToTargetPointInWorldFrame.setZero();
    } break;

    case (TorsoStateDesired::TargetPoint::TORSOCOM): {
      positionBaseToTargetPointInWorldFrame = wholeBody.getTorso().getMeasuredState().getOrientationWorldToBase().inverseRotate(
          wholeBody.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame());
    } break;

    default: { MELO_FATAL("Undefined target point!"); } break;
  }
}

inline static void getPositionBaseToTargetPointInBaseFrame(const WholeBody& wholeBody, Position& positionBaseToTargetPointInBaseFrame) {
  switch (wholeBody.getTorso().getDesiredState().getTargetPoint()) {
    case (TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ): {
      Position positionBaseToTargetPointInWorldFrame =
          wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame() -
          wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
      positionBaseToTargetPointInWorldFrame.z() = 0.0;
      positionBaseToTargetPointInBaseFrame =
          wholeBody.getTorso().getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToTargetPointInWorldFrame);
    } break;

    case (TorsoStateDesired::TargetPoint::WBCOM): {
      positionBaseToTargetPointInBaseFrame = wholeBody.getTorso().getMeasuredState().getOrientationWorldToBase().rotate(
          wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame() -
          wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame());
    } break;

    case (TorsoStateDesired::TargetPoint::BASE): {
      positionBaseToTargetPointInBaseFrame.setZero();
    } break;

    case (TorsoStateDesired::TargetPoint::TORSOCOM): {
      positionBaseToTargetPointInBaseFrame = wholeBody.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame();
    } break;

    default: { MELO_FATAL("Undefined target point!"); } break;
  }
}

inline static Position getPositionWorldToTargetInWorldFrame(const WholeBody& wholeBody) {
  switch (wholeBody.getTorso().getDesiredState().getTargetPoint()) {
    // 1. Mixed Com.
    case (TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ): {
      Position positionWorldToMeasuredTargetInWorldFrame =
          wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame();
      positionWorldToMeasuredTargetInWorldFrame.z() = wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame().z();
      return positionWorldToMeasuredTargetInWorldFrame;
    } break;

    // 2. Whole body Com.
    case (TorsoStateDesired::TargetPoint::WBCOM): {
      return wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame();
    } break;

    // 3. Base.
    case (TorsoStateDesired::TargetPoint::BASE): {
      return wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
    } break;

    // 4. Torso Com.
    case (TorsoStateDesired::TargetPoint::TORSOCOM): {
      return wholeBody.getTorso().getMeasuredState().getPositionWorldToCenterOfMassInWorldFrame();
    } break;

    default: {
      MELO_FATAL("Undefined target point!");
      return Position::Zero();
    } break;
  }
}

static inline LinearVelocity getLinearVelocityTargetInWorldFrame(const WholeBody& wholeBody) {
  const RotationQuaternion& orientationWorldToBase = wholeBody.getTorso().getMeasuredState().getOrientationWorldToBase();

  switch (wholeBody.getTorso().getDesiredState().getTargetPoint()) {
    // 1. Mixed Com.
    case (TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ): {
      LinearVelocity linearVelocityMeasuredTargetInWorldFrame =
          wholeBody.getWholeBodyStateMeasured().getLinearVelocityWholeBodyCenterOfMassInWorldFrame();
      linearVelocityMeasuredTargetInWorldFrame.z() =
          orientationWorldToBase.inverseRotate(wholeBody.getTorso().getMeasuredState().getLinearVelocityBaseInBaseFrame()).z();
      return linearVelocityMeasuredTargetInWorldFrame;
    } break;

    // 2. Whole body Com.
    case (TorsoStateDesired::TargetPoint::WBCOM): {
      return wholeBody.getWholeBodyStateMeasured().getLinearVelocityWholeBodyCenterOfMassInWorldFrame();
    } break;

    // 3. Base.
    case (TorsoStateDesired::TargetPoint::BASE): {
      return orientationWorldToBase.inverseRotate(wholeBody.getTorso().getMeasuredState().getLinearVelocityBaseInBaseFrame());
    } break;

    // 4. Torso Com.
    case (TorsoStateDesired::TargetPoint::TORSOCOM): {
      return orientationWorldToBase.inverseRotate(wholeBody.getTorso().getMeasuredState().getLinearVelocityCenterOfMassInBaseFrame());
    } break;

    default: {
      MELO_FATAL("[loco_common::getLinearVelocityTargetInWorldFrame] Undefined target point!");
      return LinearVelocity::Zero();
    } break;
  }
}

} /* namespace loco */
