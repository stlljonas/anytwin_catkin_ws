/*
 * FootholdGeneratorInvertedPendulumPathRegularizer.cpp
 *
 *  Created on: Jan 09, 2020
 *      Author: Fabian Jenelten
 */

// tinyxml tools.
#include "tinyxml_tools/tinyxml_tools.hpp"

// loco.
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumPathRegularizer.hpp"

namespace loco {

FootholdGeneratorInvertedPendulumPathRegularizer::FootholdGeneratorInvertedPendulumPathRegularizer(
    WholeBody& wholeBody,
    TerrainModelBase& terrain,
    ContactScheduleZmp& contactSchedule,
    ComSupportControlZmp& comSupportControl)
    : FootholdGeneratorInvertedPendulumMotionGen(wholeBody, terrain, contactSchedule),
      comSupportControl_(comSupportControl),
      adaptFoothooldToPathRegularizer_(true) {
}

bool FootholdGeneratorInvertedPendulumPathRegularizer::loadParameters(const TiXmlHandle& handle) {
  if (!FootholdGeneratorInvertedPendulumMotionGen::loadParameters(handle)) {
    return false;
  }

  TiXmlHandle fpsHandle = handle;
  if (!tinyxml_tools::getChildHandle(fpsHandle, handle, "FootPlacementStrategy")) { return false; }

  TiXmlHandle fhgHandle = handle;
  if (!tinyxml_tools::getChildHandle(fhgHandle, fpsHandle, "FootholdGenerator")) { return false; }

  TiXmlHandle optfhgHandle = handle;
  if (!tinyxml_tools::getChildHandle(optfhgHandle, fhgHandle, "OptimizedInvPend")) { return false; }

  if (!tinyxml_tools::loadParameter(adaptFoothooldToPathRegularizer_, optfhgHandle, "adapt_to_path_regularizer")) { return false; }

  return true;
}

Position FootholdGeneratorInvertedPendulumPathRegularizer::evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg) {
  const auto legId = static_cast<contact_schedule::LegEnumAnymal>(leg.getId());
  Position positionDefaultFootholdToVelocityProjectionInControlFrame = Position::Zero();

  if (stepFeedforwardScale_ <= 0.0) {
    return positionReferencePointToDefaultFootholdInControlFrame_[legId];
  }

  // Rotations.
  const auto& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const auto& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
  const auto orientationBaseToControl = orientationWorldToControl * orientationWorldToBase.inverted();

  // Gait timing.
  const double timeUntilNextTouchDown = std::fmax(contactSchedule_.getTimeUntilNextStance(legId), 0.0);

  if (adaptFoothooldToPathRegularizer_) {
    // Predicted thigh position at touch-down.
    Position positionWorldToLimbThighAtTouchDownInWorldFrame;
    comSupportControl_.getPredictedThighPositionUsingPathRegularizer(legId, timeUntilNextTouchDown, positionWorldToLimbThighAtTouchDownInWorldFrame);

    const Position positionWorldToThighInWorldFrame =
        orientationWorldToBase.inverseRotate(leg.getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame()) +
        torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();

    // Current thigh position to predicted thigh position.
    positionDefaultFootholdToVelocityProjectionInControlFrame = orientationWorldToControl.rotate(
        positionWorldToLimbThighAtTouchDownInWorldFrame - positionWorldToThighInWorldFrame
    );
  } else {
    // Get desired velocity twist.
    const LinearVelocity linearVelocityTargetInControlFrame(
        torso_.getDesiredState().getLinearVelocityTargetInControlFrame().x(),
        torso_.getDesiredState().getLinearVelocityTargetInControlFrame().y(),
        0.0
     );

    const LocalAngularVelocity angularVelocityTargetInControlFrame(
        0.0,
        0.0,
        torso_.getDesiredState().getAngularVelocityBaseInControlFrame().z()
    );

    // Get base to end-effector position.
    // ToDo: should it be limb-thigh instead of end-effector??
    Position positionBaseToEndEffectorInControlFrame = orientationBaseToControl.rotate(
        leg.getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame()
    );
    positionBaseToEndEffectorInControlFrame.z() = 0.0;

    // Evaluate feed-forward component.
   positionDefaultFootholdToVelocityProjectionInControlFrame = Position(
        linearVelocityTargetInControlFrame.toImplementation() +
        angularVelocityTargetInControlFrame.toImplementation().cross(positionBaseToEndEffectorInControlFrame.toImplementation())
    ) * timeUntilNextTouchDown;

  }

  // Build foot position and return it.
  return (
      positionReferencePointToDefaultFootholdInControlFrame_[legId] +
      stepFeedforwardScale_ * positionDefaultFootholdToVelocityProjectionInControlFrame
  );
}

} /* namespace loco */
