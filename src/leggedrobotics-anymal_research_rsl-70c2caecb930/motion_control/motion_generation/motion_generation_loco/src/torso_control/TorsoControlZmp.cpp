/*
 * TorsoControlZmp.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/torso_control/TorsoControlZmp.hpp"


namespace loco {


TorsoControlZmp::TorsoControlZmp(WholeBody& wholeBody,
                                 TerrainModelBase& terrain,
                                 ComSupportControlZmp& comControl)
    : TorsoControlGaitContainer(wholeBody, terrain, comControl) {
}

bool TorsoControlZmp::advance(double dt) {
  ComSupportControlZmp& comControlZmp = getComControlRef();

  // compute new target values
  if(!comControl_.advance(dt)) {
    MELO_WARN_STREAM("[TorsoControlZmp::advance] Failed to advance com control.");
    return false;
  }

  // Get measured orientations
  const auto& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const auto& orientationControlToBase  = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();
  const auto& orientationWorldToBase    = torso_.getMeasuredState().getOrientationWorldToBase();
  const auto& positionWorldToControlInWorldFrame = torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame();

  // Get desired target point (target can be base, whole body com, mixed com or torso com)
  const auto positionWorldToDesiredTargetInControlFrame = orientationWorldToControl.rotate(comControl_.getPositionWorldToDesiredCoMInWorldFrame());
  const auto linearVelocityDesiredTargetInControlFrame  = orientationWorldToControl.rotate(comControl_.getLinearVelocityDesiredBaseInWorldFrame());
  const auto& angularVelocityDesiredBaseInControlFrame  = comControlZmp.getAngularVelocityDesiredBaseInControlFrame();

  // Get relative rotation between control and desired base frame.
  const auto& orientationControlToDesiredBase = comControlZmp.getOrientationControlToDesiredBase();

  // Set control references (Linear)
  torso_.getDesiredStatePtr()->setPositionControlToTargetInControlFrame(
      positionWorldToDesiredTargetInControlFrame -
      orientationWorldToControl.rotate(positionWorldToControlInWorldFrame)
  );
  torso_.getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(linearVelocityDesiredTargetInControlFrame);
  torso_.getDesiredStatePtr()->setLinearAccelerationTargetInControlFrame(comControlZmp.getLinearAccelerationDesiredTargetInControlFrame());

  // Set control reference (Angular).
  torso_.getDesiredStatePtr()->setOrientationControlToBase(orientationControlToDesiredBase);
  torso_.getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(angularVelocityDesiredBaseInControlFrame);
  torso_.getDesiredStatePtr()->setAngularAccelerationTargetInControlFrame(comControlZmp.getAngularAccelerationDesiredBaseInControlFrame());

  // Computed control errors.
  const Position positionErrorInControlFrame =
      positionWorldToDesiredTargetInControlFrame -
      orientationWorldToControl.rotate(getPositionWorldToTargetInWorldFrame(wholeBody_));

  const LinearVelocity linearVelocityErrorInControlFrame =
      linearVelocityDesiredTargetInControlFrame -
      orientationWorldToControl.rotate(getLinearVelocityTargetInWorldFrame(wholeBody_));

  const LocalAngularVelocity angularVelocityErrorInControlFrame =
      angularVelocityDesiredBaseInControlFrame -
      orientationControlToBase.inverseRotate(wholeBody_.getTorso().getMeasuredState().getAngularVelocityBaseInBaseFrame());

  // Set control errors.
  torso_.getDesiredStatePtr()->setPositionErrorInControlFrame(positionErrorInControlFrame);
  torso_.getDesiredStatePtr()->setLinearVelocityErrorInControlFrame(linearVelocityErrorInControlFrame);
  torso_.getDesiredStatePtr()->setAngularVelocityErrorInControlFrame(angularVelocityErrorInControlFrame);

  // Helper vector.
  Position positionBaseToTargetPointInBaseFrame;
  getPositionBaseToTargetPointInBaseFrame(wholeBody_, positionBaseToTargetPointInBaseFrame);

  const RotationQuaternion orientationWorldToDesiredBase = orientationControlToDesiredBase * orientationWorldToControl;
  Position positionDesiredBaseToTargetPointInWorldFrame = orientationWorldToDesiredBase.inverseRotate(positionBaseToTargetPointInBaseFrame);

  // Store desired base orientation for logging.
  const auto orientationEulerAnglesZyxWorldToDesiredBase = EulerAnglesZyx(orientationWorldToDesiredBase);
  torso_.getDesiredStatePtr()->setOrientationEulerAnglesZyxBaseToWorld(orientationEulerAnglesZyxWorldToDesiredBase.inverted().getUnique());

  // Compute desired base position. (assumption: vector base to target point is constant in base frame)
  const Position positionWorldToDesiredTargetInWorldFrame = comControl_.getPositionWorldToDesiredCoMInWorldFrame();
  const Position positionWorldToDesiredBaseInWorldFrame = positionWorldToDesiredTargetInWorldFrame - positionDesiredBaseToTargetPointInWorldFrame;
  torso_.getDesiredStatePtr()->setPositionWorldToBaseInWorldFrame(positionWorldToDesiredBaseInWorldFrame);

  // Compute desired com position (assumption: vector base to WBCOM is constant in base frame)
  const Position positionBaseToWholeBodyComInWorldFrame =
      wholeBody_.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame() -
      wholeBody_.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();

  wholeBody_.getWholeBodyStateDesiredPtr()->setPositionWorldToWholeBodyCenterOfMassInWorldFrame(
      positionWorldToDesiredBaseInWorldFrame +
      orientationWorldToDesiredBase.inverseRotate(orientationWorldToBase.rotate(positionBaseToWholeBodyComInWorldFrame))
  );

  return true;
}

bool TorsoControlZmp::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
  return false;
}

bool TorsoControlZmp::addParametersToHandler(const std::string& ns) {
  return comControl_.addParametersToHandler(ns);
}

bool TorsoControlZmp::removeParametersFromHandler() {
  return comControl_.removeParametersFromHandler();
}

bool TorsoControlZmp::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle torsoControlHandle = handle;
  if (!tinyxml_tools::getChildHandle(torsoControlHandle, handle, "TorsoControl")) { return false; }

  TiXmlHandle torsoConfigurationHandle = handle;
  if (!tinyxml_tools::getChildHandle(torsoConfigurationHandle, torsoControlHandle, "TorsoConfiguration")) { return false; }

  TiXmlHandle referencePointHandle = handle;
  if (!tinyxml_tools::getChildHandle(referencePointHandle, torsoConfigurationHandle, "Reference")) { return false; }


  std::string referenceTargetPoint = "";
  if (!tinyxml_tools::loadParameter(referenceTargetPoint, referencePointHandle, "target_point", std::string{"WBCOM"})) { return false; }

  loco::TorsoStateDesired::TargetPoint target_point = loco::TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ;
  if (referenceTargetPoint == "WBCOM") {
    MELO_DEBUG_STREAM("Setting target point to WBCOM.");
    target_point = loco::TorsoStateDesired::TargetPoint::WBCOM;
  } else if (referenceTargetPoint == "WBCOMXY_BASEZ") {
    MELO_DEBUG_STREAM("Setting target point to WBCOMXY_BASEZ.");
    target_point = loco::TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ;
  } else if (referenceTargetPoint == "BASE") {
    MELO_DEBUG_STREAM("Setting target point to BASE.");
    target_point = loco::TorsoStateDesired::TargetPoint::BASE;
  } else if (referenceTargetPoint == "TORSOCOM") {
    MELO_DEBUG_STREAM("Setting target point to TORSO_COM.");
    target_point = loco::TorsoStateDesired::TargetPoint::TORSOCOM;
  } else {
    return false;
  }
  torso_.getDesiredStatePtr()->setTargetPoint(target_point);

  if (!comControl_.loadParameters(handle)) { return false; }

  return true;
}


inline ComSupportControlZmp& TorsoControlZmp::getComControlRef() {
  return dynamic_cast<ComSupportControlZmp&>(comControl_);
}

} /* namespace loco */
