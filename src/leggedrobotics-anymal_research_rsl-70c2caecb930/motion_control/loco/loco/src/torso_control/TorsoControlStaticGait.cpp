/*
 * TorsoControlStaticGait.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/torso_control/TorsoControlStaticGait.hpp"
#include "loco/torso_control/ComSupportControlStaticGait.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

namespace loco {

TorsoControlStaticGait::TorsoControlStaticGait(WholeBody& wholeBody, TerrainModelBase& terrain, GaitPatternBase& gaitPattern,
                                               ComSupportControlBase& comControl)
    : TorsoControlGaitContainer(wholeBody, terrain, comControl), gaitPattern_(gaitPattern) {}

bool TorsoControlStaticGait::initialize(double dt) {
  if (!Base::initialize(dt)) {
    return false;
  }
  torso_.getDesiredStatePtr()->setTargetPoint(TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ);

  return true;
}

bool TorsoControlStaticGait::advance(double dt) {
  bool success = comControl_.advance(dt);

  const RotationQuaternion& orientationWorldToControl =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();  // --> current heading orientation
  const RotationQuaternion& orientationControlToBase = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();
  const RotationQuaternion orientationWorldToTerrain = getOrientationWorldToHeadingOnTerrainSurface(RotationQuaternion());

  /********************************************************************************************************
   * Set desired base position in world frame
   ********************************************************************************************************/

  /* Compute the horizontal component of the desired position in world frame.
   *
   *  evaluate desired CoM position in control frame
   */
  const Position& positionWorldToDesiredHorizontalBaseInWorldFrame = comControl_.getPositionWorldToDesiredCoMInWorldFrame();

  // this is the desired location of the base location relative to the origin of the control frame projected on the x-y plane of the world
  // frame and expressed in the world frame
  Position positionHorizontalControlToHorizontalBaseInWorldFrame =
      positionWorldToDesiredHorizontalBaseInWorldFrame - torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame();
  positionHorizontalControlToHorizontalBaseInWorldFrame.z() = 0.0;

  const Position positionWorldToDesiredHeightAboveTerrainInTerrainFrame(0.0, 0.0, paramTorsoHeightAboveGround_.getValue());
  const Position positionWorldToDesiredHeightAboveTerrainInWorldFrame =
      orientationWorldToTerrain.inverseRotate(positionWorldToDesiredHeightAboveTerrainInTerrainFrame);

  loco::Vector surfaceNormalInWorldFrame;
  terrain_.getNormal(positionWorldToDesiredHeightAboveTerrainInWorldFrame, surfaceNormalInWorldFrame);
  double heightOverTerrain = positionWorldToDesiredHeightAboveTerrainInWorldFrame.dot(surfaceNormalInWorldFrame);
  heightOverTerrain /= surfaceNormalInWorldFrame.z();

  double heightOfTerrainInWorldFrame = 0.0;
  terrain_.getHeight(positionWorldToDesiredHorizontalBaseInWorldFrame, heightOfTerrainInWorldFrame);

  //  Position positionWorldToHorizontalBaseInWorldFrame_temp = positionWorldToDesiredHorizontalBaseInWorldFrame +
  //  heightOfTerrainInWorldFrame*Position::UnitZ();

  const Position positionControlToTargetBaseInWorldFrame =
      positionHorizontalControlToHorizontalBaseInWorldFrame + (heightOfTerrainInWorldFrame + heightOverTerrain) * Position::UnitZ();

  // store desired base position for logging
  positionControlToTargetBaseInWorldFrame_ = loco::Vector(positionControlToTargetBaseInWorldFrame);

  const Position positionControlToTargetBaseInControlFrame = orientationWorldToControl.rotate(
      positionControlToTargetBaseInWorldFrame + torso_.getDesiredState().getDesiredPositionOffsetInWorldFrame());

  /********************************************************************************************************
   * End set desired CoM position in world frame *
   ********************************************************************************************************/

  /********************************************************************************************************
   * Set the desired orientation of the base frame with respect to the control frame
   ********************************************************************************************************/
  // this is the orientation we need to compute
  const RotationQuaternion orientationControlToDesiredBase = torso_.getDesiredState().getDesiredOrientationOffset();
  //---

  // set control references
  torso_.getDesiredStatePtr()->setPositionControlToTargetInControlFrame(positionControlToTargetBaseInControlFrame);
  torso_.getDesiredStatePtr()->setPositionWorldToBaseInWorldFrame(
      torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame() +
      orientationWorldToControl.inverseRotate(positionControlToTargetBaseInControlFrame));
  torso_.getDesiredStatePtr()->setOrientationControlToBase(orientationControlToDesiredBase);

  const LinearVelocity linearVelocityBaseInControlFrame =
      orientationWorldToControl.rotate(comControl_.getLinearVelocityDesiredBaseInWorldFrame());
  torso_.getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(linearVelocityBaseInControlFrame);
  const LocalAngularVelocity angularVelocityBaseInControlFrame;
  torso_.getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(angularVelocityBaseInControlFrame);

  // set computed control errors
  Position positionErrorInControlFrame;
  getMixedComBasePositionError(wholeBody_, positionErrorInControlFrame,
                               torso_.getDesiredState().getPositionControlToTargetInControlFrame());
  //  getBasePositionError(wholeBody_, positionErrorInControlFrame, torso_.getDesiredState().getPositionControlToBaseInControlFrame());
  torso_.getDesiredStatePtr()->setPositionErrorInControlFrame(positionErrorInControlFrame);

  LinearVelocity linearVelocityErrorInControlFrame;
  //  getMixedComBaseLinearVelocityError(wholeBody_, linearVelocityErrorInControlFrame,
  //  torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
  getBaseLinearVelocityError(wholeBody_, linearVelocityErrorInControlFrame,
                             torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
  torso_.getDesiredStatePtr()->setLinearVelocityErrorInControlFrame(linearVelocityErrorInControlFrame);

  // store desired velocity for logging
  linearVelocityTargetBaseInBaseFrame_ = orientationControlToBase.rotate(loco::Vector(linearVelocityBaseInControlFrame));

  // set desired orientation for logging
  loco::EulerAnglesZyx desiredOrientationEulerAnglesZyxBaseToWorld(
      (torso_.getDesiredState().getOrientationControlToBase() * torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl())
          .inverted());
  desiredOrientationEulerAnglesZyxBaseToWorld.setUnique();
  torso_.getDesiredStatePtr()->setOrientationEulerAnglesZyxBaseToWorld(desiredOrientationEulerAnglesZyxBaseToWorld);

  // Update desired CoM position in whole body module.
  wholeBody_.getWholeBodyStateDesiredPtr()->setPositionWorldToWholeBodyCenterOfMassInWorldFrame(
      torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame() +
      orientationWorldToControl.inverseRotate(torso_.getDesiredState().getPositionControlToTargetInControlFrame()));

  return success;
}

void TorsoControlStaticGait::setMainBodyDesiredHeightFromTerrain(double height) {
  desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ = height;
  robot_utils::boundToRange(&desiredTorsoCoMHeightAboveGroundInControlFrameOffset_, 0.42, 0.51);
}

bool TorsoControlStaticGait::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2,
                                               double t) {
  return false;
}

void TorsoControlStaticGait::setIsInStandConfiguration(bool isInStandConfiguration) {
  dynamic_cast<ComSupportControlStaticGait&>(comControl_).setIsInStandConfiguration(isInStandConfiguration);
}

bool TorsoControlStaticGait::addVariablesToLog(bool update) {
  return true;
}

bool TorsoControlStaticGait::loadParameters(const TiXmlHandle& handle) {
  if (!Base::loadParameters(handle)) {
    return false;
  }
  if (!dynamic_cast<ComSupportControlStaticGait&>(comControl_).loadParameters(handle)) {
    return false;
  }

  return true;
}

} /* namespace loco */
