/*
 * TorsoControlDynamicGaitFreePlane.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/torso_control/TorsoControlDynamicGaitFreePlane.hpp"
#include "loco/torso_control/ComSupportControlDynamicGait.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// stl
#include <exception>

namespace loco {

TorsoControlDynamicGaitFreePlane::TorsoControlDynamicGaitFreePlane(WholeBody& wholeBody, TerrainModelBase& terrain,
                                                                   ComSupportControlBase& comSupportControl)
    : TorsoControlGaitContainer(wholeBody, terrain, comSupportControl),
      wholeBody_(wholeBody),
      lateralOffsetInControlFrameGain_(80.0),
      angularOffsetInControlFrameGain_(0.05),
      maxDesiredPitchRadians_(5.0 * M_PI / 180.0),
      desiredPitchSlope_(1.0),
      maxDesiredRollRadians_(5.0 * M_PI / 180.0),
      desiredRollSlope_(1.0),
      adaptToTerrain_(AdaptToTerrain::CompleteAdaption) {}

bool TorsoControlDynamicGaitFreePlane::initialize(double dt) {
  if (!Base::initialize(dt)) {
    return false;
  }
  torso_.getDesiredStatePtr()->setTargetPoint(TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ);

  return true;
}

bool TorsoControlDynamicGaitFreePlane::advance(double dt) {
  // compute the desired torso/wholebody position on the terrain
  comControl_.advance(dt);

  // Get measured orientation
  const RotationQuaternion& orientationWorldToControl =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();  // --> current heading orientation
  const RotationQuaternion& orientationControlToBase = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();

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

  const RotationQuaternion orientationWorldToTerrain = getOrientationWorldToHeadingOnTerrainSurface(RotationQuaternion());
  const Position positionWorldToDesiredHeightAboveTerrainInTerrainFrame(0.0, 0.0, desiredTorsoCoMHeightAboveGroundInControlFrameOffset_);
  const Position positionWorldToDesiredHeightAboveTerrainInWorldFrame =
      orientationWorldToTerrain.inverseRotate(positionWorldToDesiredHeightAboveTerrainInTerrainFrame);

  loco::Vector surfaceNormalInWorldFrame;
  terrain_.getNormal(loco::Position(), surfaceNormalInWorldFrame);

  double heightOverTerrain = positionWorldToDesiredHeightAboveTerrainInWorldFrame.dot(surfaceNormalInWorldFrame);
  heightOverTerrain /= surfaceNormalInWorldFrame.z();

  double heightOfTerrainInWorldFrame = 0.0;
  terrain_.getHeight(positionWorldToDesiredHorizontalBaseInWorldFrame, heightOfTerrainInWorldFrame);

  const Position positionControlToTargetBaseInWorldFrame =
      positionHorizontalControlToHorizontalBaseInWorldFrame + (heightOfTerrainInWorldFrame + heightOverTerrain) * Position::UnitZ();

  // this is the position we need to compute:
  Position positionControlToTargetBaseInControlFrame = orientationWorldToControl.rotate(
      positionControlToTargetBaseInWorldFrame + torso_.getDesiredState().getDesiredPositionOffsetInWorldFrame());

  // Add a desired velocity dependent contribution to the desired center of mass x-y position.
  const Eigen::Vector3d offsetGains = lateralOffsetInControlFrameGain_ * Eigen::Vector3d::UnitY();
  const Position lateralOffsetInControlFrame(
      offsetGains.cwiseProduct(torso_.getDesiredState().getLinearVelocityTargetInControlFrame().toImplementation()) * dt);
  const Position angularOffsetInControlFrame(-1.0 * angularOffsetInControlFrameGain_ *
                                             torso_.getDesiredState().getLinearVelocityTargetInControlFrame().cross(
                                                 torso_.getDesiredState().getAngularVelocityBaseInControlFrame()));
  positionControlToTargetBaseInControlFrame += lateralOffsetInControlFrame + angularOffsetInControlFrame;

  /********************************************************************************************************
   * End set desired CoM position in world frame *
   ********************************************************************************************************/

  // Set the desired orientation of the base frame with respect to the control frame.
  const RotationQuaternion orientationCurrentHeadingToDesiredHeading = getOrientationBaseToFootprint();
  const RotationQuaternion orientationControlToDesiredBase =
      torso_.getDesiredState().getDesiredOrientationOffset() * orientationCurrentHeadingToDesiredHeading;

  // Set control references.
  torso_.getDesiredStatePtr()->setPositionControlToTargetInControlFrame(positionControlToTargetBaseInControlFrame);
  torso_.getDesiredStatePtr()->setPositionWorldToBaseInWorldFrame(
      torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame() +
      orientationWorldToControl.inverseRotate(positionControlToTargetBaseInControlFrame));
  torso_.getDesiredStatePtr()->setOrientationControlToBase(orientationControlToDesiredBase);
  torso_.getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(torso_.getDesiredState().getLinearVelocityTargetInControlFrame());

  // set computed control errors
  Position positionErrorInControlFrame;
  getMixedComBasePositionError(wholeBody_, positionErrorInControlFrame,
                               torso_.getDesiredState().getPositionControlToTargetInControlFrame());
  //  getBasePositionError(wholeBody_, positionErrorInControlFrame, torso_.getDesiredState().getPositionControlToTargetInControlFrame());
  torso_.getDesiredStatePtr()->setPositionErrorInControlFrame(positionErrorInControlFrame);

  LinearVelocity linearVelocityErrorInControlFrame;
  //  getMixedComBaseLinearVelocityError(wholeBody_, linearVelocityErrorInControlFrame,
  //  torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
  getBaseLinearVelocityError(wholeBody_, linearVelocityErrorInControlFrame,
                             torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
  torso_.getDesiredStatePtr()->setLinearVelocityErrorInControlFrame(linearVelocityErrorInControlFrame);

  // set desired orientation for logging
  loco::EulerAnglesZyx desiredOrientationEulerAnglesZyxBaseToWorld(
      (torso_.getDesiredState().getOrientationControlToBase() * torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl())
          .inverted());
  desiredOrientationEulerAnglesZyxBaseToWorld.setUnique();
  torso_.getDesiredStatePtr()->setOrientationEulerAnglesZyxBaseToWorld(desiredOrientationEulerAnglesZyxBaseToWorld);

  // @todo: switch to mixed
  torso_.getDesiredStatePtr()->setTargetPoint(TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ);

  return true;
}

bool TorsoControlDynamicGaitFreePlane::loadParameters(const TiXmlHandle& handle) {
  if (!Base::loadParameters(handle)) {
    return false;
  }

  TiXmlHandle handleDynamicGait(handle.FirstChild("TorsoControl").FirstChild("DynamicGait"));

  if (!comControl_.loadParameters(handleDynamicGait)) {
    return false;
  }

  TiXmlElement* pElem;
  pElem = handleDynamicGait.FirstChild("BaseControl").FirstChild("VelocityDependentTerms").Element();
  if (pElem == nullptr) {
    printf("Could not find TorsoControl:DynamicGait:BaseControl:VelocityDependentTerms!\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("lateralOffsetInControlFrameGain", &this->lateralOffsetInControlFrameGain_) != TIXML_SUCCESS) {
    printf("Could not find TorsoControl:DynamicGait:BaseControl:lateralOffsetInControlFrameGain!\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("angularOffsetInControlFrameGain", &this->angularOffsetInControlFrameGain_) != TIXML_SUCCESS) {
    printf("Could not find TorsoControl:DynamicGait:BaseControl:angularOffsetInControlFrameGain!\n");
    return false;
  }

  return true;
}

bool TorsoControlDynamicGaitFreePlane::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2,
                                                         double t) {
  const auto& controller1 = dynamic_cast<const TorsoControlDynamicGaitFreePlane&>(torsoController1);
  const auto& controller2 = dynamic_cast<const TorsoControlDynamicGaitFreePlane&>(torsoController2);

  this->comControl_.setToInterpolated(controller1.getComSupportControl(), controller2.getComSupportControl(), t);

  desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ =
      robot_utils::linearlyInterpolate(controller1.getDesiredTorsoCoMHeightAboveGroundInControlFrameOffset(),
                                       controller2.getDesiredTorsoCoMHeightAboveGroundInControlFrameOffset(), 0.0, 1.0, t);

  return true;
}

const WholeBody& TorsoControlDynamicGaitFreePlane::getWholeBody() const {
  return wholeBody_;
}

WholeBody* TorsoControlDynamicGaitFreePlane::getWholeBodyPtr() const {
  return &wholeBody_;
}

} /* namespace loco */
