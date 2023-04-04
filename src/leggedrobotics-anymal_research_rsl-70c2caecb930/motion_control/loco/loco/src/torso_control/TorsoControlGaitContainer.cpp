/*
 * TorsoControlGaitContainer.cpp
 *
 *  Created on: Oct 29, 2014
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/torso_control/TorsoControlGaitContainer.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// stl
#include <exception>

namespace loco {

TorsoControlGaitContainer::TorsoControlGaitContainer(WholeBody& wholeBody, TerrainModelBase& terrain, ComSupportControlBase& comControl)
    : TorsoControlBase(),
      wholeBody_(wholeBody),
      legs_(*wholeBody.getLegsPtr()),
      torso_(*wholeBody.getTorsoPtr()),
      terrain_(terrain),
      comControl_(comControl),
      desiredTorsoCoMHeightAboveGroundInControlFrameOffset_(0.4) {}

bool TorsoControlGaitContainer::initialize(double dt) {
  if (!comControl_.initialize(dt)) {
    return false;
  }

  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const RotationQuaternion& orientationControlToBase = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();

  torso_.getDesiredStatePtr()->setPositionControlToTargetInControlFrame(
      orientationWorldToControl.rotate(torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame() -
                                       torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame()));

  torso_.getDesiredStatePtr()->setPositionWorldToBaseInWorldFrame(torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame());
  torso_.getDesiredStatePtr()->setOrientationControlToBase(torso_.getMeasuredState().inControlFrame().getOrientationControlToBase());

  torso_.getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(
      orientationControlToBase.inverseRotate(torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame()));

  // Set control errors.
  torso_.getDesiredStatePtr()->setPositionErrorInControlFrame(loco::Position());
  torso_.getDesiredStatePtr()->setLinearVelocityErrorInControlFrame(loco::LinearVelocity());

  // set desired orientation for logging
  loco::EulerAnglesZyx desiredOrientationEulerAnglesZyxBaseToWorld(
      (torso_.getDesiredState().getOrientationControlToBase() * torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl())
          .inverted());
  desiredOrientationEulerAnglesZyxBaseToWorld.setUnique();
  torso_.getDesiredStatePtr()->setOrientationEulerAnglesZyxBaseToWorld(
      loco::EulerAnglesZyx(torso_.getMeasuredState().getOrientationWorldToBase().inverted()));

  return true;
}

double TorsoControlGaitContainer::getDesiredTorsoCoMHeightAboveGroundInControlFrameOffset() const {
  return desiredTorsoCoMHeightAboveGroundInControlFrameOffset_;
}

ComSupportControlBase* TorsoControlGaitContainer::getComSupportControlPtr() {
  return &comControl_;
}

const ComSupportControlBase& TorsoControlGaitContainer::getComSupportControl() const {
  return comControl_;
}

bool TorsoControlGaitContainer::loadParameters(const TiXmlHandle& handle) {
  // Get handles.
  TiXmlHandle torsoControlHandle = handle;
  if (!tinyxml_tools::getChildHandle(torsoControlHandle, handle, "TorsoControl")) {
    return false;
  }

  TiXmlHandle torsoConfigurationHandle = handle;
  if (!tinyxml_tools::getChildHandle(torsoConfigurationHandle, torsoControlHandle, "TorsoConfiguration")) {
    return false;
  }

  TiXmlHandle torsoHeightHandle = handle;
  if (!tinyxml_tools::getChildHandle(torsoHeightHandle, torsoConfigurationHandle, "TorsoHeight")) {
    return false;
  }

  TiXmlHandle referencePointHandle = handle;
  if (!tinyxml_tools::getChildHandle(referencePointHandle, torsoConfigurationHandle, "Reference")) {
    return false;
  }

  // Load parameters.
  if (!tinyxml_tools::loadParameter(desiredTorsoCoMHeightAboveGroundInControlFrameOffset_, torsoHeightHandle, "torsoHeight")) {
    return false;
  }

  std::string referenceTargetPoint;
  if (!tinyxml_tools::loadParameter(referenceTargetPoint, referencePointHandle, "target_point", std::string{"WBCOM"})) {
    return false;
  }

  loco::TorsoStateDesired::TargetPoint target_point = loco::TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ;
  if (referenceTargetPoint == "WBCOM") {
    MELO_DEBUG_STREAM("[TorsoControlGaitContainer::loadParameters] Setting target point to WBCOM.")
    target_point = loco::TorsoStateDesired::TargetPoint::WBCOM;
  } else if (referenceTargetPoint == "WBCOMXY_BASEZ") {
    MELO_DEBUG_STREAM("[TorsoControlGaitContainer::loadParameters] Setting target point to WBCOMXY_BASEZ.")
    target_point = loco::TorsoStateDesired::TargetPoint::WBCOMXY_BASEZ;
  } else if (referenceTargetPoint == "BASE") {
    MELO_DEBUG_STREAM("[TorsoControlGaitContainer::loadParameters] Setting target point to BASE.")
    target_point = loco::TorsoStateDesired::TargetPoint::BASE;
  } else if (referenceTargetPoint == "TORSOCOM") {
    MELO_DEBUG_STREAM("[TorsoControlGaitContainer::loadParameters] Setting target point to TORSO_COM.")
    target_point = loco::TorsoStateDesired::TargetPoint::TORSOCOM;
  } else {
    return false;
  }
  torso_.getDesiredStatePtr()->setTargetPoint(target_point);

  // Initialize and add parameters to the parameter handler.
  paramTorsoHeightAboveGround_.setValue(desiredTorsoCoMHeightAboveGroundInControlFrameOffset_);
  paramTorsoHeightAboveGround_.setDefaultValue(desiredTorsoCoMHeightAboveGroundInControlFrameOffset_);
  paramTorsoHeightAboveGround_.setMinValue(0.20);
  // fixme: max height should be set higher
  paramTorsoHeightAboveGround_.setMaxValue(0.50);

  return true;
}

RotationQuaternion TorsoControlGaitContainer::getOrientationWorldToHeadingOnTerrainSurface(
    const RotationQuaternion& orientationWorldToHeading) const {
  loco::Vector normalInWorldFrame;
  terrain_.getNormal(loco::Position::Zero(), normalInWorldFrame);

  const loco::Vector normalInHeadingControlFrame = orientationWorldToHeading.rotate(normalInWorldFrame);
  const double terrainPitch = atan2(normalInHeadingControlFrame.x(), normalInHeadingControlFrame.z());
  const double terrainRoll = atan2(normalInHeadingControlFrame.y(), normalInHeadingControlFrame.z());

  return RotationQuaternion(AngleAxis(terrainRoll, -1.0, 0.0, 0.0)) * RotationQuaternion(AngleAxis(terrainPitch, 0.0, 1.0, 0.0)) *
         orientationWorldToHeading;
}

RotationQuaternion TorsoControlGaitContainer::getOrientationBaseToFootprint() const {
  //--- Get desired heading direction with respect to the current feet
  // fixme replace get
  const Position positionForeFeetMidPointInWorldFrame =
      (legs_.get(0).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() +
       legs_.get(1).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()) *
      0.5;
  const Position positionHindFeetMidPointInWorldFrame =
      (legs_.get(2).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() +
       legs_.get(3).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()) *
      0.5;
  Vector footprintHeadingDirection(positionForeFeetMidPointInWorldFrame - positionHindFeetMidPointInWorldFrame);
  footprintHeadingDirection.z() = 0.0;
  //---

  //--- Get current heading direction defined by the mid hip points
  // fixme replace get
  const Position positionForeHipsMidPointInWorldFrame = (legs_.get(0).getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame() +
                                                         legs_.get(1).getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame()) *
                                                        0.5;
  const Position positionHindHipsMidPointInWorldFrame = (legs_.get(2).getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame() +
                                                         legs_.get(3).getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame()) *
                                                        0.5;
  Vector baseHeadingDirection = Vector(positionForeHipsMidPointInWorldFrame - positionHindHipsMidPointInWorldFrame);
  baseHeadingDirection.z() = 0.0;
  //---

  // Yaw angle in world frame
  RotationQuaternion orientationBaseToFootprint;

  try {
    orientationBaseToFootprint.setFromVectors(footprintHeadingDirection.vector(), baseHeadingDirection.vector());
  } catch (std::exception& e) {
    MELO_WARN_STREAM(e.what() << "baseHeadingDirection: " << baseHeadingDirection
                              << " footprintHeadingDirection: " << footprintHeadingDirection << std::endl);
    orientationBaseToFootprint.setIdentity();
  }

  return orientationBaseToFootprint;
}

void TorsoControlGaitContainer::addVariablesToLog(const std::string& ns) {
  signal_logger::add(desiredTorsoCoMHeightAboveGroundInControlFrameOffset_, "desiredTorsoCoMHeightAboveGroundInControlFrameOffset", ns,
                     "m");
  signal_logger::add(positionControlToTargetBaseInWorldFrame_, "positionControlToTargetBaseInWorldFrame", ns, "m");
  signal_logger::add(linearVelocityTargetBaseInBaseFrame_, "linearVelocityTargetBaseInBaseFrame", ns, "m/s");
  TorsoControlBase::addVariablesToLog(ns);
}

bool TorsoControlGaitContainer::addParametersToHandler(const std::string& ns) {
  bool success = true;
  success = parameter_handler::handler->addParam(ns + "/torsoHeight", paramTorsoHeightAboveGround_) && success;
  success = comControl_.addParametersToHandler(ns) && success;
  return success;
}

bool TorsoControlGaitContainer::removeParametersFromHandler() {
  bool success = true;
  success = parameter_handler::handler->removeParam(paramTorsoHeightAboveGround_.getName()) && success;
  success = comControl_.removeParametersFromHandler() && success;
  return success;
}

} /* namespace loco */
