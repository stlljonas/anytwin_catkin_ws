/*
 * FootholdGeneratorInvertedPendulum.cpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso, Christian Gehring
 */

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// loco
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulum.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace loco {

FootholdGeneratorInvertedPendulum::FootholdGeneratorInvertedPendulum(WholeBody& wholeBody, TerrainModelBase& terrain)
    : FootholdGeneratorInvertedPendulumBase(wholeBody, terrain) {}

bool FootholdGeneratorInvertedPendulum::loadParameters(const TiXmlHandle& handle) {
  MELO_INFO_STREAM("[FootholdGeneratorInvertedPendulum::loadParameters] Loading parameters for foothold generator inverted pendulum.")

  TiXmlHandle fpsHandle = handle;
  if (!tinyxml_tools::getChildHandle(fpsHandle, handle, "FootPlacementStrategy")) {
    return false;
  }

  TiXmlHandle fgHandle = handle;
  if (!tinyxml_tools::getChildHandle(fgHandle, fpsHandle, "FootholdGenerator")) {
    return false;
  }

  TiXmlHandle ipHandle = handle;
  if (!tinyxml_tools::getChildHandle(ipHandle, fgHandle, "InvertedPendulum")) {
    return false;
  }

  TiXmlHandle offsetHandle = handle;
  if (!tinyxml_tools::getChildHandle(offsetHandle, ipHandle, "Offset")) {
    return false;
  }

  TiXmlHandle gainsHandle = handle;
  if (!tinyxml_tools::getChildHandle(gainsHandle, ipHandle, "Gains")) {
    return false;
  }

  TiXmlHandle foreHandle = handle;
  if (!tinyxml_tools::getChildHandle(foreHandle, offsetHandle, "Fore")) {
    return false;
  }

  TiXmlHandle hindHandle = handle;
  if (!tinyxml_tools::getChildHandle(hindHandle, offsetHandle, "Hind")) {
    return false;
  }

  if (!tinyxml_tools::loadParameter(stepFeedbackScale_, gainsHandle, "feedbackScale")) {
    return false;
  }

  double offsetHeadingFore = 0.0;
  if (!tinyxml_tools::loadParameter(offsetHeadingFore, foreHandle, "heading")) {
    return false;
  }

  double offsetLateralFore = 0.0;
  if (!tinyxml_tools::loadParameter(offsetLateralFore, foreHandle, "lateral")) {
    return false;
  }
  // fixme: remove get
  legs_.getPtr(0)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      Position(offsetHeadingFore, offsetLateralFore, 0.0));
  legs_.getPtr(1)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      Position(offsetHeadingFore, -offsetLateralFore, 0.0));

  double offsetHeadingHind = 0.0;
  if (!tinyxml_tools::loadParameter(offsetHeadingHind, hindHandle, "heading")) {
    return false;
  }

  double offsetLateralHind = 0.0;
  if (!tinyxml_tools::loadParameter(offsetLateralHind, hindHandle, "lateral")) {
    return false;
  }
  // fixme: remove get
  legs_.getPtr(2)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      Position(offsetHeadingHind, offsetLateralHind, 0.0));
  legs_.getPtr(3)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      Position(offsetHeadingHind, -offsetLateralHind, 0.0));

  MELO_INFO_STREAM("[FootholdGeneratorInvertedPendulum::loadParameters] Loaded offset parameters:")
  MELO_INFO_STREAM("Left fore leg offset:  " << legs_.get(0).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame())
  MELO_INFO_STREAM("Right fore leg offset: " << legs_.get(1).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame())
  MELO_INFO_STREAM("Left hind leg offset:  " << legs_.get(2).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame())
  MELO_INFO_STREAM("Right hind leg offset: " << legs_.get(3).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame())
  MELO_INFO_STREAM("[FootholdGeneratorInvertedPendulum::loadParameters] Done loading parameters.")

  return true;
}

Position FootholdGeneratorInvertedPendulum::computeWorldToFootholdInWorldFrame(int legId) {
  // fixme: remove get
  LegBase* leg = legs_.getPtr(legId);

  // Find starting point: hip projected vertically on ground
  Position positionWorldToHipOnPlaneAlongNormalInWorldFrame = leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame();
  terrain_.getHeight(positionWorldToHipOnPlaneAlongNormalInWorldFrame);

  //--- Evaluate the inverted pendulum (feedback) and velocity (feed-forward) contributions
  const Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame =
      getPositionReferenceToDesiredFootOnTerrainInWorldFrame(*leg);
  //---

  const RotationQuaternion& orientationWorldToBase = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase();
  Position positionBaseToCenterOfMassInWorldFrame =
      orientationWorldToBase.inverseRotate(wholeBody_.getTorso().getProperties().getBaseToCenterOfMassPositionInBaseFrame());
  terrain_.getHeight(positionBaseToCenterOfMassInWorldFrame);

  //--- Build the desired foothold and return it
  Position positionWorldToFootholdInWorldFrame =
      positionWorldToHipOnPlaneAlongNormalInWorldFrame  // starting point, hip projected on the plane along world z axis
      + positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame + positionBaseToCenterOfMassInWorldFrame;  // x-y
  terrain_.getHeight(positionWorldToFootholdInWorldFrame);
  //---

  leg->getFootPtr()->getStateDesiredPtr()->setPositionWorldToFootholdInWorldFrame(positionWorldToFootholdInWorldFrame);

  return positionWorldToFootholdInWorldFrame;
}

Position FootholdGeneratorInvertedPendulum::evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg) {
  // Get default offset
  const Position& positionDesiredFootOnTerrainDefaultOffsetInControlFrame =
      leg.getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame();

  // Build foot position and return it
  positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()] =
      stepFeedforwardScale_ *
      (positionDesiredFootOnTerrainDefaultOffsetInControlFrame + Position(computeLinearVelocityProjectionInControlFrame(leg)));
  return positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()];
}

Position FootholdGeneratorInvertedPendulum::evaluateDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg) {
  Position positionDesiredFootHoldOnTerrainFeedBackInControlFrame =
      Position(computeLinearVelocityErrorInControlFrame(leg).toImplementation()) * stepFeedbackScale_ * getFeedbackInvertedPendulum(leg);

  positionDesiredFootHoldOnTerrainFeedBackInControlFrame.z() = 0.0;  // project on terrain
  positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()] = positionDesiredFootHoldOnTerrainFeedBackInControlFrame;
  return positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()];
}

std::ostream& operator<<(std::ostream& out, const FootholdGeneratorInvertedPendulum& fhGen) {
  out << std::endl;
  out << "----------------------------------------" << std::endl;
  out << "Class: FootholdGeneratorInvertedPendulum" << std::endl;
  out << "step feedback scale: " << fhGen.getFeedbackScale() << std::endl;
  // fixme: remove get
  out << "default stepping position: " << std::endl
      << std::setw(10)
      << "left fore:" << fhGen.getLegs().get(0).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame() << std::endl
      << std::setw(10)
      << "right fore:" << fhGen.getLegs().get(1).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame() << std::endl
      << std::setw(10)
      << "left hind:" << fhGen.getLegs().get(2).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame() << std::endl
      << std::setw(10)
      << "right hind:" << fhGen.getLegs().get(3).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame() << std::endl;
  out << "----------------------------------------" << std::endl;
  out << std::endl;

  return out;
}

LinearVelocity FootholdGeneratorInvertedPendulum::computeLinearVelocityProjectionInControlFrame(const LegBase& leg) const {
  const double stanceDuration = leg.getContactSchedule().getStanceDuration();
  const auto orientationBaseToControl = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase().inverted();

  // Evaluate feed-forward component
  // ToDo (avijayan): The 1.5x multiplication factor in the foothold computation is worth investigating.
  const auto linearVelocityProjectionInControlFrame =
      LinearVelocity(torso_.getDesiredState().getLinearVelocityTargetInControlFrame().toImplementation().cwiseProduct(
                         (1.5 * Position::UnitX().toImplementation() + Position::UnitY().toImplementation())) +
                     torso_.getDesiredState().getAngularVelocityBaseInControlFrame().toImplementation().cross(
                         orientationBaseToControl.rotate(leg.getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame())
                             .toImplementation())) *
      stanceDuration / 2.0;

  //  const auto linearVelocityProjectionInControlFrame = LinearVelocity(
  //        torso_.getDesiredState().getLinearVelocityTargetInControlFrame().toImplementation().cwiseProduct(Position::UnitX().toImplementation())
  //               * stanceDuration);

  return linearVelocityProjectionInControlFrame;
}

LinearVelocity FootholdGeneratorInvertedPendulum::computeLinearVelocityErrorInControlFrame(const LegBase& leg) const {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const RotationQuaternion& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();

  //--- Get desired velocity heading component in control frame
  const LinearVelocity linearVelocityDesiredInWorldFrame =
      orientationWorldToControl.inverseRotate(torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
  //---

  //--- Get reference velocity estimation
  const LinearVelocity linearVelocityReferenceInWorldFrame =
      (leg.getLimbStateMeasured().getLinearVelocityLimbBaseInWorldFrame() +
       orientationWorldToBase.inverseRotate(torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame())) /
      2.0;
  //---

  const LinearVelocity linearVelocityErrorInWorldFrame = linearVelocityReferenceInWorldFrame - linearVelocityDesiredInWorldFrame;
  return orientationWorldToControl.rotate(linearVelocityErrorInWorldFrame);
}

} /* namespace loco */
