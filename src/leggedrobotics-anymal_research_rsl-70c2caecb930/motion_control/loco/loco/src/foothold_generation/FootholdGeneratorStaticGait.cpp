/*
 * FootholdGeneratorStaticGait.cpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/foothold_generation/FootholdGeneratorStaticGait.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

namespace color = message_logger::color;

namespace loco {

FootholdGeneratorStaticGait::FootholdGeneratorStaticGait(WholeBody& wholeBody, TerrainModelBase& terrain)
    : FootholdGeneratorBase(),
      wholeBody_(wholeBody),
      terrain_(terrain),
      torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      positionFootprintCenterToDefaultFootholdInControlFrame_(legs_.size()),
      ellipsoid_(0.0, 0.0, 0.0) {
  for (auto leg : legs_) {
    positionFootprintCenterToDefaultFootholdInControlFrame_[leg->getId()].setZero();
  }
}

bool FootholdGeneratorStaticGait::initialize(double /*dt*/) {
  /*************************************
   * Set default desired foot position *
   *************************************/
  // fixme: remove get
  const double halfHipToHipSide = legs_.get(0).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame().x();
  const double halfHipToHipFront = legs_.get(0).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame().y();

  positionFootprintCenterToDefaultFootholdInControlFrame_[0] =
      Position(halfHipToHipSide + paramFootholdOffsetX_.getValue(), halfHipToHipFront + paramFootholdOffsetY_.getValue(), 0.0);
  positionFootprintCenterToDefaultFootholdInControlFrame_[1] =
      Position(halfHipToHipSide + paramFootholdOffsetX_.getValue(), -halfHipToHipFront - paramFootholdOffsetY_.getValue(), 0.0);
  positionFootprintCenterToDefaultFootholdInControlFrame_[2] =
      Position(-halfHipToHipSide - paramFootholdOffsetX_.getValue(), halfHipToHipFront + paramFootholdOffsetY_.getValue(), 0.0);
  positionFootprintCenterToDefaultFootholdInControlFrame_[3] =
      Position(-halfHipToHipSide - paramFootholdOffsetX_.getValue(), -halfHipToHipFront - paramFootholdOffsetY_.getValue(), 0.0);
  /*************************************/

  // Reset parameters to their default values.
  paramFootholdOffsetX_.resetToDefault();
  paramFootholdOffsetY_.resetToDefault();
  paramMaxStepLengthHeading_.resetToDefault();
  paramMaxStepLengthLateral_.resetToDefault();
  paramMaxStepAngular_.resetToDefault();
  paramMaxDistanceForeHind_.resetToDefault();
  paramMaxDistanceLeftRight_.resetToDefault();
  paramMaxDistanceDiagonal_.resetToDefault();
  paramMinDistanceForeHind_.resetToDefault();
  paramMinDistanceLeftRight_.resetToDefault();
  paramMinDistanceDiagonal_.resetToDefault();

  // fixme: remove this call
  addVariablesToLog();

  return true;
}

bool FootholdGeneratorStaticGait::addVariablesToLog() {
  return true;
}

bool FootholdGeneratorStaticGait::loadParameters(const TiXmlHandle& handle) {
  const TiXmlHandle fpsHandle = tinyxml_tools::getChildHandle(handle, "FootPlacementStrategy");
  const TiXmlHandle fgHandle = tinyxml_tools::getChildHandle(fpsHandle, "FootholdGenerator");
  const TiXmlHandle sgHandle = tinyxml_tools::getChildHandle(fgHandle, "StaticGait");
  const TiXmlHandle footholdLimitsHandle = tinyxml_tools::getChildHandle(sgHandle, "FootHoldLimits");
  const TiXmlHandle legDistanceLateralHandle = tinyxml_tools::getChildHandle(sgHandle, "LegDistanceLateral");
  const TiXmlHandle legDistanceLongitudinalHandle = tinyxml_tools::getChildHandle(sgHandle, "LegDistanceLongitudinal");
  const TiXmlHandle legDistanceDiagonalHandle = tinyxml_tools::getChildHandle(sgHandle, "LegDistanceDiagonal");
  const TiXmlHandle defaultFootholdOffsetHandle = tinyxml_tools::getChildHandle(sgHandle, "DefaultFootholdOffset");

  // Maximum step offsets.
  double maxStepLengthHeading = 0.0;
  tinyxml_tools::loadParameter(maxStepLengthHeading, footholdLimitsHandle, "heading");
  double maxStepLengthLateral = 0.0;
  tinyxml_tools::loadParameter(maxStepLengthLateral, footholdLimitsHandle, "lateral");
  double maxStepAngular = 0.0;
  tinyxml_tools::loadParameter(maxStepAngular, footholdLimitsHandle, "angular");

  // Boundary distances from left to right footholds.
  double minDistanceLeftRight = 0.0;
  tinyxml_tools::loadParameter(minDistanceLeftRight, legDistanceLateralHandle, "min");
  double maxDistanceLeftRight = 0.0;
  tinyxml_tools::loadParameter(maxDistanceLeftRight, legDistanceLateralHandle, "max");

  // Boundary distances from fore to hind footholds.
  double minDistanceForeHind = 0.0;
  tinyxml_tools::loadParameter(minDistanceForeHind, legDistanceLongitudinalHandle, "min");
  double maxDistanceForeHind = 0.0;
  tinyxml_tools::loadParameter(maxDistanceForeHind, legDistanceLongitudinalHandle, "max");

  // Boundary distances from fore to hind diagonal footholds.
  double minDistanceForeHindDiagonal = 0.0;
  tinyxml_tools::loadParameter(minDistanceForeHindDiagonal, legDistanceDiagonalHandle, "min");
  double maxDistanceForeHindDiagonal = 0.0;
  tinyxml_tools::loadParameter(maxDistanceForeHindDiagonal, legDistanceDiagonalHandle, "max");

  // Default foothold locations.
  double defaultOffsetX = 0.0;
  tinyxml_tools::loadParameter(defaultOffsetX, defaultFootholdOffsetHandle, "x");
  double defaultOffsetY = 0.0;
  tinyxml_tools::loadParameter(defaultOffsetY, defaultFootholdOffsetHandle, "y");

  // Update parameters.
  paramMaxStepLengthHeading_.resetWithDefaultValueAndRelativeBounds(maxStepLengthHeading, 0.02);
  parameter_handler::handler->addParam("maxStepHeading", paramMaxStepLengthHeading_);

  paramMaxStepLengthLateral_.resetWithDefaultValueAndRelativeBounds(maxStepLengthLateral, 0.02);
  parameter_handler::handler->addParam("maxStepLateral", paramMaxStepLengthLateral_);

  paramMaxStepAngular_.resetWithDefaultValueAndRelativeBounds(maxStepAngular, 0.02);
  parameter_handler::handler->addParam("maxStepAngular", paramMaxStepAngular_);

  paramMinDistanceLeftRight_.resetWithDefaultValueAndRelativeBounds(minDistanceLeftRight, 0.02);
  parameter_handler::handler->addParam("minFeetLeftRight", paramMinDistanceLeftRight_);

  paramMaxDistanceLeftRight_.resetWithDefaultValueAndRelativeBounds(maxDistanceLeftRight, 0.02);
  parameter_handler::handler->addParam("maxFeetLeftRight", paramMaxDistanceLeftRight_);

  paramMinDistanceForeHind_.resetWithDefaultValueAndRelativeBounds(minDistanceForeHind, 0.02);
  parameter_handler::handler->addParam("minFeetForeHind", paramMinDistanceForeHind_);

  paramMaxDistanceForeHind_.resetWithDefaultValueAndRelativeBounds(maxDistanceForeHind, 0.02);
  parameter_handler::handler->addParam("maxFeetForeHind", paramMaxDistanceForeHind_);

  paramMinDistanceDiagonal_.resetWithDefaultValueAndRelativeBounds(minDistanceForeHindDiagonal, 0.02);
  parameter_handler::handler->addParam("minFeetDiagonal", paramMinDistanceDiagonal_);

  paramMaxDistanceDiagonal_.resetWithDefaultValueAndRelativeBounds(maxDistanceForeHindDiagonal, 0.02);
  parameter_handler::handler->addParam("maxFeetDiagonal", paramMaxDistanceDiagonal_);

  paramFootholdOffsetX_.resetWithDefaultValueAndRelativeBounds(defaultOffsetX, 0.02);
  parameter_handler::handler->addParam("fhOffsetX", paramFootholdOffsetX_);

  paramFootholdOffsetY_.resetWithDefaultValueAndRelativeBounds(defaultOffsetY, 0.02);
  parameter_handler::handler->addParam("fhOffsetY", paramFootholdOffsetY_);

  // Initialize ellipsoid.
  ellipsoid_.setSemiPrincipalAxisA(paramMaxStepLengthHeading_.getValue());
  ellipsoid_.setSemiPrincipalAxisB(paramMaxStepLengthLateral_.getValue());
  ellipsoid_.setSemiPrincipalAxisC(paramMaxStepAngular_.getValue());

  return true;
}

void FootholdGeneratorStaticGait::generateFootprint() {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  Position positionWorldToLastFullStanceInControlFrame;
  for (auto leg : legs_) {
    positionWorldToLastFullStanceInControlFrame +=
        orientationWorldToControl.rotate(leg->getPositionWorldToLastOrCurrentContactInWorldFrame());
  }
  if (legs_.size() > 0u) {
    positionWorldToLastFullStanceInControlFrame /= legs_.size();
  }

  Position positionWorldToBaseOnGroundInControlFrame =
      orientationWorldToControl.rotate(torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame()) +
      torso_.getMeasuredState().inControlFrame().getPositionControlToBaseInControlFrame();
  positionWorldToBaseOnGroundInControlFrame.z() = 0.0;
  //  const Position positionWorldToCenterInControlFrame = 0.2*positionWorldToLastFullStanceInControlFrame +
  //  0.8*positionWorldToBaseOnGroundInControlFrame;
  const Position positionWorldToCenterInControlFrame(positionWorldToBaseOnGroundInControlFrame.x(),
                                                     positionWorldToLastFullStanceInControlFrame.y(), 0.0);

  for (auto leg : legs_) {
    const Position positionCenterToFootInControlFrame = getPositionFootprintCenterToDesiredFootHoldInControlFrame(*leg);
    legs_.getPtr(leg->getId())
        ->getFootPtr()
        ->getStateDesiredPtr()
        ->setPositionWorldToFootholdInWorldFrame(
            orientationWorldToControl.inverseRotate(positionWorldToCenterInControlFrame + positionCenterToFootInControlFrame));
  }
}

Position FootholdGeneratorStaticGait::generateFootHold(const int legId, const Footprint& footprintInWorldFrame,
                                                       bool compareWithPlannedFootholds) {
  /*
   * Generation strategy (this is called when all four legs are in stance according to the gait pattern):
   *  1. find the average of the current foot positions (e.g. footprint center)
   *  2. add a default foot position (defined at initialization) to the average in 1.
   *  3. rotate the default foot position according to the desired angular velocity
   *  4. add an offset proportional to desired translation velocity to the position found  in 2.
   *  5. correct the final foothold height
   */
  // fixme: remove get
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const LegBase& leg = legs_.get(legId);

  // Find average of current foot positions
  Position positionWorldToCenterOfFootprintInWorldFrame;
  for (int k = 0; k < legs_.size(); k++) {
    positionWorldToCenterOfFootprintInWorldFrame += Position(footprintInWorldFrame.col(k) / legs_.size());
  }

  Position positionWorldToFootHoldInWorldFrame =
      positionWorldToCenterOfFootprintInWorldFrame +
      orientationWorldToControl.inverseRotate(getPositionFootprintCenterToDesiredFootHoldInControlFrame(leg));
  // get correct terrain height at foot hold
  terrain_.getHeight(positionWorldToFootHoldInWorldFrame);

  constrainFootholdInFootprintInWorldFrame(positionWorldToFootHoldInWorldFrame, leg, compareWithPlannedFootholds);

  return positionWorldToFootHoldInWorldFrame;
}

Position FootholdGeneratorStaticGait::generateFootHold(const int legId) {
  /*
   * Generation strategy (this is called when all four legs are in stance according to the gait pattern):
   *  1. find the average of the current foot positions (e.g. footprint center)
   *  2. add a default foot position (defined at initialization) to the average in 1.
   *  3. rotate the default foot position according to the desired angular velocity
   *  4. add an offset proportional to desired translation velocity to the position found  in 2.
   *  5. correct the final foothold height
   */
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const RotationQuaternion& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
  // fixme: remove get
  const LegBase& leg = legs_.get(legId);

  // Find average of current foot positions
  Position positionBaseToCenterOfFootprintInBaseFrame;
  for (int k = 0; k < legs_.size(); k++) {
    // fixme: remove get
    positionBaseToCenterOfFootprintInBaseFrame +=
        legs_.get(k).getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame() / legs_.size();
  }

  Position positionWorldToFootHoldInWorldFrame =
      torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame() +
      orientationWorldToBase.inverseRotate(positionBaseToCenterOfFootprintInBaseFrame) +
      orientationWorldToControl.inverseRotate(getPositionFootprintCenterToDesiredFootHoldInControlFrame(leg));
  // get correct terrain height at foot hold
  terrain_.getHeight(positionWorldToFootHoldInWorldFrame);

  constrainFootholdInFootprintInWorldFrame(positionWorldToFootHoldInWorldFrame, leg);

  return positionWorldToFootHoldInWorldFrame;
}

void FootholdGeneratorStaticGait::constrainFootholdInFootprintInWorldFrame(Position& positionWorldToFootHoldInWorldFrame,
                                                                           const LegBase& leg, bool compareWithPlannedFootholds) {
  const RotationQuaternion& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
  const Position& positionWorldToBaseInWorldFrame = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
  Position positionBaseToFootHoldInBaseFrame =
      orientationWorldToBase.rotate(positionWorldToFootHoldInWorldFrame - positionWorldToBaseInWorldFrame);

  Position positionBaseToComplementLateralLegFootHoldInBaseFrame;
  Position positionBaseToComplementLongitudinalLegFootHoldInBaseFrame;

  if (compareWithPlannedFootholds) {
    positionBaseToComplementLateralLegFootHoldInBaseFrame =
        orientationWorldToBase.rotate(getComplementLegLateral(leg).getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame() -
                                      positionWorldToBaseInWorldFrame);
    positionBaseToComplementLongitudinalLegFootHoldInBaseFrame =
        orientationWorldToBase.rotate(getComplementLegLongitude(leg).getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame() -
                                      positionWorldToBaseInWorldFrame);
  } else {
    positionBaseToComplementLateralLegFootHoldInBaseFrame = orientationWorldToBase.rotate(
        getComplementLegLateral(leg).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() -
        positionWorldToBaseInWorldFrame);
    positionBaseToComplementLongitudinalLegFootHoldInBaseFrame = orientationWorldToBase.rotate(
        getComplementLegLongitude(leg).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() -
        positionWorldToBaseInWorldFrame);
  }

  /*************************************************************************************************
   * Check for min/max distances between the newly generated foothold and the other foot positions *
   *************************************************************************************************/
  const double longitudeDistance =
      std::abs((positionBaseToFootHoldInBaseFrame -
                getComplementLegLongitude(leg).getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame())
                   .x());
  const double lateralDistance =
      std::abs((positionBaseToFootHoldInBaseFrame -
                getComplementLegLateral(leg).getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame())
                   .y());

  const loco::Vector unitVectorLongitudeInBaseFrame(
      (positionBaseToFootHoldInBaseFrame - positionBaseToComplementLongitudinalLegFootHoldInBaseFrame).normalized());

  // constrain desired position to be between min and max distance from complement lateral and longitudinal leg
  if (longitudeDistance >= paramMaxDistanceForeHind_.getValue()) {
    positionBaseToFootHoldInBaseFrame = positionBaseToComplementLongitudinalLegFootHoldInBaseFrame +
                                        loco::Position(paramMaxDistanceForeHind_.getValue() * unitVectorLongitudeInBaseFrame);
  } else if (longitudeDistance <= paramMinDistanceForeHind_.getValue()) {
    positionBaseToFootHoldInBaseFrame = positionBaseToComplementLongitudinalLegFootHoldInBaseFrame +
                                        loco::Position(paramMinDistanceForeHind_.getValue() * unitVectorLongitudeInBaseFrame);
  }

  const loco::Vector unitVectorLateralInBaseFrame(
      (positionBaseToFootHoldInBaseFrame - positionBaseToComplementLateralLegFootHoldInBaseFrame).normalized());

  if (lateralDistance >= paramMaxDistanceLeftRight_.getValue()) {
    positionBaseToFootHoldInBaseFrame = positionBaseToComplementLateralLegFootHoldInBaseFrame +
                                        loco::Position(paramMaxDistanceLeftRight_.getValue() * unitVectorLateralInBaseFrame);
  } else if (lateralDistance <= paramMinDistanceLeftRight_.getValue()) {
    positionBaseToFootHoldInBaseFrame = positionBaseToComplementLateralLegFootHoldInBaseFrame +
                                        loco::Position(paramMinDistanceLeftRight_.getValue() * unitVectorLateralInBaseFrame);
  }

  positionWorldToFootHoldInWorldFrame =
      positionWorldToBaseInWorldFrame + orientationWorldToBase.inverseRotate(positionBaseToFootHoldInBaseFrame);

  terrain_.getHeight(positionWorldToFootHoldInWorldFrame);
}

// Evaluate feed forward component
Position FootholdGeneratorStaticGait::getPositionFootprintCenterToDesiredFootHoldInControlFrame(const LegBase& leg) {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  double desiredHeadingVelocity = torso_.getDesiredState().getLinearVelocityTargetInControlFrame().x();
  double desiredLateralVelocity = torso_.getDesiredState().getLinearVelocityTargetInControlFrame().y();
  double desiredAngularVelocity = torso_.getDesiredState().getAngularVelocityBaseInControlFrame().z();
  const double strideDuration = torso_.getStrideDuration();

  // Update ellispoid with latest parameters values
  ellipsoid_.setSemiPrincipalAxisA(paramMaxStepLengthHeading_.getValue());
  ellipsoid_.setSemiPrincipalAxisB(paramMaxStepLengthLateral_.getValue());
  ellipsoid_.setSemiPrincipalAxisC(paramMaxStepAngular_.getValue());

  Vector desOffset(desiredHeadingVelocity * strideDuration, desiredLateralVelocity * strideDuration,
                   desiredAngularVelocity * strideDuration);
  ellipsoid_.constrainVectorToVolume(desOffset);

  const Position positionDesiredFootOnTerrainHeadingOffsetInControlFrame(desOffset.x(), 0.0, 0.0);
  const Position positionDesiredFootOnTerrainLateralOffsetInControlFrame(0.0, desOffset.y(), 0.0);

  // x-y position
  const Position positionDefaultFootToDesiredFootHoldInControlFrame =
      positionDesiredFootOnTerrainHeadingOffsetInControlFrame + positionDesiredFootOnTerrainLateralOffsetInControlFrame;

  EulerAnglesZyx eulerYaw;
  eulerYaw.setYaw(-desOffset.z());
  // eulerYaw.setYaw(-desiredAngularOffset);
  const RotationQuaternion rotationCurrentToDesired(eulerYaw);

  const Position positionFootprintCenterToRotatedDefaultFootholdInWorldFrame =
      rotationCurrentToDesired.rotate(positionFootprintCenterToDefaultFootholdInControlFrame_[leg.getId()]);

  return (positionFootprintCenterToRotatedDefaultFootholdInWorldFrame + positionDefaultFootToDesiredFootHoldInControlFrame);
}

const Legs& FootholdGeneratorStaticGait::getLegs() const {
  return legs_;
}

const double FootholdGeneratorStaticGait::getMaxStepLengthHeading() const {
  return paramMaxStepLengthHeading_.getValue();
}

const double FootholdGeneratorStaticGait::getMaxStepLengthLateral() const {
  return paramMaxStepLengthLateral_.getValue();
}

const double FootholdGeneratorStaticGait::getMaxStepAngular() const {
  return paramMaxStepAngular_.getValue();
}

std::ostream& operator<<(std::ostream& out, const FootholdGeneratorStaticGait& fhGen) {
  out << std::endl;
  out << "----------------------------------" << std::endl;
  out << "Class: FootholdGeneratorStaticGait" << std::endl;
  out << "max step components: " << std::endl
      << color::blue << "\theading: " << color::red << fhGen.getMaxStepLengthHeading() << color::def << std::endl
      << color::blue << "\tlateral: " << color::red << fhGen.getMaxStepLengthLateral() << color::def << std::endl
      << color::blue << "\tangular: " << color::red << fhGen.getMaxStepAngular() << color::def << std::endl;
  out << "----------------------------------" << std::endl;
  out << std::endl;

  return out;
}

const LegBase& FootholdGeneratorStaticGait::getComplementLegLateral(const LegBase& leg) {
  int legId = static_cast<int>(leg.getId());
  int complementLegId;

  switch (legId) {
    case 0:
      complementLegId = 1;
      break;
    case 1:
      complementLegId = 0;
      break;
    case 2:
      complementLegId = 3;
      break;
    case 3:
      complementLegId = 2;
      break;
    default:
      complementLegId = -1;
  }

  if (complementLegId == -1) {
    throw std::runtime_error("[FootholdGeneratorStaticGait::getComplementLegLateral] Error: leg id was -1.");
  }
  // fixme: remove get
  return legs_.get(complementLegId);
}

const LegBase& FootholdGeneratorStaticGait::getComplementLegLongitude(const LegBase& leg) {
  int legId = static_cast<int>(leg.getId());
  int complementLegId;

  switch (legId) {
    case 0:
      complementLegId = 2;
      break;
    case 1:
      complementLegId = 3;
      break;
    case 2:
      complementLegId = 0;
      break;
    case 3:
      complementLegId = 1;
      break;
    default:
      complementLegId = -1;
  }

  if (complementLegId == -1) {
    throw std::runtime_error("[FootholdGeneratorStaticGait::getComplementLegLongitude] Error: leg id was -1.");
  }
  // fixme: remove get
  return legs_.get(complementLegId);
}

const LegBase& FootholdGeneratorStaticGait::getComplementLegDiagonal(const LegBase& leg) {
  int legId = static_cast<int>(leg.getId());
  int complementLegId;

  switch (legId) {
    case 0:
      complementLegId = 3;
      break;
    case 1:
      complementLegId = 2;
      break;
    case 2:
      complementLegId = 1;
      break;
    case 3:
      complementLegId = 0;
      break;
    default:
      complementLegId = -1;
  }

  if (complementLegId == -1) {
    throw std::runtime_error("[FootholdGeneratorStaticGait::getComplementLegDiagonal] Error: leg id was -1.");
  }

  // fixme: remove get
  return legs_.get(complementLegId);
}

} /* namespace loco */
