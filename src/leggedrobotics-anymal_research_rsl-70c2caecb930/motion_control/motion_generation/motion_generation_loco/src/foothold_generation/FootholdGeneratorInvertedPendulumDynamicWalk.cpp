/*
 * FootholdGeneratorInvertedPendulumDynamicWalk.cpp
 *
 *  Created on: April 12, 2018
 *      Author: Fabian Jenelten, C. Dario Bellicoso, Christian Gehring
 */

// tinyxml tools.
#include "tinyxml_tools/tinyxml_tools.hpp"

// loco.
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumDynamicWalk.hpp"
#include <loco/common/loco_common.hpp>

// robot_utils.
#include "robot_utils/physical_definitions.hpp"

namespace loco {

FootholdGeneratorInvertedPendulumDynamicWalk::FootholdGeneratorInvertedPendulumDynamicWalk(
    WholeBody& wholeBody,
    TerrainModelBase& terrain,
    ContactScheduleZmp& contactSchedule)
    : FootholdGeneratorInvertedPendulumBase(wholeBody, terrain),
      positionReferencePointToDefaultFootholdInControlFrame_(),
      footholdCenter_(foothold_generator::FootholdCenter::TorsoCom),
      contactSchedule_(contactSchedule),
      feedbackNominalInvertedPendulum_(0.0),
      gravityFactor_(1.0) {
  double maxLimbExtension = 0.0;
  for (const auto leg : legs_) {
    maxLimbExtension += leg->getLegProperties().getMaximumLimbExtension();
  }
  if (legs_.size() > 0u) {
    maxLimbExtension /= static_cast<double>(legs_.size());
  }

  // Compute nominal feedback term.
  const double nominalLimbExtension = 0.7 * maxLimbExtension;
  feedbackNominalInvertedPendulum_ =  std::sqrt(nominalLimbExtension / robot_utils::physical_definitions::getAbsoluteGravityAcceleration());
}

bool FootholdGeneratorInvertedPendulumDynamicWalk::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle fpsHandle = handle;
  if (!tinyxml_tools::getChildHandle(fpsHandle, handle, "FootPlacementStrategy")) { return false; }

  TiXmlHandle fhgHandle = handle;
  if (!tinyxml_tools::getChildHandle(fhgHandle, fpsHandle, "FootholdGenerator")) { return false; }

  TiXmlHandle optfhgHandle = handle;
  if (!tinyxml_tools::getChildHandle(optfhgHandle, fhgHandle, "OptimizedInvPend")) { return false; }

  std::string footprintCenter;
  if (!tinyxml_tools::loadParameter(footprintCenter, optfhgHandle, "footprint_center")) { return false; }

  if (footprintCenter=="torso_com") {
    footholdCenter_ = foothold_generator::FootholdCenter::TorsoCom;
  } else if (footprintCenter=="base") {
    footholdCenter_ = foothold_generator::FootholdCenter::Base;
  } else if (footprintCenter=="whole_body_com") {
    footholdCenter_ = foothold_generator::FootholdCenter::WholeBodyCom;
  } else {
    MELO_WARN_STREAM("[FootholdGeneratorInvertedPendulumDynamicWalk::loadParameters] Foothold center must be torso_com, base or whole_body_com");
    return false;
  }

  // Gravity factor.
  TiXmlHandle comSupportHandle = handle;
  if(!tinyxml_tools::getChildHandle(comSupportHandle, handle, "ComSupportControl")) { return false; }
  TiXmlHandle terrainHandle = handle;
  if(!tinyxml_tools::getChildHandle(terrainHandle, comSupportHandle, "TerrainAdaption")) { return false; }
  if(!tinyxml_tools::loadParameter(gravityFactor_, terrainHandle, "gravity_factor"))  { return false; }

  return true;
}

void FootholdGeneratorInvertedPendulumDynamicWalk::setDefaultFootholds(
    double distanceReferenceToDefaultFootholdHeading,
    double distanceReferenceToDefaultFootholdLateralHind,
    double distanceReferenceToDefaultFootholdLateralFront,
    double offsetHeading,
    double offsetLateral) {

  // LF.
  positionReferencePointToDefaultFootholdInControlFrame_[contact_schedule::LegEnumAnymal::LF] =
      Position(
          distanceReferenceToDefaultFootholdHeading      + offsetHeading,
          distanceReferenceToDefaultFootholdLateralFront + offsetLateral,
          0.0
      );

  // RF.
  positionReferencePointToDefaultFootholdInControlFrame_[contact_schedule::LegEnumAnymal::RF] =
      Position(
           distanceReferenceToDefaultFootholdHeading      + offsetHeading,
          -distanceReferenceToDefaultFootholdLateralFront + offsetLateral,
          0.0
       );

  // LH.
  positionReferencePointToDefaultFootholdInControlFrame_[contact_schedule::LegEnumAnymal::LH] =
      Position(
          -distanceReferenceToDefaultFootholdHeading     + offsetHeading,
           distanceReferenceToDefaultFootholdLateralHind + offsetLateral,
          0.0
      );

  // RH.
  positionReferencePointToDefaultFootholdInControlFrame_[contact_schedule::LegEnumAnymal::RH] =
      Position(
          -distanceReferenceToDefaultFootholdHeading     + offsetHeading,
          -distanceReferenceToDefaultFootholdLateralHind + offsetLateral,
          0.0
      );

  // Store.
  const Position positionBaseToReferencePoint = getPositionWorldToReferencePointInWorldFrame() - torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
  legs_.getPtr(0)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      positionBaseToReferencePoint +
      positionReferencePointToDefaultFootholdInControlFrame_[contact_schedule::LegEnumAnymal::LF] -
      legs_.get(0).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame());

  legs_.getPtr(1)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      positionBaseToReferencePoint +
      positionReferencePointToDefaultFootholdInControlFrame_[contact_schedule::LegEnumAnymal::RF] -
      legs_.get(1).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame());

  legs_.getPtr(2)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      positionBaseToReferencePoint +
      positionReferencePointToDefaultFootholdInControlFrame_[contact_schedule::LegEnumAnymal::LH] -
      legs_.get(2).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame());

  legs_.getPtr(3)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      positionBaseToReferencePoint +
      positionReferencePointToDefaultFootholdInControlFrame_[contact_schedule::LegEnumAnymal::RH] -
      legs_.get(3).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame());
}

Position FootholdGeneratorInvertedPendulumDynamicWalk::computeWorldToFootholdInWorldFrame(int legId) {
  // Build the desired foothold.
  Position positionWorldToFootholdInWorldFrame =
      getPositionWorldToReferencePointInWorldFrame() +
      getPositionReferenceToDesiredFootOnTerrainInWorldFrame(legs_.get(legId));

  // Project on ground.
  Position positionWorldToFootholdProjectedAlongSurfaceNormal = terrain_.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(positionWorldToFootholdInWorldFrame);
  Position positionWorldToFootholdProjectedAlongGravityVerctor = positionWorldToFootholdInWorldFrame;
  terrain_.getHeight(positionWorldToFootholdProjectedAlongGravityVerctor);

  // Return desired foothold projected on ground.
  return (gravityFactor_ * positionWorldToFootholdProjectedAlongGravityVerctor + (1.0 - gravityFactor_) * positionWorldToFootholdProjectedAlongSurfaceNormal);
}

Position FootholdGeneratorInvertedPendulumDynamicWalk::getPositionWorldToReferencePointInWorldFrame() const {
  switch (footholdCenter_) {
    case foothold_generator::FootholdCenter::Base : {
      return wholeBody_.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
    } break;

    case foothold_generator::FootholdCenter::TorsoCom : {
      return wholeBody_.getTorso().getMeasuredState().getPositionWorldToCenterOfMassInWorldFrame();
    } break;

    case foothold_generator::FootholdCenter::WholeBodyCom : {
      return wholeBody_.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame();
    } break;

    default : {
      MELO_FATAL_STREAM("[FootholdGeneratorInvertedPendulumDynamicWalk] Unknown footprint center.");
      return Position::Zero();
    } break;
  }
}

Position FootholdGeneratorInvertedPendulumDynamicWalk::evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg) {
  const auto legId = static_cast<contact_schedule::LegEnumAnymal>(leg.getId());

  if (stepFeedforwardScale_<=0.0) {
    return positionReferencePointToDefaultFootholdInControlFrame_[legId];
  }

  // Build foot position and return it.
  return positionReferencePointToDefaultFootholdInControlFrame_[legId] +
      stepFeedforwardScale_ * Position(computeLinearVelocityProjectionInControlFrame(leg));
}

Position FootholdGeneratorInvertedPendulumDynamicWalk::evaluateDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg) {
  if (stepFeedbackScale_<=0.0) { return Position(0.0, 0.0, 0.0); }

  // Compute feedback correction.
  const double feedbackGain = stepFeedbackScale_*feedbackNominalInvertedPendulum_;
  return Position(computeLinearVelocityErrorInControlFrame(leg)) * feedbackGain;
}

LinearVelocity FootholdGeneratorInvertedPendulumDynamicWalk::computeLinearVelocityProjectionInControlFrame(const LegBase& leg) const {
  const auto legId = static_cast<contact_schedule::LegEnumAnymal>(leg.getId());

  // Get orientations.
  const auto orientationBaseToControl   =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl()*
          torso_.getMeasuredState().getOrientationWorldToBase().inverted();

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
  auto positionBaseToEndEffectorInControlFrame = orientationBaseToControl.rotate(
      leg.getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame());
  positionBaseToEndEffectorInControlFrame.z() = 0.0;

  LinearVelocity velocityProjectionInControlFrame = LinearVelocity(
      linearVelocityTargetInControlFrame.toImplementation() +
          angularVelocityTargetInControlFrame.toImplementation().cross(positionBaseToEndEffectorInControlFrame.toImplementation())
  ) * leg.getContactSchedule().getSwingDuration();

  return velocityProjectionInControlFrame;
}

LinearVelocity FootholdGeneratorInvertedPendulumDynamicWalk::computeLinearVelocityErrorInControlFrame(const LegBase& leg) const {
  const auto& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const auto& orientationWorldToBase    = torso_.getMeasuredState().getOrientationWorldToBase();

  const auto linearVelocityDesiredInWorldFrame  = orientationWorldToControl.inverseRotate(torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
  const auto linearVelocityMeasuredInWorldFrame = 0.5 * (
      leg.getLimbStateMeasured().getLinearVelocityLimbBaseInWorldFrame() +
          orientationWorldToBase.inverseRotate(torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame())
  );

  LinearVelocity linearVelocityErrorInControlFrame = orientationWorldToControl.rotate(linearVelocityMeasuredInWorldFrame - linearVelocityDesiredInWorldFrame);
  linearVelocityErrorInControlFrame.z() = 0.0;
  return linearVelocityErrorInControlFrame;
}

Position FootholdGeneratorInvertedPendulumDynamicWalk::getPositionReferenceToDesiredFootOnTerrainInWorldFrame(const LegBase& leg) {
  return torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().inverseRotate(
      evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(leg) +
      evaluateDesiredFootHoldOnTerrainFeedBackInControlFrame(leg)
  );
}

} /* namespace loco */
