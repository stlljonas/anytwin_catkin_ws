/*
 * FootholdGeneratorInvertedPendulumBase.cpp
 *
 *  Created on: April 12, 2018
 *      Author: Fabian Jenelten, C. Dario Bellicoso, Christian Gehring
 */

// loco
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumBase.hpp"

namespace loco {

FootholdGeneratorInvertedPendulumBase::FootholdGeneratorInvertedPendulumBase(WholeBody& wholeBody, TerrainModelBase& terrain)
    : FootholdGeneratorFreePlane(),
      wholeBody_(wholeBody),
      torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      stepFeedbackScale_(1.0),
      stepFeedforwardScale_(1.0),
      terrain_(terrain),
      positionDesiredFootHoldOnTerrainFeedBackInControlFrame_(legs_.size()),
      positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_(legs_.size()) {}

void FootholdGeneratorInvertedPendulumBase::setFeedbackScale(double feedbackScale) noexcept {
  stepFeedbackScale_ = feedbackScale;
}

double FootholdGeneratorInvertedPendulumBase::getFeedbackScale() const noexcept {
  return stepFeedbackScale_;
}

void FootholdGeneratorInvertedPendulumBase::setFeedforwardScale(double feedforwardScale) noexcept {
  stepFeedforwardScale_ = feedforwardScale;
}

double FootholdGeneratorInvertedPendulumBase::getFeedforwardScale() const noexcept {
  return stepFeedforwardScale_;
}

void FootholdGeneratorInvertedPendulumBase::setDefaultFootholds(double distanceReferenceToDefaultFootholdHeading,
                                                                double distanceReferenceToDefaultFootholdLateralHind,
                                                                double distanceReferenceToDefaultFootholdLateralFront, double offsetHeading,
                                                                double offsetLateral) {
  // LF.
  const Position positionLimbBaseLFToDefaultFootholdInControlFrame =
      Position(distanceReferenceToDefaultFootholdHeading, distanceReferenceToDefaultFootholdLateralFront, 0.0) -
      legs_.get(0).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame();

  // RF.
  const Position positionLimbBaseRFToDefaultFootholdInControlFrame =
      Position(distanceReferenceToDefaultFootholdHeading, -distanceReferenceToDefaultFootholdLateralFront, 0.0) -
      legs_.get(1).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame();

  // LH.
  const Position positionLimbBaseLHToDefaultFootholdInControlFrame =
      Position(-distanceReferenceToDefaultFootholdHeading, distanceReferenceToDefaultFootholdLateralHind, 0.0) -
      legs_.get(2).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame();

  // RH.
  const Position positionLimbBaseRHToDefaultFootholdInControlFrame =
      Position(-distanceReferenceToDefaultFootholdHeading, -distanceReferenceToDefaultFootholdLateralHind, 0.0) -
      legs_.get(3).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame();

  // LF.
  legs_.getPtr(0)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      Position(positionLimbBaseLFToDefaultFootholdInControlFrame.x() + offsetHeading,
               positionLimbBaseLFToDefaultFootholdInControlFrame.y() + offsetLateral, 0.0));

  // RF.
  legs_.getPtr(1)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      Position(positionLimbBaseRFToDefaultFootholdInControlFrame.x() + offsetHeading,
               positionLimbBaseRFToDefaultFootholdInControlFrame.y() + offsetLateral, 0.0));

  // LH.
  legs_.getPtr(2)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      Position(positionLimbBaseLHToDefaultFootholdInControlFrame.x() + offsetHeading,
               positionLimbBaseLHToDefaultFootholdInControlFrame.y() + offsetLateral, 0.0));

  // RH.
  legs_.getPtr(3)->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(
      Position(positionLimbBaseRHToDefaultFootholdInControlFrame.x() + offsetHeading,
               positionLimbBaseRHToDefaultFootholdInControlFrame.y() + offsetLateral, 0.0));
}

const Legs& FootholdGeneratorInvertedPendulumBase::getLegs() const {
  return legs_;
}

const Position& FootholdGeneratorInvertedPendulumBase::getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame(
    int legId) const {
  return positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[legId];
}

const Position& FootholdGeneratorInvertedPendulumBase::getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame(
    int legId) const {
  return positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[legId];
}

Position FootholdGeneratorInvertedPendulumBase::getPositionReferenceToDesiredFootOnTerrainInWorldFrame(const LegBase& leg) {
  positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()] = evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(leg);
  positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()] = evaluateDesiredFootHoldOnTerrainFeedBackInControlFrame(leg);

  return torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().inverseRotate(
      positionDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg.getId()] +
      positionDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg.getId()]);
}

double FootholdGeneratorInvertedPendulumBase::getFeedbackInvertedPendulum(const LegBase& leg) const {
  const double heightInvertedPendulum = std::fabs(leg.getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame().z() -
                                                  getPositionWorldToDefaultFootholdOnGroundInWorldFrame(leg).z());
  return std::sqrt(heightInvertedPendulum / torso_.getProperties().getGravity().toImplementation().norm());
}

Position FootholdGeneratorInvertedPendulumBase::getPositionWorldToDefaultFootholdOnGroundInWorldFrame(const LegBase& leg) const {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  auto positionWorldToDefaultFootholdInWorldFrame =
      (leg.getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame() +
       orientationWorldToControl.inverseRotate(leg.getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame()));

  terrain_.getHeight(positionWorldToDefaultFootholdInWorldFrame);  // map on terrain
  return positionWorldToDefaultFootholdInWorldFrame;
}

} /* namespace loco */
