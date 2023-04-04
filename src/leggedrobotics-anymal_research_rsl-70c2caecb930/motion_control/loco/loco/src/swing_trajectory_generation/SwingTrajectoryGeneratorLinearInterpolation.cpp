/*
 * SwingTrajectoryGeneratorLinearInterpolation.cpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso, Christian Gehring
 */

// loco
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorLinearInterpolation.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace loco {

SwingTrajectoryGeneratorLinearInterpolation::SwingTrajectoryGeneratorLinearInterpolation(WholeBody& wholeBody, TerrainModelBase& terrain)
    : SwingTrajectoryGeneratorBase(),
      torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      terrain_(terrain),
      stepInterpolationFunction_(),
      swingFootHeightTrajectory_() {}

bool SwingTrajectoryGeneratorLinearInterpolation::loadParameters(const TiXmlHandle& handle) {
  stepInterpolationFunction_.clear();
  swingFootHeightTrajectory_.clear();

  TiXmlHandle hFPS(handle.FirstChild("FootPlacementStrategy").FirstChild("SwingTrajectoryGenerator").FirstChild("LinearInterpolation"));

  /***********************************
   * Height trajectory interpolation *
   ***********************************/
  TiXmlElement* pElem = hFPS.FirstChild("HeightTrajectory").Element();
  if (pElem == nullptr) {
    std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorLinearInterpolation/loadParameters] "
              << message_logger::color::red << "***" << message_logger::color::blue
              << "Could not find section 'FootPlacementStrategy::SwingTrajectoryGenerator::LinearInterpolation::HeightTrajectory'."
              << message_logger::color::red << "***" << message_logger::color::def << std::endl;
    return false;
  } else {
    int iKnot;
    double t, value;
    std::vector<double> tValues, xValues;
    for (TiXmlElement* child = pElem->FirstChild()->ToElement(); child != nullptr; child = child->NextSiblingElement()) {
      if (child->QueryDoubleAttribute("t", &t) != TIXML_SUCCESS) {
        std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorLinearInterpolation/loadParameters] "
                  << message_logger::color::blue << "Could not find t of knot!" << message_logger::color::def << std::endl;
        return false;
      }
      if (child->QueryDoubleAttribute("v", &value) != TIXML_SUCCESS) {
        std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorLinearInterpolation/loadParameters] "
                  << message_logger::color::blue << "Could not find v of knot!" << message_logger::color::def << std::endl;
        return false;
      }
      tValues.push_back(t);
      xValues.push_back(value);
      swingFootHeightTrajectory_.addKnot(t, value);
    }

    std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorLinearInterpolation/loadParameters] "
              << message_logger::color::blue << "Added the following knots to footstep ground interpolation: " << message_logger::color::def
              << std::endl;
    for (int k = 0; k < swingFootHeightTrajectory_.getKnotCount(); k++) {
      std::cout << message_logger::color::blue << "\tt: " << message_logger::color::red << swingFootHeightTrajectory_.getKnotPosition(k)
                << message_logger::color::blue << "\t\tv: " << message_logger::color::red << swingFootHeightTrajectory_.getKnotValue(k)
                << message_logger::color::def << std::endl;
    }
  }
  /***********************************/

  /***********************************
   * Ground trajectory interpolation *
   ***********************************/
  pElem = hFPS.FirstChild("PlaneTrajectory").Element();
  if (pElem == nullptr) {
    std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorLinearInterpolation/loadParameters] "
              << message_logger::color::red << "***" << message_logger::color::blue
              << "Could not find section 'FootPlacementStrategyInvertedPendulum::PlaneTrajectory'." << message_logger::color::red << "***"
              << message_logger::color::def << std::endl;
    return false;
  } else {
    int iKnot;
    double t, value;
    std::vector<double> tValues, xValues;
    for (TiXmlElement* child = pElem->FirstChild()->ToElement(); child != nullptr; child = child->NextSiblingElement()) {
      if (child->QueryDoubleAttribute("t", &t) != TIXML_SUCCESS) {
        std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorLinearInterpolation/loadParameters] "
                  << message_logger::color::blue << "Could not find t of knot!" << message_logger::color::def << std::endl;
        return false;
      }
      if (child->QueryDoubleAttribute("v", &value) != TIXML_SUCCESS) {
        std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorLinearInterpolation/loadParameters] "
                  << message_logger::color::blue << "Could not find v of knot!" << message_logger::color::def << std::endl;
        return false;
      }
      tValues.push_back(t);
      xValues.push_back(value);
      stepInterpolationFunction_.addKnot(t, value);
    }

    std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorLinearInterpolation/loadParameters] "
              << message_logger::color::blue << "Added the following knots to footstep ground interpolation: " << message_logger::color::def
              << std::endl;
    for (int k = 0; k < stepInterpolationFunction_.getKnotCount(); k++) {
      std::cout << message_logger::color::blue << "\tt: " << message_logger::color::red << stepInterpolationFunction_.getKnotPosition(k)
                << message_logger::color::blue << "\t\tv: " << message_logger::color::red << stepInterpolationFunction_.getKnotValue(k)
                << message_logger::color::def << std::endl;
    }
  }
  /***********************************/

  return true;
}

bool SwingTrajectoryGeneratorLinearInterpolation::getDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame,
                                                                      LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                                                                      LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                                                                      const Position& positionWorldToDesiredFootholdInControlFrame,
                                                                      LegBase* leg, double dt) {
  Position positionWorldToHipOnTerrainInWorldFrame = getPositionWorldToHipOnPlaneAlongWorldZInWorldFrame(leg);
  const Position positionHipOnTerrainToDesiredFootholdInControlFrame =
      positionWorldToDesiredFootholdInControlFrame -
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().rotate(positionWorldToHipOnTerrainInWorldFrame);
  positionWorldToDesiredFootInWorldFrame =
      getPositionWorldToDesiredFootInWorldFrame(positionHipOnTerrainToDesiredFootholdInControlFrame, leg->getId());
  linearVelocityDesiredFootInWorldFrame.setZero();
  linearAccelerationDesiredFootInWorldFrame.setZero();
  return true;
}

Position SwingTrajectoryGeneratorLinearInterpolation::getPositionWorldToHipOnPlaneAlongWorldZInWorldFrame(LegBase* leg) {
  // Find starting point: hip projected vertically on ground
  Position positionWorldToHipOnPlaneAlongWorldZInWorldFrame = leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame();
  terrain_.getHeight(positionWorldToHipOnPlaneAlongWorldZInWorldFrame);
  return positionWorldToHipOnPlaneAlongWorldZInWorldFrame;
}

Position SwingTrajectoryGeneratorLinearInterpolation::getPositionWorldToDesiredFootInWorldFrame(
    const Position& positionHipOnTerrainToDesiredFootholdInControlFrame, int legId) {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  // fixme: remove get
  LegBase* leg = legs_.getPtr(legId);

  // Find starting point: hip projected vertically on ground
  Position positionWorldToHipOnPlaneAlongWorldZInWorldFrame = getPositionWorldToHipOnPlaneAlongWorldZInWorldFrame(leg);

  //--- Evaluate the interpolation contributions
  const Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame =
      getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(*leg,
                                                                             positionHipOnTerrainToDesiredFootholdInControlFrame);  // x-y
  const Position positionDesiredFootOnTerrainToDesiredFootInControlFrame =
      getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                 positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame);  // z
  //---

  //--- Build the world to desired foot position and return it
  const Position positionWorldToDesiredFootInWorldFrame =
      positionWorldToHipOnPlaneAlongWorldZInWorldFrame  // starting point, hip projected on the plane along world z axis
      + orientationWorldToControl.inverseRotate(positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame)  // ground
      + orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);             // height
  //---

  return positionWorldToDesiredFootInWorldFrame;
}

double SwingTrajectoryGeneratorLinearInterpolation::getLateralComponentOfFootStep(double phase, double initialStepOffset, double stepGuess,
                                                                                  LegBase* leg) {
  // we want the step, for the first part of the motion, to be pretty conservative, and towards the end pretty aggressive in terms of
  // stepping at the desired feedback-based foot position
  phase = robot_utils::mapTo01Range(phase - 0.3, 0, 0.5);
  //  return stepGuess * phase + initialStepOffset * (1-phase);
  double result = stepGuess * phase + initialStepOffset * (1.0 - phase);
  const double legLength = leg->getLimbProperties().getMaximumLimbExtension();
  robot_utils::boundToRange(&result, -legLength * 0.5, legLength * 0.5);
  return result;
}

double SwingTrajectoryGeneratorLinearInterpolation::getHeadingComponentOfFootStep(double phase, double initialStepOffset, double stepGuess,
                                                                                  LegBase* leg) {
  phase = stepInterpolationFunction_.evaluate_linear(phase);
  //  return stepGuess * phase + initialStepOffset * (1-phase);
  double result = stepGuess * phase + initialStepOffset * (1.0 - phase);
  const double legLength = leg->getLimbProperties().getMaximumLimbExtension();
  const double sagittalMaxLegLengthScale = 0.5;
  robot_utils::boundToRange(&result, -legLength * sagittalMaxLegLengthScale, legLength * sagittalMaxLegLengthScale);
  return result;
}

// Evaluate the interpolation phase relative to a given leg
double SwingTrajectoryGeneratorLinearInterpolation::getInterpolationPhase(const LegBase& leg) {
  double swingPhase = 1.0;
  if (!((leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant))) {
    swingPhase = leg.getContactSchedule().getSwingPhase();
  }
  //  return robot_utils::mapTo01Range(swingPhase, leg.getStateLiftOff().getPhase(), 1.0);
  return robot_utils::mapTo01Range(swingPhase, 0.0, 1.0);
}

/*
 * Interpolate height: get the vector pointing from the current interpolated foot hold to the height of the desired foot based on the
 * interpolation phase
 */
Position SwingTrajectoryGeneratorLinearInterpolation::getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(
    const LegBase& leg, const Position& positionHipOnTerrainToDesiredFootOnTerrainInControlFrame) {
  const double desiredFootHeight = swingFootHeightTrajectory_.evaluate(leg.getContactSchedule().getSwingPhase());
  const double interpolationParameter = getInterpolationPhase(leg);

  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const Position positionHipOnTerrainToDesiredFootOnTerrainInWorldFrame =
      orientationWorldToControl.inverseRotate(positionHipOnTerrainToDesiredFootOnTerrainInControlFrame);

  Vector normalToPlaneAtCurrentFootPositionInWorldFrame;
  terrain_.getNormal(positionHipOnTerrainToDesiredFootOnTerrainInWorldFrame, normalToPlaneAtCurrentFootPositionInWorldFrame);

  const Vector normalToPlaneAtCurrentFootPositionInControlFrame =
      orientationWorldToControl.rotate(normalToPlaneAtCurrentFootPositionInWorldFrame);

  const Position positionDesiredFootOnTerrainToDesiredFootInControlFrame =
      desiredFootHeight * Position(normalToPlaneAtCurrentFootPositionInControlFrame);
  return positionDesiredFootOnTerrainToDesiredFootInControlFrame;
}

Position SwingTrajectoryGeneratorLinearInterpolation::getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(
    LegBase& leg, const Position& positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame) {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  //--- starting point for trajectory interpolation
  const Position positionHipOnTerrainAlongNormalToFootAtLiftOffInWorldFrame =
      leg.getStateLiftOff()->getPositionWorldToFootInWorldFrame() -
      leg.getStateLiftOff()->getPositionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame();
  const Position positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame =
      orientationWorldToControl.rotate(positionHipOnTerrainAlongNormalToFootAtLiftOffInWorldFrame);
  //---

  Position positionWorldToHipOnTerrainInWorldFrame = leg.getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame();
  terrain_.getHeight(positionWorldToHipOnTerrainInWorldFrame);

  // todo: get hip along normal, not vertical
  //  Position positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame =
  //      orientationWorldToControl.rotate(
  //          positionWorldToFootholdIWorldFrame
  //              - positionWorldToHipOnTerrainInWorldFrame);
  //  positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame.z() = 0.0;

  const double interpolationParameter = getInterpolationPhase(leg);

  const Position positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame(
      // x
      getHeadingComponentOfFootStep(interpolationParameter, positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame.x(),
                                    positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame.x(), &leg),
      // y
      getLateralComponentOfFootStep(interpolationParameter, positionHipOnTerrainAlongNormalToFootAtLiftOffInControlFrame.y(),
                                    positionHipOnTerrainAlongNormalToDesiredFootHoldOnTerrainInControlFrame.y(), &leg),
      // z
      0.0);

  return positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInControlFrame;
}

const robot_utils::catmull_rom::Trajectory1D& SwingTrajectoryGeneratorLinearInterpolation::getStepInterpolationFunction() const {
  return stepInterpolationFunction_;
}
const robot_utils::catmull_rom::Trajectory1D& SwingTrajectoryGeneratorLinearInterpolation::getSwingFootHeightTrajectory() const {
  return swingFootHeightTrajectory_;
}

std::ostream& operator<<(std::ostream& out, const SwingTrajectoryGeneratorLinearInterpolation& stGen) {
  out << std::endl;
  out << "--------------------------------------------------" << std::endl;
  out << "Class: SwingTrajectoryGeneratorLinearInterpolation" << std::endl;
  out << "Height trajectory knots:" << std::endl;
  for (int k = 0; k < stGen.getSwingFootHeightTrajectory().getKnotCount(); k++) {
    std::cout << message_logger::color::blue << "\tt: " << message_logger::color::red
              << stGen.getSwingFootHeightTrajectory().getKnotPosition(k) << message_logger::color::blue
              << "\t\tv: " << message_logger::color::red << stGen.getSwingFootHeightTrajectory().getKnotValue(k)
              << message_logger::color::def << std::endl;
  }
  out << "Plane trajectory knots:" << std::endl;
  for (int k = 0; k < stGen.getStepInterpolationFunction().getKnotCount(); k++) {
    std::cout << message_logger::color::blue << "\tt: " << message_logger::color::red
              << stGen.getStepInterpolationFunction().getKnotPosition(k) << message_logger::color::blue
              << "\t\tv: " << message_logger::color::red << stGen.getStepInterpolationFunction().getKnotValue(k)
              << message_logger::color::def << std::endl;
  }
  out << "--------------------------------------------------" << std::endl;
  out << std::endl;

  return out;
}

} /* namespace loco */
