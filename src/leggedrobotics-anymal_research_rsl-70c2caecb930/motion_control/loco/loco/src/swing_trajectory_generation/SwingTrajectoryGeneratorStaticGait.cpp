/*
 * SwingTrajectoryGeneratorStaticGait.cpp
 *
 *  Created on: Feb 6, 2015
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorStaticGait.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

namespace loco {

SwingTrajectoryGeneratorStaticGait::SwingTrajectoryGeneratorStaticGait(WholeBody& wholeBody, TerrainModelBase& terrain)
    : wholeBody_(wholeBody),
      terrain_(terrain),
      torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      endPlaneMotionAtPhase_(1.0),
      endHeightMotionAtPhase_(1.0),
      planeMotionInterpolationParameter_(0.0),
      heightMotionInterpolationParameter_(0.0) {}

bool SwingTrajectoryGeneratorStaticGait::loadParameters(const TiXmlHandle& handle) {
  std::vector<double> tValues, xValues;

  TiXmlHandle hFPS(handle.FirstChild("FootPlacementStrategy").FirstChild("SwingTrajectoryGenerator").FirstChild("StaticGait"));

  /***********************************
   * Height trajectory interpolation *
   ***********************************/
  tValues.clear();
  xValues.clear();
  TiXmlElement* pElem = hFPS.FirstChild("HeightTrajectory").Element();
  if (pElem == nullptr) {
    std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorStaticGait/loadParameters] " << message_logger::color::red
              << "***" << message_logger::color::blue
              << "Could not find section 'FootPlacementStrategy::SwingTrajectoryGenerator::StaticGait::HeightTrajectory'."
              << message_logger::color::red << "***" << message_logger::color::def << std::endl;
    return false;
  } else {
    int iKnot;
    double t, value;

    for (TiXmlElement* child = pElem->FirstChild()->ToElement(); child != nullptr; child = child->NextSiblingElement()) {
      if (child->QueryDoubleAttribute("t", &t) != TIXML_SUCCESS) {
        std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorStaticGait/loadParameters] " << message_logger::color::blue
                  << "Could not find t of knot!" << message_logger::color::def << std::endl;
        return false;
      }
      if (child->QueryDoubleAttribute("v", &value) != TIXML_SUCCESS) {
        std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorStaticGait/loadParameters] " << message_logger::color::blue
                  << "Could not find v of knot!" << message_logger::color::def << std::endl;
        return false;
      }
      tValues.push_back(t);
      xValues.push_back(value);
    }

    std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorStaticGait/loadParameters] " << message_logger::color::blue
              << "Added the following knots to footstep height interpolation: " << message_logger::color::def << std::endl;
    for (int k = 0; k < tValues.size(); k++) {
      std::cout << message_logger::color::blue << "\tt: " << message_logger::color::red << tValues[k] << message_logger::color::blue
                << "\t\tv: " << message_logger::color::red << xValues[k] << message_logger::color::def << std::endl;
    }

    heightTrajectory_.fitCurve(tValues, xValues);
    endHeightMotionAtPhase_ = tValues.back();
  }
  /***********************************/

  /***********************************
   * Ground trajectory interpolation *
   ***********************************/
  tValues.clear();
  xValues.clear();
  pElem = hFPS.FirstChild("PlaneTrajectory").Element();
  if (pElem == nullptr) {
    std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorStaticGait/loadParameters] " << message_logger::color::red
              << "***" << message_logger::color::blue
              << "Could not find section 'FootPlacementStrategy::SwingTrajectoryGenerator::StaticGait::PlaneTrajectory'."
              << message_logger::color::red << "***" << message_logger::color::def << std::endl;
    return false;
  } else {
    int iKnot;
    double t, value;
    std::vector<double> tValues, xValues;
    for (TiXmlElement* child = pElem->FirstChild()->ToElement(); child != nullptr; child = child->NextSiblingElement()) {
      if (child->QueryDoubleAttribute("t", &t) != TIXML_SUCCESS) {
        std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorStaticGait/loadParameters] " << message_logger::color::blue
                  << "Could not find t of knot!" << message_logger::color::def << std::endl;
        return false;
      }
      if (child->QueryDoubleAttribute("v", &value) != TIXML_SUCCESS) {
        std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorStaticGait/loadParameters] " << message_logger::color::blue
                  << "Could not find v of knot!" << message_logger::color::def << std::endl;
        return false;
      }
      tValues.push_back(t);
      xValues.push_back(value);
    }

    std::cout << message_logger::color::magenta << "[SwingTrajectoryGeneratorStaticGait/loadParameters] " << message_logger::color::blue
              << "Added the following knots to footstep ground interpolation: " << message_logger::color::def << std::endl;
    for (int k = 0; k < tValues.size(); k++) {
      std::cout << message_logger::color::blue << "\tt: " << message_logger::color::red << tValues[k] << message_logger::color::blue
                << "\t\tv: " << message_logger::color::red << xValues[k] << message_logger::color::def << std::endl;
    }

    planeTrajectory_.fitCurve(tValues, xValues);
    endPlaneMotionAtPhase_ = tValues.back();
  }
  /***********************************/

  return true;
}

bool SwingTrajectoryGeneratorStaticGait::getDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame,
                                                             LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                                                             LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                                                             const Position& positionWorldToDesiredFootholdInControlFrame, LegBase* leg,
                                                             double dt) {
  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  const Position positionWorldToFootAtLiftOffInControlFrame =
      orientationWorldToControl.rotate(leg->getStateLiftOff()->getPositionWorldToFootInWorldFrame());
  const Position positionWorldToValidatedFootHoldInControlFrame = positionWorldToDesiredFootholdInControlFrame;
  const Position offsetInControlFrame = positionWorldToValidatedFootHoldInControlFrame - positionWorldToFootAtLiftOffInControlFrame;
  const Position offsetInWorldFrame = orientationWorldToControl.inverseRotate(offsetInControlFrame);

  // Interpolate on the x-y plane
  const double interpolationParameter = getInterpolationPhaseForPlaneMotion(*leg);
  planeMotionInterpolationParameter_ = interpolationParameter;
  leg->getContactSchedulePtr()->setInterpolationParameter(interpolationParameter);

  const Position positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame =
      Position(getHeadingComponentOfFootStep(planeMotionInterpolationParameter_, offsetInControlFrame.x()),  // x
               getLateralComponentOfFootStep(planeMotionInterpolationParameter_, offsetInControlFrame.y()),  // y
               0.0);                                                                                         // z

  // Interpolate height trajectory
  const Position positionDesiredFootOnTerrainToDesiredFootInControlFrame =
      getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(*leg,
                                                                 positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame);  // z

  positionWorldToDesiredFootInWorldFrame =
      orientationWorldToControl.inverseRotate(positionWorldToFootAtLiftOffInControlFrame) +
      orientationWorldToControl.inverseRotate(positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame) +
      orientationWorldToControl.inverseRotate(positionDesiredFootOnTerrainToDesiredFootInControlFrame);

  double velScale = 0.0;
  if (!planeTrajectory_.evaluateDerivative(velScale, interpolationParameter, 1)) {
    return false;
  }
  double heightVel = 0.0;
  if (!heightTrajectory_.evaluateDerivative(heightVel, leg->getContactSchedule().getSwingPhase(), 1)) {
    return false;
  }
  linearVelocityDesiredFootInWorldFrame.x() = offsetInWorldFrame.x() * velScale;
  linearVelocityDesiredFootInWorldFrame.y() = offsetInWorldFrame.y() * velScale;
  linearVelocityDesiredFootInWorldFrame.z() = heightVel;

  double accScale = 0.0;
  if (!planeTrajectory_.evaluateDerivative(accScale, interpolationParameter, 2)) {
    return false;
  }
  double heightAcc = 0.0;
  if (!heightTrajectory_.evaluateDerivative(heightAcc, leg->getContactSchedule().getSwingPhase(), 2)) {
    return false;
  }
  linearAccelerationDesiredFootInWorldFrame.x() = offsetInWorldFrame.x() * accScale;
  linearAccelerationDesiredFootInWorldFrame.y() = offsetInWorldFrame.y() * accScale;
  linearAccelerationDesiredFootInWorldFrame.z() = heightAcc;

  return true;
}

Position SwingTrajectoryGeneratorStaticGait::getPositionDesiredFootOnTerrainToDesiredFootInControlFrame(
    const LegBase& leg, const Position& positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame) {
  const double interpolationParameter = getInterpolationPhase(leg);
  double desiredFootHeight = 0.0;
  heightTrajectory_.evaluate(desiredFootHeight, leg.getContactSchedule().getSwingPhase());

  const RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const Position positionHipOnTerrainToDesiredFootOnTerrainInWorldFrame =
      orientationWorldToControl.inverseRotate(positionFootOnTerrainAtLiftOffToDesiredFootOnTerrainInControlFrame);

  Vector normalToPlaneAtCurrentFootPositionInWorldFrame;
  terrain_.getNormal(positionHipOnTerrainToDesiredFootOnTerrainInWorldFrame, normalToPlaneAtCurrentFootPositionInWorldFrame);

  const Vector normalToPlaneAtCurrentFootPositionInControlFrame =
      orientationWorldToControl.rotate(normalToPlaneAtCurrentFootPositionInWorldFrame);
  const Position positionDesiredFootOnTerrainToDesiredFootInControlFrame =
      desiredFootHeight * Position(normalToPlaneAtCurrentFootPositionInControlFrame);

  return positionDesiredFootOnTerrainToDesiredFootInControlFrame;
}

double SwingTrajectoryGeneratorStaticGait::getHeadingComponentOfFootStep(double interpolationTime, double headingTarget) {
  double scale = 0.0;
  planeTrajectory_.evaluate(scale, interpolationTime);
  return headingTarget * scale;
}

double SwingTrajectoryGeneratorStaticGait::getLateralComponentOfFootStep(double interpolationTime, double lateralTarget) {
  double scale = 0.0;
  planeTrajectory_.evaluate(scale, interpolationTime);
  return lateralTarget * scale;
}

// Evaluate the interpolation phase relative to a given leg
double SwingTrajectoryGeneratorStaticGait::getInterpolationPhase(const LegBase& leg) {
  double swingPhase = 1.0;
  if (!((leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant))) {
    swingPhase = leg.getContactSchedule().getSwingPhase();
  }
  return robot_utils::mapTo01Range(swingPhase, leg.getStateLiftOff().getPhase(), 0.8);
}

// Evaluate the interpolation phase relative to a given leg
double SwingTrajectoryGeneratorStaticGait::getInterpolationPhaseForPlaneMotion(const LegBase& leg) {
  if (!((leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) &&
      (leg.getContactSchedule().getSwingPhase() < endPlaneMotionAtPhase_)) {
    double swingPhase = leg.getContactSchedule().getSwingPhase();
    //    return robot_utils::mapTo01Range(swingPhase, leg.getStateLiftOff().getPhase(), endPlaneMotionAtPhase_);
    return robot_utils::mapTo01Range(swingPhase, 0.0, endPlaneMotionAtPhase_);
  } else {
    return 1.0;
  }
}

// Evaluate the interpolation phase relative to a given leg
double SwingTrajectoryGeneratorStaticGait::getInterpolationPhaseForHeightMotion(const LegBase& leg) {
  if (!((leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) &&
      (leg.getContactSchedule().getSwingPhase() < endHeightMotionAtPhase_)) {
    double swingPhase = leg.getContactSchedule().getSwingPhase();
    //    return robot_utils::mapTo01Range(swingPhase, leg.getStateLiftOff().getPhase(), endHeightMotionAtPhase_);
    return robot_utils::mapTo01Range(swingPhase, 0.0, endHeightMotionAtPhase_);
  } else {
    return 1.0;
  }
}

} /* namespace loco */
