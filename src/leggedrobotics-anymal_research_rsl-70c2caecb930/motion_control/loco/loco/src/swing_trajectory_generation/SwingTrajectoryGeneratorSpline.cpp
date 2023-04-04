/*
 * SwingTrajectoryGeneratorSpline.cpp
 *
 *  Created on: May 30, 2016
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorSpline.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace loco {

using Spline = SwingTrajectoryGeneratorSpline::Spline;
using SplineCurve = SwingTrajectoryGeneratorSpline::SplineCurve;

SwingTrajectoryGeneratorSpline::SwingTrajectoryGeneratorSpline(WholeBody& wholeBody, TerrainModelBase& terrain)
    : wholeBody_(wholeBody),
      terrain_(terrain),
      swingHeightScaledTimes_(),
      swingHeightValues_(),
      nominalHeightOverStep_(0.0),
      heightTrajectorySpeedInitial_(0.0),
      heightTrajectorySpeedFinal_(0.0),
      curveX_(wholeBody.getLegs().size(), Spline()),
      curveY_(wholeBody.getLegs().size(), Spline()),
      curveZ_(wholeBody.getLegs().size(), Spline()) {}

bool SwingTrajectoryGeneratorSpline::loadParameters(const TiXmlHandle& handle) {
  // Get handles.
  TiXmlHandle fpsHandle = handle;
  if (!tinyxml_tools::getChildHandle(fpsHandle, handle, "FootPlacementStrategy")) {
    return false;
  }

  TiXmlHandle swingTrajHandle = handle;
  if (!tinyxml_tools::getChildHandle(swingTrajHandle, fpsHandle, "SwingTrajectoryGenerator")) {
    return false;
  }

  TiXmlHandle linearInterpolationHandle = handle;
  if (!tinyxml_tools::getChildHandle(linearInterpolationHandle, swingTrajHandle, "LinearInterpolation")) {
    return false;
  }

  TiXmlHandle heightTrajHandle = handle;
  if (!tinyxml_tools::getChildHandle(heightTrajHandle, linearInterpolationHandle, "HeightTrajectory")) {
    return false;
  }

  TiXmlHandle heightTrajVelocityHandle = handle;
  if (!tinyxml_tools::getChildHandle(heightTrajVelocityHandle, linearInterpolationHandle, "HeightVelocity")) {
    return false;
  }

  // Load nominal height parameter.
  if (!tinyxml_tools::loadParameter(nominalHeightOverStep_, swingTrajHandle, "TrajectoryHeightOverStep")) {
    return false;
  }

  // Load height trajectory initial and final velocity.
  if (!tinyxml_tools::loadParameter(heightTrajectorySpeedInitial_, heightTrajVelocityHandle, "initial")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(heightTrajectorySpeedFinal_, heightTrajVelocityHandle, "final")) {
    return false;
  }

  // Get child elements.
  std::vector<TiXmlElement*> swingHeightElements;
  if (!tinyxml_tools::getChildElements(swingHeightElements, heightTrajHandle, "Knot")) {
    return false;
  }

  // Parse the child elements.
  swingHeightScaledTimes_.clear();
  swingHeightValues_.clear();

  swingHeightScaledTimes_.reserve(swingHeightElements.size());
  swingHeightValues_.reserve(swingHeightElements.size());
  for (const auto& knot : swingHeightElements) {
    double knotTime = 0.0;
    if (!tinyxml_tools::loadParameter(knotTime, knot, "t")) {
      return false;
    }
    double knotValue = 0.0;
    if (!tinyxml_tools::loadParameter(knotValue, knot, "v")) {
      return false;
    }

    swingHeightScaledTimes_.push_back(knotTime);
    swingHeightValues_.push_back(knotValue);
  }

  // MELO_INFO_STREAM("[SwingTrajectoryGeneratorSpline::loadParameters] Done loading parameters.");

  return true;
}

// Evaluate the interpolation phase relative to a given leg
double SwingTrajectoryGeneratorSpline::getInterpolationPhase(const LegBase& leg) {
  double swingPhase = 1.0;
  if ((leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Motion) ||
      (leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactRecovery)) {
    swingPhase = leg.getContactSchedule().getSwingPhase();
  }
  //  return robot_utils::mapTo01Range(swingPhase, leg.getStateLiftOff().getPhase(), 1.0);
  // fixme! the commented code had issues related to the reset of state lift off
  return robot_utils::mapTo01Range(swingPhase, 0.0, 1.0);
}

double SwingTrajectoryGeneratorSpline::getEffectiveSwingDuration(const LegBase& leg) {
  // This is accounting for late lift-off.
  return (1.0 - leg.getStateLiftOff().getPhase()) * leg.getContactSchedule().getSwingDuration();
}

bool SwingTrajectoryGeneratorSpline::compute(const Position& positionWorldToDesiredFootholdInWorldFrame,
                                             const Position& positionWorldToFootAtLiftOffInWorldFrame, unsigned int legId,
                                             double plannedSwingDuration) {
  std::vector<double> timesXY;
  std::vector<double> xValues;
  std::vector<double> yValues;

  timesXY.reserve(2);
  xValues.reserve(2);
  yValues.reserve(2);

  // Init point.
  timesXY.push_back(0.0);
  xValues.push_back(positionWorldToFootAtLiftOffInWorldFrame.x());
  yValues.push_back(positionWorldToFootAtLiftOffInWorldFrame.y());

  // End point.
  timesXY.push_back(plannedSwingDuration);
  xValues.push_back(positionWorldToDesiredFootholdInWorldFrame.x());
  yValues.push_back(positionWorldToDesiredFootholdInWorldFrame.y());

  // Fit xy trajectories.
  fitSpline(curveX_[legId], timesXY, xValues, 0.0, 0.0, 0.0, 0.0);
  fitSpline(curveY_[legId], timesXY, yValues, 0.0, 0.0, 0.0, 0.0);

  std::vector<double> timesZ;
  std::vector<double> zValues;

  // Avoid collision with the terrain by increasing the nominal step height
  const double heightLiftOff = positionWorldToFootAtLiftOffInWorldFrame.z();
  const double heightTouchDown = positionWorldToDesiredFootholdInWorldFrame.z();

  double obstacleHeight;
  if (!terrain_.getMaxHeightBetweenTwoPoints(positionWorldToFootAtLiftOffInWorldFrame, positionWorldToDesiredFootholdInWorldFrame,
                                             obstacleHeight)) {
    MELO_WARN_THROTTLE(1.0, "[SwingTrajectoryGeneratorSpline::compute] Failed to get max height between two points!");
    return false;
  }

  timesZ.reserve(swingHeightScaledTimes_.size());
  zValues.reserve(swingHeightScaledTimes_.size());
  double positionX = 0.0;
  double positionY = 0.0;
  for (size_t i = 0; i < swingHeightScaledTimes_.size(); ++i) {
    double timeZ = swingHeightScaledTimes_[i] * plannedSwingDuration;
    if (!curveX_[legId].evaluate(positionX, timeZ)) {
      return false;
    }
    if (!curveY_[legId].evaluate(positionY, timeZ)) {
      return false;
    }
    Position positionWorldToFootInWorldFrame(positionX, positionY, 0.0);
    double swingHeight = swingHeightValues_[i];

    if (i == 0) {
      positionWorldToFootInWorldFrame.z() = heightLiftOff + swingHeight;
    } else if (i == swingHeightScaledTimes_.size() - 1) {
      positionWorldToFootInWorldFrame.z() = heightTouchDown + swingHeight;
    } else {
      const double referenceHeight = std::fmax(heightLiftOff, heightTouchDown);
      double maxHeightAboveReference = obstacleHeight - referenceHeight;
      const double footClearance = swingHeight - maxHeightAboveReference;
      if (footClearance < nominalHeightOverStep_) {
        swingHeight = maxHeightAboveReference + nominalHeightOverStep_;
      }

      positionWorldToFootInWorldFrame.z() = referenceHeight + swingHeight;
    }

    zValues.push_back(positionWorldToFootInWorldFrame.z());
    timesZ.push_back(timeZ);
  }

  // Fit z trajectory.
  fitSpline(curveZ_[legId], timesZ, zValues, heightTrajectorySpeedInitial_, 0.0, heightTrajectorySpeedFinal_, 0.0);

  return true;
}

bool SwingTrajectoryGeneratorSpline::getDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame,
                                                         LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                                                         LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                                                         const Position& positionWorldToDesiredFootholdInControlFrame, LegBase* leg,
                                                         double dt) {
  // Get initial and final end-effector position.
  const int legId = leg->getId();
  const auto& orientationWorldToControl = wholeBody_.getTorsoPtr()->getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const auto positionWorldToDesiredFootholdInWorldFrame =
      orientationWorldToControl.inverseRotate(positionWorldToDesiredFootholdInControlFrame);
  const auto& positionWorldToFootAtLiftOffInWorldFrame = leg->getPositionWorldToLastOrCurrentContactInWorldFrame();

  // Timing.
  const double swingPhase = getInterpolationPhase(*leg);
  const double plannedSwingDuration = leg->getContactSchedule().getSwingDuration();
  const double timeNow = swingPhase * plannedSwingDuration;
  if (plannedSwingDuration <= 0.0) {
    MELO_WARN_THROTTLE(1.0, "[SwingTrajectoryGeneratorSpline::getDesiredFootState] Planned swing duration is zero or negative!");
    return false;
  }

  // Compute swing leg trajectory.
  if (!compute(positionWorldToDesiredFootholdInWorldFrame, positionWorldToFootAtLiftOffInWorldFrame, legId, plannedSwingDuration)) {
    MELO_WARN_THROTTLE(1.0, "[SwingTrajectoryGeneratorSpline::getDesiredFootState] Failed to compute swing leg trajectory!");
    return false;
  }

  // Read end-effector state from swing leg trajectory.
  if (!curveX_[legId].evaluate(positionWorldToDesiredFootInWorldFrame.x(), timeNow)) {
    return false;
  }
  if (!curveY_[legId].evaluate(positionWorldToDesiredFootInWorldFrame.y(), timeNow)) {
    return false;
  }
  if (!curveZ_[legId].evaluate(positionWorldToDesiredFootInWorldFrame.z(), timeNow)) {
    return false;
  }

  if (!curveX_[legId].evaluateDerivative(linearVelocityDesiredFootInWorldFrame.x(), timeNow, 1)) {
    return false;
  }
  if (!curveY_[legId].evaluateDerivative(linearVelocityDesiredFootInWorldFrame.y(), timeNow, 1)) {
    return false;
  }
  if (!curveZ_[legId].evaluateDerivative(linearVelocityDesiredFootInWorldFrame.z(), timeNow, 1)) {
    return false;
  }

  if (!curveX_[legId].evaluateDerivative(linearAccelerationDesiredFootInWorldFrame.x(), timeNow, 2)) {
    return false;
  }
  if (!curveY_[legId].evaluateDerivative(linearAccelerationDesiredFootInWorldFrame.y(), timeNow, 2)) {
    return false;
  }
  if (!curveZ_[legId].evaluateDerivative(linearAccelerationDesiredFootInWorldFrame.z(), timeNow, 2)) {
    return false;
  }

  return true;
}

void SwingTrajectoryGeneratorSpline::fitSpline(Spline& curve, const std::vector<double>& times, const std::vector<double>& values,
                                               const double initialVelocity, const double initialAcceleration, const double finalVelocity,
                                               const double finalAcceleration) {
  curve.fitCurve(times, values, initialVelocity, initialAcceleration, finalVelocity, finalAcceleration);
}

const SplineCurve& SwingTrajectoryGeneratorSpline::getSplineX() const {
  return curveX_;
}
const SplineCurve& SwingTrajectoryGeneratorSpline::getSplineY() const {
  return curveY_;
}
const SplineCurve& SwingTrajectoryGeneratorSpline::getSplineZ() const {
  return curveZ_;
}

} /* namespace loco */
