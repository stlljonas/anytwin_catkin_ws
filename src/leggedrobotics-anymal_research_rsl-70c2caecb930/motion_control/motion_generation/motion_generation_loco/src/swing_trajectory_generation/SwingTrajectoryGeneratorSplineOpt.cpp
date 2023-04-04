/*
 * SwingTrajectoryGeneratorSplineOpt.cpp
 *
 *  Created on: June 25, 2019
 *      Author: Fabian Jenelten
 */

// loco
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorSplineOpt.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// message logger
#include <message_logger/message_logger.hpp>

using namespace message_logger::color;
namespace loco {

using Spline = SwingTrajectoryGeneratorSplineOpt::Spline;
using SplineCurve = SwingTrajectoryGeneratorSplineOpt::SplineCurve;

SwingTrajectoryGeneratorSplineOpt::SwingTrajectoryGeneratorSplineOpt(
    WholeBody& wholeBody,
    TerrainModelBase& terrain,
    ContactScheduleZmp& contactSchedule) :
  SwingTrajectoryGeneratorModule(),
  wholeBody_(wholeBody),
  terrain_(terrain),
  contactSchedule_(contactSchedule),
  curveX_(wholeBody.getLegs().size(), Spline()),
  curveY_(wholeBody.getLegs().size(), Spline()),
  curveZ_(wholeBody.getLegs().size(), Spline()),
  swingParams_()
{
}

bool SwingTrajectoryGeneratorSplineOpt::loadParameters(const TiXmlHandle& handle) {
  MELO_DEBUG_STREAM(magenta << "[SwingTrajectoryGeneratorSplineOpt] " << blue << "Load parameters." << def)

  TiXmlHandle fpsHandle = handle;
  if (!tinyxml_tools::getChildHandle(fpsHandle, handle, "FootPlacementStrategy")) { return false; }

  TiXmlHandle swingTrajHandle = handle;
  if (!tinyxml_tools::getChildHandle(swingTrajHandle, fpsHandle, "SwingTrajectoryGenerator")) { return false; }

  std::vector<TiXmlElement*> gaitElements;
  if(!tinyxml_tools::getChildElements(gaitElements, swingTrajHandle, "Gait")) { return false; }
  if(!contact_schedule::loadGaitParameters(gaitElements, swingParams_, contactSchedule_.getMapGaitNameToId())) { return false; }

  return true;
}

bool SwingTrajectoryGeneratorSplineOpt::initialize(double dt) {
  return true;
}

bool SwingTrajectoryGeneratorSplineOpt::advance(double dt) {
  return true;
}


bool SwingTrajectoryGeneratorSplineOpt::compute(
    const Position& positionWorldToDesiredFootholdInWorldFrame,
    const Position& positionWorldToFootAtLiftOffInWorldFrame,
    unsigned int legId,
    double plannedSwingDuration) {
  const auto& swingParams = swingParams_.getParams(contactSchedule_.getActiveGaitName());
  constexpr double zero = 1.0e-3;

  /*********************************
   * Vertical swing leg trajectory *
   *********************************/
  std::vector<double> timesZ;
  std::vector<double> zValues;

  // Corner positions.
  const double heightLiftOff = positionWorldToFootAtLiftOffInWorldFrame.z();
  const double heightTouchDown = positionWorldToDesiredFootholdInWorldFrame.z();
  const double referenceHeight = std::fmax(heightLiftOff, heightTouchDown);

  // Normalized step.
  const double stepHeight = heightLiftOff - heightTouchDown;
  const double maxStepHeight = wholeBody_.getLegs().get(legId).getLegProperties().getMaximumLimbExtension() * 0.5;
  if (maxStepHeight < zero) {
    MELO_WARN_THROTTLE(1.0, "[SwingTrajectoryGeneratorSplineOpt::compute] Max limb extension is zero or negative.");
    return false;
  }
  const double normalizedStepHeight = std::fmin(std::fabs(stepHeight / maxStepHeight), 1.0);

  // Compute height trajectory.
  timesZ.reserve(swingParams.zTrajectoryScaledTimes_.size());
  zValues.reserve(swingParams.zTrajectoryScaledTimes_.size());
  for (auto i = 0u; i < swingParams.zTrajectoryScaledTimes_.size(); ++i) {
    double timeZ = swingParams.zTrajectoryScaledTimes_[i] * plannedSwingDuration;
    double valueZ = 0.0;
    double swingHeight = swingParams.zTrajectoryValues_[i];

    // Start position.
    if (i == 0u) {
      valueZ = heightLiftOff + swingHeight;
    }

    // End position.
    else if (i == swingParams.zTrajectoryScaledTimes_.size()-1u) {
      valueZ = heightTouchDown + swingHeight;
    }

    // Middle position.
    else {
      swingHeight *= (1.0 - normalizedStepHeight);
      valueZ = referenceHeight + swingHeight;
    }

    zValues.push_back(valueZ);
    timesZ.push_back(timeZ);
  }

  // Fit z trajectory.
  fitSpline(curveZ_[legId], timesZ, zValues, swingParams.zTrajectorySpeedInitial_, 0.0, swingParams.zTrajectorySpeedFinal_, 0.0);
  /***********************************/

  /***********************************
   * Horizontal Swing leg Trajectory *
   ***********************************/
  std::vector<double> timesXY;
  std::vector<double> xValues;
  std::vector<double> yValues;

  timesXY.reserve(2u);
  xValues.reserve(2u);
  yValues.reserve(2u);

  // Init point.
  timesXY.push_back(0.0);
  xValues.push_back(positionWorldToFootAtLiftOffInWorldFrame.x());
  yValues.push_back(positionWorldToFootAtLiftOffInWorldFrame.y());

  // End point.
  timesXY.push_back(plannedSwingDuration);
  xValues.push_back(positionWorldToDesiredFootholdInWorldFrame.x());
  yValues.push_back(positionWorldToDesiredFootholdInWorldFrame.y());

  // Initial and final velocity vectors.
  const Eigen::Vector3d vectorPreviousToDesiredFoothooldInWorldFrame = (positionWorldToDesiredFootholdInWorldFrame - positionWorldToFootAtLiftOffInWorldFrame).toImplementation();

  RotationQuaternion orientationPlaneToWorld;
  if (vectorPreviousToDesiredFoothooldInWorldFrame.squaredNorm() > zero*zero) {
    orientationPlaneToWorld.setFromVectors<Eigen::Vector3d>(Eigen::Vector3d::UnitX(), vectorPreviousToDesiredFoothooldInWorldFrame);
  } else {
    orientationPlaneToWorld = wholeBody_.getTorsoPtr()->getMeasuredState().inControlFrame().getOrientationWorldToControl().inverted();
  }
  const Eigen::Vector3d initialVelocityInPlaneFrame(swingParams.xyTrajectorySpeedInitial_, 0.0, 0.0);
  const Eigen::Vector3d finalVelocityInPlaneFrame(swingParams.xyTrajectorySpeedFinal_, 0.0, 0.0);
  Eigen::Vector3d initialVerticalVelocityInWorldFrame = orientationPlaneToWorld.rotate(initialVelocityInPlaneFrame);
  Eigen::Vector3d finalVerticalVelocityInWorldFrame = orientationPlaneToWorld.rotate(finalVelocityInPlaneFrame);

  // Change sing of initial and final vertical velocity.
  const double sign = (kickLegAhead(legId) ? 1.0 : -1.0 );
  initialVerticalVelocityInWorldFrame *= sign;
  finalVerticalVelocityInWorldFrame *= sign;

  // Fit xy trajectories.
  fitSpline(curveX_[legId], timesXY, xValues, initialVerticalVelocityInWorldFrame.x(), 0.0, finalVerticalVelocityInWorldFrame.x(), 0.0);
  fitSpline(curveY_[legId], timesXY, yValues, initialVerticalVelocityInWorldFrame.y(), 0.0, finalVerticalVelocityInWorldFrame.z(), 0.0);
  /*********************************/

  return true;
}

bool SwingTrajectoryGeneratorSplineOpt::getDesiredFootState(
    Position& positionWorldToDesiredFootInWorldFrame,
    LinearVelocity& linearVelocityDesiredFootInWorldFrame,
    LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
    const Position& positionWorldToDesiredFootholdInControlFrame,
    LegBase* leg, double dt)
{

  // Get initial and final end-effector position.
  const int legId = leg->getId();

  const auto& orientationWorldToControl = wholeBody_.getTorsoPtr()->getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const auto positionWorldToDesiredFootholdInWorldFrame = orientationWorldToControl.inverseRotate(positionWorldToDesiredFootholdInControlFrame);

  // Timing.
  double swingPhase = leg->getContactSchedule().getSwingPhase();
  robot_utils::mapTo01Range(swingPhase, 0.0, 1.0);
  const double plannedSwingDuration = leg->getContactSchedule().getSwingDuration();
  const double timeNow = swingPhase*plannedSwingDuration;

  if (plannedSwingDuration <= 0.0) {
    MELO_WARN_THROTTLE(1.0, "[SwingTrajectoryGeneratorSplineOpt::getDesiredFootState] Planned swing duration is zero or negative!");
    return false;
  }

  // Compute swing leg trajectory.
  if(!compute(positionWorldToDesiredFootholdInWorldFrame, leg->getPositionWorldToLastOrCurrentContactInWorldFrame(), legId, plannedSwingDuration)) {
    MELO_WARN_THROTTLE(1.0, "[SwingTrajectoryGeneratorSplineOpt::getDesiredFootState] Failed to compute swing leg trajectory!");
    return false;
  }

  // Read end-effector state from swing leg trajectory.
  if (!curveX_[legId].evaluate(positionWorldToDesiredFootInWorldFrame.x(), timeNow)) { return false; }
  if (!curveY_[legId].evaluate(positionWorldToDesiredFootInWorldFrame.y(), timeNow)) { return false; }
  if (!curveZ_[legId].evaluate(positionWorldToDesiredFootInWorldFrame.z(), timeNow)) { return false; }

  if (!curveX_[legId].evaluateDerivative(linearVelocityDesiredFootInWorldFrame.x(), timeNow, 1)) { return false; }
  if (!curveY_[legId].evaluateDerivative(linearVelocityDesiredFootInWorldFrame.y(), timeNow, 1)) { return false; }
  if (!curveZ_[legId].evaluateDerivative(linearVelocityDesiredFootInWorldFrame.z(), timeNow, 1)) { return false; }

  if (!curveX_[legId].evaluateDerivative(linearAccelerationDesiredFootInWorldFrame.x(), timeNow, 2)) { return false; }
  if (!curveY_[legId].evaluateDerivative(linearAccelerationDesiredFootInWorldFrame.y(), timeNow, 2)) { return false; }
  if (!curveZ_[legId].evaluateDerivative(linearAccelerationDesiredFootInWorldFrame.z(), timeNow, 2)) { return false; }

  return true;
}

void SwingTrajectoryGeneratorSplineOpt::fitSpline(
    Spline& curve, const std::vector<double>& times, const std::vector<double>& values,
    const double initialVelocity, const double initialAcceleration,
    const double finalVelocity, const double finalAcceleration) {
  curve.fitCurve(times, values, initialVelocity, initialAcceleration, finalVelocity, finalAcceleration);
}

const SplineCurve& SwingTrajectoryGeneratorSplineOpt::getSplineX() const {
  return curveX_;
}
const SplineCurve& SwingTrajectoryGeneratorSplineOpt::getSplineY() const {
  return curveY_;
}
const SplineCurve& SwingTrajectoryGeneratorSplineOpt::getSplineZ() const {
  return curveZ_;
}

bool SwingTrajectoryGeneratorSplineOpt::kickLegAhead(unsigned int legId) const {
  bool kickLegAhead = true;
  constexpr double zero = 1.0e-3;
  const double desHeadingVelocity = wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame().x();
  const double desLateralVelocity = wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame().y();
  const bool isHeadingVelDominant = std::fabs(desHeadingVelocity) >= std::fabs(desLateralVelocity);
  if (desHeadingVelocity > zero && isHeadingVelDominant) {
    kickLegAhead = (legId == 0u || legId == 1u);
  } else if(desHeadingVelocity < -zero && isHeadingVelDominant) {
    kickLegAhead = (legId == 2u || legId == 3u);
  } else if(desLateralVelocity > zero && !isHeadingVelDominant) {
    kickLegAhead = (legId == 0u || legId == 2u);
  } else if(desLateralVelocity < -zero && !isHeadingVelDominant) {
    kickLegAhead = (legId == 1u || legId == 3u);
  }
  return kickLegAhead;
}

} /* namespace loco */
