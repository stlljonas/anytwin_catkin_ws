/*
 * TerrainPerceptionFreePlane.cpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// logging
#include "signal_logger/signal_logger.hpp"

// message logger
#include <message_logger/log/log_messages.hpp>

namespace loco {

TerrainPerceptionFreePlane::TerrainPerceptionFreePlane(TerrainModelPlane& terrainModel, WholeBody& wholeBody,
                                                       HeadingGenerator& headingGenerator,
                                                       TerrainPerceptionFreePlane::EstimatePlaneInFrame estimatePlaneInFrame,
                                                       ControlFrameHeading referenceHeading, EndEffectorFrame referenceEndEffectorContact,
                                                       bool updateTerrainWithDesiredFoothold)
    : TerrainPerceptionBase("terrain_perception_freeplane"),
      terrainModel_(terrainModel),
      wholeBody_(wholeBody),
      headingGenerator_(headingGenerator),
      torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      estimatePlaneInFrame_(estimatePlaneInFrame),
      mostRecentPositionOfFoot_(legs_.size(), loco::Position()),
      lastWorldToBasePositionInWorldFrameForFoot_(legs_.size(), loco::Position()),
      lastWorldToBaseOrientationForFoot_(legs_.size(), loco::RotationQuaternion()),
      gotFirstTouchDownOfFoot_(legs_.size(), false),
      referenceHeading_(referenceHeading),
      orientationWorldToControlOld_(),
      timeStep_(0.0025),
      normalFilter_(),
      positionFilter_(),
      filterNormalTimeConstant_{0.05},
      filterPositionTimeConstant_{0.05},
      filterNormalGain_{1.0},
      filterPositionGain_{1.0},
      referenceEndEffectorContact_(referenceEndEffectorContact),
      updateTerrainWithDesiredFoothold_(updateTerrainWithDesiredFoothold) {}

bool TerrainPerceptionFreePlane::initialize(double dt) {
  timeStep_ = dt;

  for (auto leg : legs_) {
    gotFirstTouchDownOfFoot_[leg->getId()] = false;
    updateLocalMeasuresOfLeg(*leg);
  }

  /* Initialize normal and position vectors. It is assumed that the LocomotionController class initializes
   * TerrainModelFreePlane **before** TerrainPerceptionFreePlane
   */
  updatePlaneEstimation();
  normalInWorldFrameFilterOutput_ = normalInWorldFrameFilterInput_;
  positionInWorldFrameFilterOutput_ = positionInWorldFrameFilterInput_;

  terrainModel_.setNormalandPositionInWorldFrame(normalInWorldFrameFilterOutput_, positionInWorldFrameFilterOutput_);

  normalFilter_.setFilterParameters(dt, filterNormalTimeConstant_, filterNormalGain_, normalInWorldFrameFilterOutput_);
  positionFilter_.setFilterParameters(dt, filterPositionTimeConstant_, filterPositionGain_, positionInWorldFrameFilterOutput_);

  updateControlFrameOrigin();
  updateControlFrameAttitude();
  updateTorsoStateInControlFrame(torso_);
  updateWholeBodyStateInControlFrame(wholeBody_);

  return true;
}

bool TerrainPerceptionFreePlane::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(normalInWorldFrameFilterOutput_, "planeNormalInWorldFrame", "/loco/terrain_perception/", "m");
  signal_logger::add(positionInWorldFrameFilterOutput_, "planePositionInWorldFrame", "/loco/terrain_perception/", "m");
  return true;
}

bool TerrainPerceptionFreePlane::loadParameters(const TiXmlHandle& handle) {
  using tinyxml_tools::getChildHandle;
  using tinyxml_tools::loadParameter;

  TiXmlHandle hTerrainPerception(handle);
  if (!getChildHandle(hTerrainPerception, handle, "TerrainPerceptionFreePlane")) {
    return false;
  }

  TiXmlHandle hPositionFilter(handle);
  if (!getChildHandle(hPositionFilter, hTerrainPerception, "PositionFilter")) {
    return false;
  }
  if (!loadParameter(filterPositionTimeConstant_, hPositionFilter, "time_constant", 0.05)) {
    return false;
  }
  if (!loadParameter(filterPositionGain_, hPositionFilter, "gain", 1.0)) {
    return false;
  }

  TiXmlHandle hNormalFilter(handle);
  if (!getChildHandle(hNormalFilter, hTerrainPerception, "NormalFilter")) {
    return false;
  }
  if (!loadParameter(filterNormalTimeConstant_, hNormalFilter, "time_constant", 0.05)) {
    return false;
  }
  if (!loadParameter(filterNormalGain_, hNormalFilter, "gain", 1.0)) {
    return false;
  }

  return true;
}

void TerrainPerceptionFreePlane::updateControlFrameOrigin() {
  torso_.getMeasuredStatePtr()->inControlFrame().setPositionWorldToControlInWorldFrame(Position::Zero());
}

void TerrainPerceptionFreePlane::updateControlFrameAttitude() {
  const RotationQuaternion& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();

  loco::Vector normalInWorldFrame;
  terrainModel_.getNormal(positionZero_, normalInWorldFrame);

  // Get current heading direction.
  loco::Vector currentHeadingDirectionInWorldFrame;
  switch (referenceHeading_) {
    case (ControlFrameHeading::Hips): {
      headingGenerator_.getTorsoHeadingDirectionInWorldFrame(currentHeadingDirectionInWorldFrame);
    } break;

    case (ControlFrameHeading::Feet): {
      headingGenerator_.getLegsHeadingDirectionFromCurrentFeetInWorldFrame(currentHeadingDirectionInWorldFrame);
    } break;

    default:
      MELO_FATAL("[TerrainPerceptionFreePlane::updateControlFrameAttitude] Invalid value of referenceHeading.");
      break;
  }
  currentHeadingDirectionInWorldFrame.z() = 0.0;

  // Compute the rotation from world to the heading direction of the control frame.
  RotationQuaternion orientationWorldToControlHeading;
  try {
    orientationWorldToControlHeading.setFromVectors(currentHeadingDirectionInWorldFrame.toImplementation(), unitX_);
  } catch (std::exception& e) {
    MELO_FATAL_STREAM("TerrainPerceptionFreePlane::updateControlFrameAttitude(): " << e.what() << std::endl
                                                                                   << "axisX: " << Eigen::Vector3d::UnitX()
                                                                                   << "currentHeadingDirectionInWorldFrame: "
                                                                                   << currentHeadingDirectionInWorldFrame);
  }

  RotationQuaternion orientationControlHeadingToControl;
  try {
    orientationControlHeadingToControl.setFromVectors((orientationWorldToControlHeading.rotate(normalInWorldFrame)).toImplementation(),
                                                      unitZ_);
  } catch (std::exception& e) {
    MELO_FATAL_STREAM("TerrainPerceptionFreePlane::updateControlFrameAttitude(): "
                      << e.what() << std::endl
                      << "axisZ: " << Eigen::Vector3d::UnitZ()
                      << " normalInHeadingControlFrame: " << orientationWorldToControlHeading.rotate(normalInWorldFrame));
  }

  /* Compute the rotation from world to control as:
   *    C_CW = C_CH * C_HW
   */
  const RotationQuaternion orientationWorldToControl(orientationControlHeadingToControl * orientationWorldToControlHeading);

  torso_.getMeasuredStatePtr()->inControlFrame().setOrientationWorldToControl(orientationWorldToControl);
  torso_.getMeasuredStatePtr()->inControlFrame().setPositionControlToBaseInControlFrame(
      orientationWorldToControl.rotate(torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame() -
                                       torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame()));
  torso_.getMeasuredStatePtr()->inControlFrame().setOrientationControlToBase(orientationWorldToBase * orientationWorldToControl.inverted());

  // Estimate the angular velocity of the control frame wrt the world frame.
  torso_.getMeasuredStatePtr()->inControlFrame().setAngularVelocityControlInWorldFrame(
      LocalAngularVelocity(orientationWorldToControl.boxMinus(orientationWorldToControlOld_) / timeStep_));
  orientationWorldToControlOld_ = orientationWorldToControl;
}

bool TerrainPerceptionFreePlane::advance(double dt) {
  timeStep_ = dt;

  bool receiveNewInfo = false;
  bool allLegsGroundedAtLeastOnce = true;
  int legID = 0;

  // Update foot measurements.
  for (auto leg : legs_) {
    legID = leg->getId();

    switch (leg->getLimbStrategy().getLimbStrategyEnum()) {
      case LimbStrategyEnum::Support:
      case LimbStrategyEnum::ContactInvariant:
      case LimbStrategyEnum::ContactRecovery: {
        receiveNewInfo = true;
        gotFirstTouchDownOfFoot_[legID] = true;
        updateLocalMeasuresOfLeg(*leg);
      } break;

      case LimbStrategyEnum::Motion: {
        if (updateTerrainWithDesiredFoothold_) {
          receiveNewInfo = true;
          updateDesiredMeasuresOfLeg(*leg);
        }
      } break;

      // Remaining enums are: Init, SwingBumpedIntoObstacle
      default:
        break;
    }

    allLegsGroundedAtLeastOnce &= gotFirstTouchDownOfFoot_[legID];
  }

  /* Sequence:
   * 1. update (if needed) normal and position
   * 2. filter
   * 3. set to terrain model
   * 4. update control frame
   */
  // 1. Update terrain model properties (if necessary) based on current estimation
  if (receiveNewInfo && allLegsGroundedAtLeastOnce) {
    updatePlaneEstimation();
  }

  // 2. filter
  normalInWorldFrameFilterOutput_ = normalFilter_.advance(normalInWorldFrameFilterInput_);
  positionInWorldFrameFilterOutput_ = positionFilter_.advance(positionInWorldFrameFilterInput_);

  // 3.
  terrainModel_.setNormalandPositionInWorldFrame(normalInWorldFrameFilterOutput_, positionInWorldFrameFilterOutput_);

  // 4.
  updateControlFrameOrigin();
  updateControlFrameAttitude();
  updateTorsoStateInControlFrame(torso_);
  updateWholeBodyStateInControlFrame(wholeBody_);

  return true;
}  // advance

void TerrainPerceptionFreePlane::updateLocalMeasuresOfLeg(const loco::LegBase& leg) {
  const int legID = leg.getId();

  const auto& footBaseStateMeasured = leg.getFoot().getStateMeasured(TimePoint::Now, referenceEndEffectorContact_);

  switch (estimatePlaneInFrame_) {
    case (EstimatePlaneInFrame::World): {
      mostRecentPositionOfFoot_[legID] = footBaseStateMeasured.getPositionWorldToEndEffectorInWorldFrame();
    } break;

    case (EstimatePlaneInFrame::Base): {
      mostRecentPositionOfFoot_[legID] = footBaseStateMeasured.getPositionBaseToEndEffectorInBaseFrame();
      lastWorldToBasePositionInWorldFrameForFoot_[legID] = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
      lastWorldToBaseOrientationForFoot_[legID] = torso_.getMeasuredState().getOrientationWorldToBase();
    } break;

    default: {
    error:
      throw std::out_of_range("[TerrainPerceptionFreePlane::updateLocalMeasuresOfLeg] Index out of range ...");
    } break;

  }  // switch
}  // update local measures

void TerrainPerceptionFreePlane::updateDesiredMeasuresOfLeg(const loco::LegBase& leg) {
  const int legID = leg.getId();

  const auto& footBaseStateDesired = leg.getFoot().getStateDesired(TimePoint::Now, referenceEndEffectorContact_);

  switch (estimatePlaneInFrame_) {
    case (EstimatePlaneInFrame::World): {
      mostRecentPositionOfFoot_[legID] = footBaseStateDesired.getPositionWorldToFootholdInWorldFrame();
    } break;

    case (EstimatePlaneInFrame::Base): {
      mostRecentPositionOfFoot_[legID] = torso_.getMeasuredState().getOrientationWorldToBase().rotate(
          footBaseStateDesired.getPositionWorldToFootholdInWorldFrame() - torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame());
      lastWorldToBasePositionInWorldFrameForFoot_[legID] = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
      lastWorldToBaseOrientationForFoot_[legID] = torso_.getMeasuredState().getOrientationWorldToBase();
    } break;

    default: {
    error:
      throw std::out_of_range("[TerrainPerceptionFreePlane::updateDesiredPositionOfLeg] Index out of range ...");
    } break;

  }  // switch
}  // update local measures

void TerrainPerceptionFreePlane::homogeneousTransformFromBaseToWorldFrame(loco::Position& position, int footID) {
  position = lastWorldToBasePositionInWorldFrameForFoot_[footID] + lastWorldToBaseOrientationForFoot_[footID].inverseRotate(position);
}

void TerrainPerceptionFreePlane::updatePlaneEstimation() {
  /* estimate the plane which best fits the most recent contact points of each foot in world frame
   * using least squares (pseudo inversion of the regressor matrix H)
   *
   * parameters       -> [a b d]^T
   * plane equation   -> z = d-ax-by
   * normal to plane  -> n = [a b 1]^T
   *
   * */
  Eigen::MatrixXd linearRegressor = Eigen::MatrixXd::Zero(4, 3);
  Eigen::Vector4d measuredFootHeights = Eigen::Vector4d::Zero();

  std::vector<loco::Position> mostRecenPositionOfFootInWorldFrame(legs_.size(), loco::Position());

  for (auto leg : legs_) {
    const auto legId = leg->getId();

    // Take location of stance feet and desired foothold of swing feet.
    mostRecenPositionOfFootInWorldFrame[legId] = mostRecentPositionOfFoot_[legId];
    if (estimatePlaneInFrame_ == EstimatePlaneInFrame::Base) {
      homogeneousTransformFromBaseToWorldFrame(mostRecenPositionOfFootInWorldFrame[legId], legId);
    }
  }

  linearRegressor << -mostRecenPositionOfFootInWorldFrame[0].x(), -mostRecenPositionOfFootInWorldFrame[0].y(), 1.0,
      -mostRecenPositionOfFootInWorldFrame[1].x(), -mostRecenPositionOfFootInWorldFrame[1].y(), 1.0,
      -mostRecenPositionOfFootInWorldFrame[2].x(), -mostRecenPositionOfFootInWorldFrame[2].y(), 1.0,
      -mostRecenPositionOfFootInWorldFrame[3].x(), -mostRecenPositionOfFootInWorldFrame[3].y(), 1.0;
  measuredFootHeights << mostRecenPositionOfFootInWorldFrame[0].z(), mostRecenPositionOfFootInWorldFrame[1].z(),
      mostRecenPositionOfFootInWorldFrame[2].z(), mostRecenPositionOfFootInWorldFrame[3].z();

  // Solve least squares problem.
  const Eigen::Vector3d parameters = linearRegressor.colPivHouseholderQr().solve(measuredFootHeights);

  /* Find a point on the plane. From z = d-ax-by, it is easy to find that p = [0 0 d]
   * is on the plane
   */
  positionInWorldFrameFilterInput_ << 0.0, 0.0, parameters(2);

  /* From the assumption that the normal has always unit z-component,
   * its norm will always be greater than zero
   */
  normalInWorldFrameFilterInput_ << parameters(0), parameters(1), 1.0;
}  // update plane estimation

bool TerrainPerceptionFreePlane::generateTerrainModelFromPointsInWorldFrame(const std::vector<Position>& pointsInWorldFrame,
                                                                            TerrainModelFreePlane& terrainModel) {
  const int numConstraints = pointsInWorldFrame.size();

  if (numConstraints < 3) {
    MELO_WARN_STREAM("[TerrainPerceptionFreePlane::generateTerainModelFromPointsInWorldFrame] Number of points in vector is "
                     << numConstraints << ", but should be at least 3.");
    return false;
  }

  Eigen::MatrixXd linearRegressor;
  Eigen::VectorXd measuredFootHeights;
  Eigen::Vector3d parameters;

  linearRegressor.resize(numConstraints, 3);
  measuredFootHeights.resize(numConstraints);

  linearRegressor.setZero();
  measuredFootHeights.setZero();
  parameters.setZero();

  // Construct the minimization problem
  int rowIndex = 0;
  for (const auto& point : pointsInWorldFrame) {
    //    linearRegressor << -point.x(), -point.y(), 1;

    linearRegressor(rowIndex, 0) = -point.x();
    linearRegressor(rowIndex, 1) = -point.y();
    linearRegressor(rowIndex, 2) = 1.0;

    measuredFootHeights(rowIndex) = point.z();

    rowIndex++;
  }

  if (kindr::pseudoInverse(linearRegressor, linearRegressor)) {
    /* solve least squares problem */
    parameters = linearRegressor * measuredFootHeights;

    const Position positionInWorldFrame(0.0, 0.0, parameters(2));
    Vector normalInWorldFrame(parameters(0), parameters(1), 1.0);
    //    normalInWorldFrame.normalize();

    terrainModel.setNormalandPositionInWorldFrame(normalInWorldFrame, positionInWorldFrame);
    return true;
  } else {
    MELO_WARN("[TerrainPerceptionFreePlane::generateTerainModelFromPointsInWorldFrame] Pseudoinversion returned false.");
    return false;
  }
}

void TerrainPerceptionFreePlane::setUpdateTerrainWithDesiredFoothold(bool updateTerrainWithDesiredFoothold) noexcept {
  updateTerrainWithDesiredFoothold_ = updateTerrainWithDesiredFoothold;
}

} /* namespace loco */
