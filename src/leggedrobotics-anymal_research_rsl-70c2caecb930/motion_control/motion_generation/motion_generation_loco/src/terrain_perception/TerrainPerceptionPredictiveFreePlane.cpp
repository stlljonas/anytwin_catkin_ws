/*
 * TerrainPerceptionPredictiveFreePlane.cpp
 *
 *  Created on: Jan 09, 2019
 *      Author: Fabian Jenelten
 */

// loco
#include "loco/terrain_perception/TerrainPerceptionPredictiveFreePlane.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// logging
#include "signal_logger/signal_logger.hpp"

// message logger
#include <message_logger/log/log_messages.hpp>

using namespace message_logger::log;

// ToDo: clean up this class.
// ToDO: remove perceptive stuff from this class.
namespace loco {

TerrainPerceptionPredictiveFreePlane::TerrainPerceptionPredictiveFreePlane(TerrainModelPlane& terrainModel,
                                                       WholeBody& wholeBody,
                                                       HeadingGenerator& headingGenerator,
                                                       ControlFrameHeading referenceHeading,
                                                       EndEffectorFrame referenceEndEffectorContact,
                                                       bool updateTerrainWithDesiredFoothold):
  TerrainPerceptionBase("terrain_perception_freeplane"),
  terrainModel_(terrainModel),
  wholeBody_(wholeBody),
  headingGenerator_(headingGenerator),
  torso_(*wholeBody.getTorsoPtr()),
  legs_(*wholeBody.getLegsPtr()),
  positionWorldToFootOnPlaneInWorldFrame_(legs_.size(), loco::Position()),
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
  normalFilterRegaining_(),
  filterNormalAndPositionTimeConstantForRegaining_{0.02},
  positionFilterRegaining_(),
  referenceEndEffectorContact_(referenceEndEffectorContact),
  updateTerrainWithDesiredFoothold_(updateTerrainWithDesiredFoothold),
  enableLogging_(true) {

}

bool TerrainPerceptionPredictiveFreePlane::initialize(double dt) {
  timeStep_ = dt;

  for (auto leg: legs_) {
    gotFirstTouchDownOfFoot_[leg->getId()] = false;
    updateLocalMeasuresOfLeg(*leg);
  }

  /* Initialize normal and position vectors. It is assumed that the LocomotionController class initializes
   * TerrainModelFreePlane **before** TerrainPerceptionPredictiveFreePlane
   */
  updatePlaneEstimation();
  normalInWorldFrameFilterOutput_ = normalInWorldFrameFilterInput_;
  positionInWorldFrameFilterOutput_ = positionInWorldFrameFilterInput_;

  terrainModel_.setNormalandPositionInWorldFrame(normalInWorldFrameFilterOutput_, positionInWorldFrameFilterOutput_);

  // Filters to used to smooth local terrain plane.
  normalFilter_.setFilterParameters(dt, filterNormalTimeConstant_, filterNormalGain_, normalInWorldFrameFilterOutput_);
  positionFilter_.setFilterParameters(dt, filterPositionTimeConstant_, filterPositionGain_, positionInWorldFrameFilterOutput_);

  // Filter used to smooth local terrain plane if one of the legs is regaining.
  normalFilterRegaining_.setFilterParameters(dt, filterNormalAndPositionTimeConstantForRegaining_, filterNormalGain_, normalInWorldFrameFilterOutput_);
  positionFilterRegaining_.setFilterParameters(dt, filterNormalAndPositionTimeConstantForRegaining_, filterPositionGain_, positionInWorldFrameFilterOutput_);

  updateControlFrameOrigin();
  updateControlFrameAttitude();
  updateTorsoStateInControlFrame(torso_);
  updateWholeBodyStateInControlFrame(wholeBody_);

  return true;
}


bool TerrainPerceptionPredictiveFreePlane::addVariablesToLog(const std::string & ns) const {
  if (enableLogging_) {
    signal_logger::add(normalInWorldFrameFilterOutput_, "planeNormalInWorldFrame", "/loco/terrain_perception/", "m");
    signal_logger::add(positionInWorldFrameFilterOutput_, "planePositionInWorldFrame", "/loco/terrain_perception/", "m");
  }
  return true;
}

bool TerrainPerceptionPredictiveFreePlane::loadParameters(const TiXmlHandle& handle) {
  using namespace tinyxml_tools;

  TiXmlHandle hTerrainPerception(handle);
  if(!getChildHandle(hTerrainPerception, handle, "TerrainPerceptionFreePlane")) { return false; }
  if(!loadParameter(enableLogging_, hTerrainPerception, "logging")) { return false; }

  TiXmlHandle hPositionFilter(handle);
  if(!getChildHandle(hPositionFilter, hTerrainPerception, "PositionFilter")) { return false; }
  if(!loadParameter(filterPositionTimeConstant_, hPositionFilter, "time_constant")) { return false; }
  if(!loadParameter(filterPositionGain_, hPositionFilter, "gain")) { return false; }

  TiXmlHandle hNormalFilter(handle);
  if(!getChildHandle(hNormalFilter, hTerrainPerception, "NormalFilter")) { return false; }
  if(!loadParameter(filterNormalTimeConstant_, hNormalFilter, "time_constant")) { return false; }
  if(!loadParameter(filterNormalGain_, hNormalFilter, "gain")) { return false; }

  TiXmlHandle regaininglFilter(handle);
  if(!getChildHandle(regaininglFilter, hTerrainPerception, "RegainingFilter")) { return false; }
  if(!loadParameter(filterNormalAndPositionTimeConstantForRegaining_, regaininglFilter, "time_constant")) { return false; }

  return true;
}

void TerrainPerceptionPredictiveFreePlane::updateControlFrameOrigin() {
  torso_.getMeasuredStatePtr()->inControlFrame().setPositionWorldToControlInWorldFrame(Position::Zero());
}


void TerrainPerceptionPredictiveFreePlane::updateControlFrameAttitude() {
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
      MELO_FATAL(
          "[TerrainPerceptionPredictiveFreePlane::updateControlFrameAttitude] Invalid value of referenceHeading.");
      break;
  }
  currentHeadingDirectionInWorldFrame.z() = 0.0;

  // Compute the rotation from world to the heading direction of the control frame.
  RotationQuaternion orientationWorldToControlHeading;
  try {
    orientationWorldToControlHeading.setFromVectors(
        currentHeadingDirectionInWorldFrame.toImplementation(), unitX_);
  } catch (std::exception& e) {
    MELO_FATAL_STREAM("TerrainPerceptionPredictiveFreePlane::updateControlFrameAttitude(): " << e.what() << std::endl
                      << "axisX: " << Eigen::Vector3d::UnitX()
                      << "currentHeadingDirectionInWorldFrame: " << currentHeadingDirectionInWorldFrame);
  }

  RotationQuaternion orientationControlHeadingToControl;
  try {
    orientationControlHeadingToControl.setFromVectors(
        (orientationWorldToControlHeading.rotate(normalInWorldFrame)).toImplementation(), unitZ_);
  } catch (std::exception& e) {
    MELO_FATAL_STREAM("TerrainPerceptionPredictiveFreePlane::updateControlFrameAttitude(): " << e.what() << std::endl
                      << "axisZ: " << Eigen::Vector3d::UnitZ()
                      << " normalInHeadingControlFrame: " << orientationWorldToControlHeading.rotate(normalInWorldFrame));
  }

  /* Compute the rotation from world to control as:
   *    C_CW = C_CH * C_HW
   */
  const RotationQuaternion orientationWorldToControl(orientationControlHeadingToControl*orientationWorldToControlHeading);

  torso_.getMeasuredStatePtr()->inControlFrame().setOrientationWorldToControl(orientationWorldToControl);
  torso_.getMeasuredStatePtr()->inControlFrame().setPositionControlToBaseInControlFrame(orientationWorldToControl.rotate(torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame() - torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame()));
  torso_.getMeasuredStatePtr()->inControlFrame().setOrientationControlToBase(orientationWorldToBase*orientationWorldToControl.inverted());

  // Estimate the angular velocity of the control frame wrt the world frame.
  torso_.getMeasuredStatePtr()->inControlFrame().setAngularVelocityControlInWorldFrame(
      LocalAngularVelocity(orientationWorldToControl.boxMinus(orientationWorldToControlOld_)/timeStep_));
  orientationWorldToControlOld_ = orientationWorldToControl;
}


bool TerrainPerceptionPredictiveFreePlane::advance(double dt) {
  timeStep_ = dt;

  bool receiveNewInfo = false;
  bool allLegsGroundedAtLeastOnce = true;
  int legID = 0;
  bool isRegaining = false;

  // Update foot measurements.
  for (auto leg: legs_) {
    legID = leg->getId();

    if(leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactRecovery) {
      isRegaining = true;
    }

    switch(leg->getLimbStrategy().getLimbStrategyEnum()) {
      case LimbStrategyEnum::Support :
      case LimbStrategyEnum::ContactInvariant : {
        receiveNewInfo = true;
        gotFirstTouchDownOfFoot_[legID] = true;
        updateLocalMeasuresOfLeg(*leg);
      } break;

      case LimbStrategyEnum::Motion : {
        if (updateTerrainWithDesiredFoothold_) {
          receiveNewInfo = true;
          updateDesiredMeasuresOfLeg(*leg);
        }
      } break;

      case LimbStrategyEnum::ContactRecovery : {
        updateLocalMeasuresOfLeg(*leg);

        // Predict touch down location (at max limb extension).
        // Only if leg is regaining because no contact has been established yet.
        if (leg->getContactSchedule().shouldBeGrounded() && !leg->didTouchDownAtLeastOnceDuringStance() && leg->getContactSchedule().getStancePhase() > 0.02) {
          const double expectedTouchDownHeightInWorldFrame = (leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame().z() - leg->getLegProperties().getMaximumLimbExtension());
          positionWorldToFootOnPlaneInWorldFrame_[leg->getId()].z() = expectedTouchDownHeightInWorldFrame;
        }
      } break;

      // Remaining enums are: Init, SwingBumpedIntoObstacle
      default : break;
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

  // 2. filter.
  if (!isRegaining) {
    normalInWorldFrameFilterOutput_ = normalFilter_.advance(normalInWorldFrameFilterInput_);
    positionInWorldFrameFilterOutput_ = positionFilter_.advance(positionInWorldFrameFilterInput_);
    normalFilterRegaining_.reset(normalInWorldFrameFilterOutput_);
    positionFilterRegaining_.reset(positionInWorldFrameFilterOutput_);
  } else {
    normalInWorldFrameFilterOutput_ = normalFilterRegaining_.advance(normalInWorldFrameFilterInput_);
    positionInWorldFrameFilterOutput_ = positionFilterRegaining_.advance(positionInWorldFrameFilterInput_);
    normalFilter_.reset(normalInWorldFrameFilterOutput_);
    positionFilter_.reset(positionInWorldFrameFilterOutput_);
  }

  // 3.
  terrainModel_.setNormalandPositionInWorldFrame(normalInWorldFrameFilterOutput_, positionInWorldFrameFilterOutput_);

  // 4.
  updateControlFrameOrigin();
  updateControlFrameAttitude();
  updateTorsoStateInControlFrame(torso_);
  updateWholeBodyStateInControlFrame(wholeBody_);

  return true;
} // advance


void TerrainPerceptionPredictiveFreePlane::updateLocalMeasuresOfLeg(const loco::LegBase& leg) {
  const int legID = leg.getId();
  const auto& footBaseStateMeasured = leg.getFoot().getStateMeasured(TimePoint::Now, referenceEndEffectorContact_);
  positionWorldToFootOnPlaneInWorldFrame_[legID] = footBaseStateMeasured.getPositionWorldToEndEffectorInWorldFrame();
} // update local measures

void TerrainPerceptionPredictiveFreePlane::updateDesiredMeasuresOfLeg(const loco::LegBase& leg) {
  const int legID = leg.getId();
  const auto& footBaseStateDesired = leg.getFoot().getStateDesired(TimePoint::Now, referenceEndEffectorContact_);
  positionWorldToFootOnPlaneInWorldFrame_[legID] = footBaseStateDesired.getPositionWorldToFootholdInWorldFrame();
} // update local measures

void TerrainPerceptionPredictiveFreePlane::updatePlaneEstimation() {
  /* estimate the plane which best fits the most recent contact points of each foot in world frame
   * using least squares (pseudo inversion of the regressor matrix H)
   *
   * parameters       -> [a b d]^T
   * plane equation   -> z = d-ax-by
   * normal to plane  -> n = [a b 1]^T
   *
   * */
  Eigen::MatrixXd linearRegressor = Eigen::MatrixXd::Zero(4,3);
  Eigen::Vector4d measuredFootHeights = Eigen::Vector4d::Zero();

  linearRegressor << -positionWorldToFootOnPlaneInWorldFrame_[0].x(), -positionWorldToFootOnPlaneInWorldFrame_[0].y(), 1.0,
                     -positionWorldToFootOnPlaneInWorldFrame_[1].x(), -positionWorldToFootOnPlaneInWorldFrame_[1].y(), 1.0,
                     -positionWorldToFootOnPlaneInWorldFrame_[2].x(), -positionWorldToFootOnPlaneInWorldFrame_[2].y(), 1.0,
                     -positionWorldToFootOnPlaneInWorldFrame_[3].x(), -positionWorldToFootOnPlaneInWorldFrame_[3].y(), 1.0;
  measuredFootHeights << positionWorldToFootOnPlaneInWorldFrame_[0].z(),
                         positionWorldToFootOnPlaneInWorldFrame_[1].z(),
                         positionWorldToFootOnPlaneInWorldFrame_[2].z(),
                         positionWorldToFootOnPlaneInWorldFrame_[3].z();

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
} // update plane estimation


bool TerrainPerceptionPredictiveFreePlane::generateTerrainModelFromPointsInWorldFrame(
    const std::vector<Position>& pointsInWorldFrame,
    TerrainModelFreePlane& terrainModel) {
  const int numConstraints = pointsInWorldFrame.size();

  if (numConstraints < 3) {
    MELO_WARN_STREAM(
        "[TerrainPerceptionPredictiveFreePlane::generateTerainModelFromPointsInWorldFrame] Number of points in vector is " << numConstraints << ", but should be at least 3.");
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

    terrainModel.setNormalandPositionInWorldFrame(normalInWorldFrame,
                                                  positionInWorldFrame);
    return true;
  } else {
    MELO_WARN(
        "[TerrainPerceptionPredictiveFreePlane::generateTerainModelFromPointsInWorldFrame] Pseudoinversion returned false.");
    return false;
  }

}

void TerrainPerceptionPredictiveFreePlane::setUpdateTerrainWithDesiredFoothold(bool updateTerrainWithDesiredFoothold) noexcept {
  updateTerrainWithDesiredFoothold_ = updateTerrainWithDesiredFoothold;
}


} /* namespace loco */
