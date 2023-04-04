/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Declaration of torso trajectory generator for WBC test controller
 * @date    Jul 15, 2019
 */

// anymal_ctrl_test_whole_body_control
#include "anymal_ctrl_test_whole_body_control/torso_control/TorsoControlTestWholeBodyController.hpp"

namespace anymal_ctrl_test_whole_body_control {

TorsoControlTestWholeBodyController::TorsoControlTestWholeBodyController(loco::TorsoBase& torso, loco::HeadingGenerator& headingGenerator,
                                                                         loco::TerrainModelBase& terrain)
    : torso_(torso),
      headingGenerator_(headingGenerator),
      terrain_(terrain),
      time_(0.0),
      positionSchedule_(),
      orientationSchedule_(),
      restDuration_(1.0),
      defaultTorsoHeight_(std::string{"Test WBC - Base motion : Default base height [s]"}, 0.52, 0.3, 0.65),
      rampDuration_(std::string{"Test WBC - Base motion : Ramp duration [s]"}, 0.5, 0.1, 10.0) {
  // Add null trajectory
  positionSchedule_.addProfile(new robot_utils::ProfileStep<loco::Position>(loco::Position::Zero(), 0.0));
  // orientationSchedule_.addProfile(new robot_utils::ProfileStep<RotationQuaternion>(RotationQuaternion(), 0.0));
  addParametersToHandler("");
}

bool TorsoControlTestWholeBodyController::initialize(double /*dt*/) {
  return true;
}

bool TorsoControlTestWholeBodyController::advance(double dt) {
  const loco::RotationQuaternion& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  const loco::Position positionControlToTargetBaseInWorldFrame;
  loco::Position positionControlToTargetBaseInControlFrame;
  loco::LinearVelocity linearVelocityBaseInControlFrame;

  const loco::RotationQuaternion orientationControlToDesiredBase = torso_.getDesiredState().getDesiredOrientationOffset();
  const loco::LocalAngularVelocity angularVelocityBaseInControlFrame(0.0, 0.0, 0.0);

  if (time_ >= positionSchedule_.getDuration() || positionSchedule_.getDuration() <= 0.0) {
    positionControlToTargetBaseInControlFrame = orientationWorldToControl.rotate(defaultTorsoPositionInWorldFrameAtStart_);
    linearVelocityBaseInControlFrame = loco::LinearVelocity(0.0, 0.0, 0.0);
    isScheduleFinished_ = true;
  } else {
    positionControlToTargetBaseInControlFrame =
        orientationWorldToControl.rotate(defaultTorsoPositionInWorldFrameAtStart_) + positionSchedule_.getValue(time_);
    linearVelocityBaseInControlFrame = loco::LinearVelocity(positionSchedule_.getValueFirstDerivative(time_));
  }

  // set control references
  torso_.getDesiredStatePtr()->setPositionControlToTargetInControlFrame(positionControlToTargetBaseInControlFrame);
  torso_.getDesiredStatePtr()->setPositionWorldToBaseInWorldFrame(
      torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame() +
      orientationWorldToControl.inverseRotate(positionControlToTargetBaseInControlFrame));
  torso_.getDesiredStatePtr()->setOrientationControlToBase(orientationControlToDesiredBase);

  torso_.getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(linearVelocityBaseInControlFrame);
  torso_.getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(angularVelocityBaseInControlFrame);

  // set computed control errors
  torso_.getDesiredStatePtr()->setPositionErrorInControlFrame(
      torso_.getDesiredState().getPositionControlToTargetInControlFrame() -
      torso_.getMeasuredState().inControlFrame().getPositionControlToBaseInControlFrame());
  torso_.getDesiredStatePtr()->setLinearVelocityErrorInControlFrame(
      torso_.getDesiredState().getLinearVelocityTargetInControlFrame() -
      torso_.getMeasuredState().inControlFrame().getLinearVelocityBaseInControlFrame());

  time_ += dt;
  return true;
}

bool TorsoControlTestWholeBodyController::loadParameters(const TiXmlHandle& /*handle*/) {
  return true;
}

void TorsoControlTestWholeBodyController::clearTrajectory() {
  time_ = 0.0;
  isScheduleFinished_ = false;

  // Clear schedule
  positionSchedule_.clear();
  positionSchedule_.addProfile(new robot_utils::ProfileStep<loco::Position>(loco::Position::Zero(), 0.0));
  orientationSchedule_.clear();
  // orientationSchedule_.addProfile(new robot_utils::ProfileStep<RotationQuaternion>(RotationQuaternion(), 0.0));
}

void TorsoControlTestWholeBodyController::computeTrajectory() {
  time_ = 0.0;
  isScheduleFinished_ = false;

  // Add a ramp from current to default torso position
  defaultTorsoPositionInWorldFrameAtStart_ = getDefaultTorsoPositionInWorldFrame(defaultTorsoHeight_.getValue());
  auto positionDefaultToMeasTorsoInWorldFrame =
      torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame() - defaultTorsoPositionInWorldFrameAtStart_;
  positionSchedule_.addProfile(
      new robot_utils::ProfileRamp<loco::Position>(positionDefaultToMeasTorsoInWorldFrame, loco::Position::Zero(), initialRampDuration_));

  // Generate torso position trajectory according to parameters
  std::vector<loco::Position> desiredPosSetpoints;
  desiredPosSetpoints.emplace_back(0.0, 0.0, 0.0);
  // Heading offsets
  desiredPosSetpoints.emplace_back(minPosOffset_.x(), 0.0, 0.0);
  desiredPosSetpoints.emplace_back(maxPosOffset_.x(), 0.0, 0.0);
  // Lateral offsets
  desiredPosSetpoints.emplace_back(0.0, minPosOffset_.y(), 0.0);
  desiredPosSetpoints.emplace_back(0.0, maxPosOffset_.y(), 0.0);
  // Height offsets
  desiredPosSetpoints.emplace_back(0.0, 0.0, minPosOffset_.z());
  desiredPosSetpoints.emplace_back(0.0, 0.0, maxPosOffset_.z());
  // End offset is zero
  desiredPosSetpoints.emplace_back(0.0, 0.0, 0.0);

  for (auto setpointIt = std::next(desiredPosSetpoints.begin()); setpointIt != desiredPosSetpoints.end(); setpointIt++) {
    // Fill in position schedule
    positionSchedule_.addProfile(
        new robot_utils::ProfileRamp<loco::Position>(*std::prev(setpointIt), *setpointIt, rampDuration_.getValue()));
    positionSchedule_.addProfile(new robot_utils::ProfileStep<loco::Position>(*setpointIt, restDuration_));
  }
}

bool TorsoControlTestWholeBodyController::addParametersToHandler(const std::string& /*ns*/) {
  parameter_handler::handler->addParam(defaultTorsoHeight_);
  parameter_handler::handler->addParam(rampDuration_);
  return true;
}

loco::Position TorsoControlTestWholeBodyController::getDefaultTorsoPositionInWorldFrame(double heightOverTerrain) const {
  // Assuming
  loco::RotationQuaternion orientationWorldToTorsoHeading;
  headingGenerator_.getOrientationWorldToTorsoHeading(orientationWorldToTorsoHeading);
  loco::RotationQuaternion orientationWorldToTerrain = getOrientationWorldToHeadingOnTerrainSurface(orientationWorldToTorsoHeading);
  const loco::Position positionBaseProjectionToDesiredHeightAboveTerrainInTerrainFrame(0.0, 0.0, heightOverTerrain);
  const loco::Position positionWorldToDefaultBaseProjectionInWorldFrame =
      terrain_.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame());
  const loco::Position positionWorldToDesiredHeightAboveTerrainInWorldFrame =
      orientationWorldToTerrain.inverseRotate(positionBaseProjectionToDesiredHeightAboveTerrainInTerrainFrame) +
      positionWorldToDefaultBaseProjectionInWorldFrame;

  return positionWorldToDesiredHeightAboveTerrainInWorldFrame;
}

loco::RotationQuaternion TorsoControlTestWholeBodyController::getOrientationWorldToHeadingOnTerrainSurface(
    const loco::RotationQuaternion& orientationWorldToHeading) const {
  loco::Vector normalInWorldFrame;
  terrain_.getNormal(loco::Position::Zero(), normalInWorldFrame);

  const loco::Vector normalInHeadingControlFrame = orientationWorldToHeading.rotate(normalInWorldFrame);
  const double terrainPitch = atan2(normalInHeadingControlFrame.x(), normalInHeadingControlFrame.z());
  const double terrainRoll = atan2(normalInHeadingControlFrame.y(), normalInHeadingControlFrame.z());

  return loco::RotationQuaternion(loco::AngleAxis(terrainRoll, -1.0, 0.0, 0.0)) *
         loco::RotationQuaternion(loco::AngleAxis(terrainPitch, 0.0, 1.0, 0.0)) * orientationWorldToHeading;
}

const loco::Position& TorsoControlTestWholeBodyController::getMinPosOffset() const {
  return minPosOffset_;
}
const loco::Position& TorsoControlTestWholeBodyController::getMaxPosOffset() const {
  return maxPosOffset_;
}

void TorsoControlTestWholeBodyController::setMinPosOffset(const loco::Position& minPosOffset) {
  minPosOffset_ = minPosOffset;
}
void TorsoControlTestWholeBodyController::setMaxPosOffset(const loco::Position& maxPosOffset) {
  maxPosOffset_ = maxPosOffset;
}

bool TorsoControlTestWholeBodyController::isScheduleFinished() const {
  return isScheduleFinished_;
}

}  // namespace anymal_ctrl_test_whole_body_control
