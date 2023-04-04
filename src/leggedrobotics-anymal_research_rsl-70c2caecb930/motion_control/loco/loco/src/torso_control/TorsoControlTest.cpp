/*
 * TorsoControlTest.cpp
 *
 *  Created on: Feb, 2016
 *      Author: Christian Gehring
 */

#include "loco/torso_control/TorsoControlTest.hpp"
#include "loco/torso_control/ComSupportControlStaticGait.hpp"

namespace loco {

TorsoControlTest::TorsoControlTest(WholeBody& wholeBody, TerrainModelBase& terrain, GaitPatternStaticGait& gaitPattern,
                                   ComSupportControlStaticGait& comSupportControl)
    : TorsoControlGaitContainer(wholeBody, terrain, comSupportControl),
      time_(0.0),
      positionTrackingError_(),
      velocityTrackingError_(),
      paramTorsoHorizPosSinusoidFreqHz_("TorsoTest: hPos freq Hz", 0.5, 0.0, 10.0) {
  parameter_handler::handler->addParam(paramTorsoHorizPosSinusoidFreqHz_);
}

bool TorsoControlTest::initialize(double dt) {
  if (!Base::initialize(dt)) {
    return false;
  }

  positionTrackingError_.setZero();
  velocityTrackingError_.setZero();

  positionWorldToDesiredBaseInWorldFrame_.setZero();
  linearVelocityDesiredBaseInWorldFrame_.setZero();

  // Clear the schedule
  scheduleX_.clear();
  scheduleY_.clear();
  scheduleZ_.clear();
  scheduleRoll_.clear();
  schedulePitch_.clear();
  scheduleYaw_.clear();

  const double rampDuration = 3.0;
  const double freq = paramTorsoHorizPosSinusoidFreqHz_.getValue();
  const double frequencyX = freq;
  const double frequencyY = freq;
  const double amplitudeX = 0.05;
  const double amplitudeY = 0.05;

  const Position startPosition = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
  Position endPosition = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
  endPosition.x() = 0.0;
  endPosition.y() = 0.0;
  endPosition.z() = paramTorsoHeightAboveGround_.getValue();

  scheduleX_.addProfile(new robot_utils::ProfileRamp<double>(startPosition.x(), endPosition.x(), rampDuration));
  scheduleY_.addProfile(new robot_utils::ProfileRamp<double>(startPosition.y(), endPosition.y() + amplitudeY, rampDuration));
  scheduleZ_.addProfile(new robot_utils::ProfileRamp<double>(startPosition.z(), endPosition.z(), rampDuration));

  double sinusoidDuration = 10.0;
  scheduleX_.addProfile(new robot_utils::ProfileSinusoid<double>(amplitudeX, endPosition.x(), frequencyX, 0, sinusoidDuration));
  scheduleY_.addProfile(new robot_utils::ProfileSinusoid<double>(amplitudeY, endPosition.y(), frequencyY, M_PI / 2.0, sinusoidDuration));

  EulerAnglesZyx startEulerZyx = EulerAnglesZyx(torso_.getMeasuredState().getOrientationWorldToBase());
  EulerAnglesZyx endEulerZyx;
  scheduleRoll_.addProfile(new robot_utils::ProfileRamp<double>(startEulerZyx.x(), endEulerZyx.x(), rampDuration));
  schedulePitch_.addProfile(new robot_utils::ProfileRamp<double>(startEulerZyx.y(), endEulerZyx.y(), rampDuration));
  scheduleYaw_.addProfile(new robot_utils::ProfileRamp<double>(startEulerZyx.z(), endEulerZyx.z(), rampDuration));

  time_ = 0.0;
  timer_.pinTime();
  return true;
}

bool TorsoControlTest::advance(double dt) {
  bool success = comControl_.advance(dt);
  time_ = timer_.getElapsedTimeSec();

  const RotationQuaternion& orientationWorldToControl =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();  // --> current heading orientation
  const RotationQuaternion& orientationControlToBase = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase();
  const RotationQuaternion orientationWorldToTerrain = getOrientationWorldToHeadingOnTerrainSurface(RotationQuaternion());

  /********************************************************************************************************
   * Set desired base position in world frame
   ********************************************************************************************************/

  /* Compute the horizontal component of the desired position in world frame.
   *
   *  evaluate desired CoM position in control frame
   */
  const Position positionWorldToDesiredHorizontalBaseInWorldFrame = Position(scheduleX_.getValue(time_), scheduleY_.getValue(time_), 0.0);

  // this is the desired location of the base location relative to the origin of the control frame projected on the x-y plane of the world
  // frame and expressed in the world frame
  Position positionHorizontalControlToHorizontalBaseInWorldFrame =
      positionWorldToDesiredHorizontalBaseInWorldFrame - torso_.getMeasuredState().inControlFrame().getPositionWorldToControlInWorldFrame();
  positionHorizontalControlToHorizontalBaseInWorldFrame.z() = 0.0;

  // const Position positionWorldToDesiredHeightAboveTerrainInTerrainFrame(0.0, 0.0, paramTorsoHeightAboveGround_.getValue());
  const Position positionWorldToDesiredHeightAboveTerrainInTerrainFrame(0.0, 0.0, scheduleZ_.getValue(time_));
  const Position positionWorldToDesiredHeightAboveTerrainInWorldFrame =
      orientationWorldToTerrain.inverseRotate(positionWorldToDesiredHeightAboveTerrainInTerrainFrame);

  loco::Vector surfaceNormalInWorldFrame;
  terrain_.getNormal(loco::Position::Zero(), surfaceNormalInWorldFrame);
  double heightOverTerrain = positionWorldToDesiredHeightAboveTerrainInWorldFrame.dot(surfaceNormalInWorldFrame);
  heightOverTerrain /= surfaceNormalInWorldFrame.z();

  double heightOfTerrainInWorldFrame = 0.0;
  terrain_.getHeight(positionWorldToDesiredHorizontalBaseInWorldFrame, heightOfTerrainInWorldFrame);

  //  Position positionWorldToHorizontalBaseInWorldFrame_temp = positionWorldToDesiredHorizontalBaseInWorldFrame +
  //  heightOfTerrainInWorldFrame*Position::UnitZ();

  const Position positionControlToTargetBaseInWorldFrame =
      positionHorizontalControlToHorizontalBaseInWorldFrame + (heightOfTerrainInWorldFrame + heightOverTerrain) * Position::UnitZ();

  // store desired base position for logging
  positionControlToTargetBaseInWorldFrame_ = loco::Vector(positionControlToTargetBaseInWorldFrame);

  const Position positionControlToTargetBaseInControlFrame = orientationWorldToControl.rotate(
      positionControlToTargetBaseInWorldFrame + torso_.getDesiredState().getDesiredPositionOffsetInWorldFrame());

  /********************************************************************************************************
   * End set desired CoM position in world frame *
   ********************************************************************************************************/

  /********************************************************************************************************
   * Set the desired orientation of the base frame with respect to the control frame
   ********************************************************************************************************/
  // this is the orientation we need to compute

  // desiredOrientationOffset_ = RotationQuaternion(EulerAnglesZyx(scheduleYaw_.getValue(time_), schedulePitch_.getValue(time_),
  // scheduleRoll_.getValue(time_)));
  const RotationQuaternion orientationControlToDesiredBase = torso_.getDesiredState().getDesiredOrientationOffset();
  //---

  // set control references
  torso_.getDesiredStatePtr()->setPositionControlToTargetInControlFrame(positionControlToTargetBaseInControlFrame);

  torso_.getDesiredStatePtr()->setPositionWorldToBaseInWorldFrame(
      orientationWorldToControl.inverseRotate(positionControlToTargetBaseInControlFrame));

  torso_.getDesiredStatePtr()->setOrientationControlToBase(orientationControlToDesiredBase);

  const LinearVelocity linearVelocityDesiredBaseInWorldFrame(
      scheduleX_.getValueFirstDerivative(time_), scheduleY_.getValueFirstDerivative(time_), scheduleZ_.getValueFirstDerivative(time_));
  const LinearVelocity linearVelocityBaseInControlFrame = orientationWorldToControl.rotate(linearVelocityDesiredBaseInWorldFrame);
  torso_.getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(linearVelocityBaseInControlFrame);
  const LocalAngularVelocity angularVelocityBaseInControlFrame;
  torso_.getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(angularVelocityBaseInControlFrame);

  const LinearAcceleration linearAccelerationBaseInWorldFrame(
      scheduleX_.getValueSecondDerivative(time_), scheduleY_.getValueSecondDerivative(time_), scheduleZ_.getValueSecondDerivative(time_));
  torso_.getDesiredStatePtr()->setLinearAccelerationTargetInControlFrame(
      orientationWorldToControl.rotate(linearAccelerationBaseInWorldFrame));

  // set computed control errors

  Position positionErrorInControlFrame;
  getMixedComBasePositionError(wholeBody_, positionErrorInControlFrame,
                               torso_.getDesiredState().getPositionControlToTargetInControlFrame());
  //  getBasePositionError(wholeBody_, positionErrorInControlFrame, torso_.getDesiredState().getPositionControlToBaseInControlFrame());
  torso_.getDesiredStatePtr()->setPositionErrorInControlFrame(positionErrorInControlFrame);

  LinearVelocity linearVelocityErrorInControlFrame;
  getMixedComBaseLinearVelocityError(wholeBody_, linearVelocityErrorInControlFrame,
                                     torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
  //  getBaseLinearVelocityError(linearVelocityErrorInControlFrame, torso_.getDesiredState().getLinearVelocityBaseInControlFrame());
  torso_.getDesiredStatePtr()->setLinearVelocityErrorInControlFrame(linearVelocityErrorInControlFrame);

  // store desired velocity for logging
  linearVelocityTargetBaseInBaseFrame_ = orientationControlToBase.rotate(loco::Vector(linearVelocityBaseInControlFrame));

  // update for visualization
  positionWorldToDesiredBaseInWorldFrame_ = loco::Vector(positionControlToTargetBaseInWorldFrame);
  linearVelocityDesiredBaseInWorldFrame_ =
      loco::Vector(orientationControlToBase.rotate(torso_.getDesiredState().getLinearVelocityTargetInControlFrame()));

  positionTrackingError_ = loco::Vector(positionControlToTargetBaseInControlFrame -
                                        torso_.getMeasuredState().inControlFrame().getPositionControlToBaseInControlFrame());
  velocityTrackingError_ =
      loco::Vector(torso_.getDesiredState().getLinearVelocityTargetInControlFrame() -
                   orientationControlToBase.inverseRotate(torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame()));

  // set desired orientation for logging
  loco::EulerAnglesZyx desiredOrientationEulerAnglesZyxBaseToWorld(
      (torso_.getDesiredState().getOrientationControlToBase() * torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl())
          .inverted());
  desiredOrientationEulerAnglesZyxBaseToWorld.setUnique();
  torso_.getDesiredStatePtr()->setOrientationEulerAnglesZyxBaseToWorld(desiredOrientationEulerAnglesZyxBaseToWorld);

  time_ += dt;
  return success;
}

void TorsoControlTest::setMainBodyDesiredHeightFromTerrain(double height) {
  desiredTorsoCoMHeightAboveGroundInControlFrameOffset_ = height;
  robot_utils::boundToRange(&desiredTorsoCoMHeightAboveGroundInControlFrameOffset_, 0.42, 0.51);
}

bool TorsoControlTest::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
  return false;
}

void TorsoControlTest::setIsInStandConfiguration(bool isInStandConfiguration) {
  dynamic_cast<ComSupportControlStaticGait&>(comControl_).setIsInStandConfiguration(isInStandConfiguration);
}

bool TorsoControlTest::addVariablesToLog(bool update) {
  signal_logger::add(positionTrackingError_, "positionTrackingError", "/loco/torso_controller/", "m");

  signal_logger::add(velocityTrackingError_, "velocityTrackingError", "/loco/torso_controller/", "m/s");

  signal_logger::add(positionWorldToDesiredBaseInWorldFrame_, "positionWorldToDesiredBaseInWorldFrame", "/loco/torso_controller/", "m");

  signal_logger::add(linearVelocityDesiredBaseInWorldFrame_, "linearVelocityDesiredBaseInWorldFrame", "/loco/torso_controller/", "m/s");

  return true;
}

bool TorsoControlTest::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle handleTorsoConfiguration(handle.FirstChild("TorsoControl").FirstChild("TorsoConfiguration"));
  TiXmlHandle handleDynamicGait(handle.FirstChild("TorsoControl").FirstChild("DynamicGait"));

  if (!loadParameters(handleTorsoConfiguration)) {
    return false;
  }

  if (!dynamic_cast<ComSupportControlStaticGait&>(comControl_).loadParameters(handle)) {
    return false;
  }

  return true;
}

} /* namespace loco */
