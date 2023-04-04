/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Implementation of mission controller for WBC test controller
 * @date    Jul 12, 2019
 */

// anymal_ctrl_test_whole_body_control
#include "anymal_ctrl_test_whole_body_control/mission_control/MissionControlTestWholeBodyController.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace anymal_ctrl_test_whole_body_control {

MissionControlTestWholeBodyController::MissionControlTestWholeBodyController(
    loco::Pose* desRobotPose, loco::TorsoBase& torso,
    anymal_ctrl_test_whole_body_control::SwingTrajectoryTestWholeBodyController& swingTrajectory,
    anymal_ctrl_test_whole_body_control::TorsoControlTestWholeBodyController& torsoControl,
    loco_anymal::WholeBodyController& wholeBodyController)
    : desRobotPose_(desRobotPose),
      torso_(torso),
      swingTrajectory_(swingTrajectory),
      torsoControl_(torsoControl),
      wholeBodyController_(wholeBodyController),
      speedFilter_(),
      locomotionState_(LocomotionState::Freeze),
      locomotionMode_(LocomotionMode::ModeUndefined) {}

bool MissionControlTestWholeBodyController::initialize(double /*dt*/) {
  locomotionState_ = LocomotionState::Freeze;
  locomotionMode_ = LocomotionMode::ModeUndefined;
  return true;
}

bool MissionControlTestWholeBodyController::advance(double dt) {
  {
    std::lock_guard<std::mutex> advanceLock(advanceMutex_);
    switch (locomotionMode_) {
      case LocomotionMode::ModeJointMotion:
        if (locomotionState_ == LocomotionState::Freeze) {
          return switchToSwing();
        }
        break;
      case LocomotionMode::ModeEEMotion:
        if (locomotionState_ == LocomotionState::Freeze) {
          return switchToSwing();
        }
        break;
      case LocomotionMode::ModeBaseMotion:
        if (locomotionState_ == LocomotionState::Freeze) {
          return switchToStand();
        }
        // Set inputs to speed filter and run it
        speedFilter_.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(desRobotPose_->getPosition());
        speedFilter_.setUnfilteredDesiredOrientationOffset(loco::RotationQuaternion(desRobotPose_->getRotation().getUnique()));
        speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(loco::Twist());
        speedFilter_.advance(dt);

        // Add desired and limit offsets and compute trajectory
        torso_.getDesiredStatePtr()->setDesiredPositionOffsetInWorldFrame(speedFilter_.getFilteredPositionOffsetInWorldFrame());
        torso_.getDesiredStatePtr()->setDesiredOrientationOffset(speedFilter_.getFilteredDesiredOrientationOffset());

        break;
      default:
        MELO_DEBUG_THROTTLE(1.0, "Undefined locomotion mode!");
        if (locomotionState_ != LocomotionState::Freeze) {
          switchToFreeze();
        }
        break;
    }
  }
  advanceCv_.notify_all();
  return true;
}

bool MissionControlTestWholeBodyController::loadParameters(const TiXmlHandle& handle) {
  return speedFilter_.loadParameters(handle);
}

const loco::Twist& MissionControlTestWholeBodyController::getDesiredBaseTwistInControlFrame() const {
  return speedFilter_.getDesiredBaseTwistInControlFrame();
}

const loco::Twist& MissionControlTestWholeBodyController::getMaximumBaseTwistInControlFrame() const {
  return speedFilter_.getMaximumBaseTwistInControlFrame();
}

const loco::Pose& MissionControlTestWholeBodyController::getMinimalPoseOffset() const {
  return speedFilter_.getMinimalPoseOffset();
}

const loco::Pose& MissionControlTestWholeBodyController::getMaximalPoseOffset() const {
  return speedFilter_.getMaximalPoseOffset();
}

bool MissionControlTestWholeBodyController::switchToStand() {
  torsoControl_.setMinPosOffset(speedFilter_.getMinimalPoseOffset().getPosition());
  torsoControl_.setMaxPosOffset(speedFilter_.getMaximalPoseOffset().getPosition());
  torsoControl_.computeTrajectory();
  locomotionState_ = LocomotionState::Standing;
  return true;
}

bool MissionControlTestWholeBodyController::switchToSwing() {
  swingTrajectory_.computeTrajectory();
  locomotionState_ = LocomotionState::Swinging;
  return true;
}

bool MissionControlTestWholeBodyController::switchToFreeze() {
  swingTrajectory_.clearTrajectory();
  torsoControl_.clearTrajectory();
  locomotionState_ = LocomotionState::Freeze;
  return true;
}

bool MissionControlTestWholeBodyController::executeJointMotion() {
  if (locomotionState_ != LocomotionState::Freeze) {
    MELO_WARN("Joint motion mode can only be started from freeze");
    return false;
  }
  if (!wholeBodyController_.switchTaskSetup("joint_motion")) {
    MELO_ERROR("WBC Task setup could not be change to joint_motion");
    return false;
  };
  {
    std::lock_guard<std::mutex> setLocoModeLock(advanceMutex_);
    locomotionMode_ = LocomotionMode::ModeJointMotion;
  }
  waitForSwingScheduleToFinish();
  return true;
}

bool MissionControlTestWholeBodyController::executeEEMotion() {
  if (locomotionState_ != LocomotionState::Freeze) {
    MELO_WARN("EE motion mode can only be started from freeze");
    return false;
  }
  if (!wholeBodyController_.switchTaskSetup("ee_motion")) {
    MELO_ERROR("WBC Task setup could not be change to ee_motion");
    return false;
  };
  {
    std::lock_guard<std::mutex> setLocoModeLock(advanceMutex_);
    locomotionMode_ = LocomotionMode::ModeEEMotion;
  }
  waitForSwingScheduleToFinish();
  return true;
}

bool MissionControlTestWholeBodyController::executeBaseMotion() {
  if (locomotionState_ != LocomotionState::Freeze) {
    MELO_WARN("Base motion mode can only be started from freeze");
    return false;
  }
  if (!wholeBodyController_.switchTaskSetup("base_motion")) {
    MELO_ERROR("WBC Task setup could not be change to base_motion");
    return false;
  };
  {
    std::lock_guard<std::mutex> setLocoModeLock(advanceMutex_);
    locomotionMode_ = LocomotionMode::ModeBaseMotion;
  }
  waitForTorsoScheduleToFinish();
  return true;
}

const std::string& MissionControlTestWholeBodyController::getCurrentLocomotionState() const {
  return stateNameMap_.at(locomotionState_);
}

const std::string& MissionControlTestWholeBodyController::getCurrentLocomotionMode() const {
  return modeNameMap_.at(locomotionMode_);
}

MissionControlTestWholeBodyController::LocomotionMode MissionControlTestWholeBodyController::getLocomotionModeFromName(
    const std::string& name) const {
  if (name == "joint_motion") {
    return LocomotionMode::ModeJointMotion;
  } else if (name == "ee_motion") {
    return LocomotionMode::ModeEEMotion;
  } else if (name == "base_motion") {
    return LocomotionMode::ModeBaseMotion;
  } else {
    return LocomotionMode::ModeUndefined;
  }
}

void MissionControlTestWholeBodyController::waitForTorsoScheduleToFinish() {
  std::unique_lock<std::mutex> advanceLock(advanceMutex_);
  while (!advanceCv_.wait_for(advanceLock, std::chrono::milliseconds(100), [this] {
    return this->torsoControl_.isScheduleFinished() || locomotionMode_ == LocomotionMode::ModeUndefined;
  })) {
  }
  locomotionMode_ = LocomotionMode::ModeUndefined;
}

void MissionControlTestWholeBodyController::waitForSwingScheduleToFinish() {
  std::unique_lock<std::mutex> advanceLock(advanceMutex_);
  while (!advanceCv_.wait_for(advanceLock, std::chrono::milliseconds(100), [this] {
    return this->swingTrajectory_.isScheduleFinished() || locomotionMode_ == LocomotionMode::ModeUndefined;
  })) {
  }
  locomotionMode_ = LocomotionMode::ModeUndefined;
}

bool MissionControlTestWholeBodyController::stop() {
  locomotionMode_ = LocomotionMode::ModeUndefined;
  switchToFreeze();
  advanceCv_.notify_all();
  return true;
}

}  // namespace anymal_ctrl_test_whole_body_control
