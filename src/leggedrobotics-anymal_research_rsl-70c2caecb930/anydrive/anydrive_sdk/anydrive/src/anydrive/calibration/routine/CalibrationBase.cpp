#include "anydrive/calibration/routine/CalibrationBase.hpp"

namespace anydrive {
namespace calibration {
namespace routine {

CalibrationBase::CalibrationBase(const AnydrivePtr& anydrive)  // NOLINT
    : isRunning_(false), shutdownRequested_(false), anydrive_(anydrive), filesFolder_(getHomeDirectory()) {}

bool CalibrationBase::isRunning() {
  return isRunning_;
}

void CalibrationBase::requestShutdown() {
  shutdownRequested_ = true;
}

bool CalibrationBase::run() {
  if (stopRequested()) {
    return false;
  }

  isRunning_ = true;
  startupDataCollection();
  const bool collectDataSuccess = collectData();
  shutdownDataCollection();
  if (!collectDataSuccess || stopRequested()) {
    isRunning_ = false;
    return false;
  }
  if (!postProcessData()) {
    isRunning_ = false;
    return false;
  }
  isRunning_ = false;
  return true;
}

void CalibrationBase::shutdown() {
  while (isRunning_) {
    threadSleep(timeStep_);
  }
}

std::string CalibrationBase::getHomeDirectory() {
  const char* home = getenv("HOME");
  if (home == nullptr) {
    ANYDRIVE_ERROR("Home directory environment variable could not be evaluated.");
    return std::string();
  }
  return std::string(home);
}

void CalibrationBase::startupDataCollection() {}

void CalibrationBase::shutdownDataCollection() {
  anydrive_->stageFreeze();
  anydrive_->clearLogs();
}

bool CalibrationBase::stopRequested() {
  return shutdownRequested_;
}

bool CalibrationBase::preemptableSleep(const double duration) {
  ANYDRIVE_ASSERT(duration >= 0.0, "The duration is smaller than 0.");
  double remainingDuration = duration;
  while (remainingDuration > timeStep_) {
    if (stopRequested()) {
      return false;
    }
    threadSleep(timeStep_);
    remainingDuration -= timeStep_;
  }
  threadSleep(remainingDuration);
  return true;
}

void CalibrationBase::stageCommandDisable() {
  Command command;
  command.setModeEnum(mode::ModeEnum::Disable);
  command.setStamp(any_measurements::Time::NowWallClock());
  anydrive_->stageCommand(command);
}

void CalibrationBase::stageCommandMotorVelocity(const double motorVelocity) {
  Command command;
  command.setModeEnum(mode::ModeEnum::MotorVelocity);
  command.setStamp(any_measurements::Time::NowWallClock());
  command.setMotorVelocity(motorVelocity);
  anydrive_->stageCommand(command);
}

void CalibrationBase::stageCommandJointVelocity(const double jointVelocity) {
  Command command;
  command.setModeEnum(mode::ModeEnum::JointVelocity);
  command.setStamp(any_measurements::Time::NowWallClock());
  command.setJointVelocity(jointVelocity);
  anydrive_->stageCommand(command);
}

bool CalibrationBase::getGearRatio(uint32_t& gearRatio) {
  if (!anydrive_->getGearboxRatio(gearRatio)) {
    // Provide a fallback, such that old ANYdrives can still be calibrated with a new SDK:
    ANYDRIVE_INFO("Could not read gear ratio from device (old firmware?). Falling back to default value of " << gearRatioDefault_ << ":1.");
    gearRatio = gearRatioDefault_;
  } else {
    ANYDRIVE_INFO("Obtained gearbox ratio " << gearRatio << ":1 from drive.");
  }
  // Check if the obtained gearbox value is allowed:
  if (!(std::find(gearRatios_.begin(), gearRatios_.end(), gearRatio) != gearRatios_.end())) {
    ANYDRIVE_ERROR("Faulty gear ratio detected: " << gearRatio << ":1.");
    return false;
  }
  return true;
}

}  // namespace routine
}  // namespace calibration
}  // namespace anydrive
