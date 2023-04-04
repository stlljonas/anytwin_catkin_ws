#include <cmath>
#include <sstream>

#include "anydrive/Command.hpp"

namespace anydrive {

const any_measurements::Time& Command::getStamp() const {
  return stamp_;
}

void Command::setStamp(const any_measurements::Time& stamp) {
  stamp_ = stamp;
}

mode::ModeEnum Command::getModeEnum() const {
  return modeEnum_;
}

void Command::setModeEnum(const mode::ModeEnum modeEnum) {
  modeEnum_ = modeEnum;
}

double Command::getCurrent() const {
  return current_;
}

void Command::setCurrent(const double current) {
  current_ = current;
}

double Command::getMotorPosition() const {
  return motorPosition_;
}

void Command::setMotorPosition(const double motorPosition) {
  motorPosition_ = motorPosition;
}

double Command::getMotorVelocity() const {
  return motorVelocity_;
}

void Command::setMotorVelocity(const double motorVelocity) {
  motorVelocity_ = motorVelocity;
}

double Command::getGearPosition() const {
  return gearPosition_;
}

void Command::setGearPosition(const double gearPosition) {
  gearPosition_ = gearPosition;
}

double Command::getGearVelocity() const {
  return gearVelocity_;
}

void Command::setGearVelocity(const double gearVelocity) {
  gearVelocity_ = gearVelocity;
}

double Command::getJointPosition() const {
  return jointPosition_;
}

void Command::setJointPosition(const double jointPosition) {
  jointPosition_ = jointPosition;
}

double Command::getJointVelocity() const {
  return jointVelocity_;
}

void Command::setJointVelocity(const double jointVelocity) {
  jointVelocity_ = jointVelocity;
}

double Command::getJointTorque() const {
  return jointTorque_;
}

void Command::setJointTorque(const double jointTorque) {
  jointTorque_ = jointTorque;
}

mode::PidGainsF& Command::getPidGains() {
  return pidGains_;
}

const mode::PidGainsF& Command::getPidGains() const {
  return pidGains_;
}

void Command::setPidGains(const mode::PidGainsF& pidGains) {
  pidGains_ = pidGains;
}

bool Command::isValid() const {
  return (!stamp_.isZero() && modeEnum_ != mode::ModeEnum::NA && std::isfinite(current_) && std::isfinite(motorPosition_) &&
          std::isfinite(motorVelocity_) && std::isfinite(gearPosition_) && std::isfinite(gearVelocity_) && std::isfinite(jointPosition_) &&
          std::isfinite(jointVelocity_) && std::isfinite(jointTorque_) && pidGains_.isValid());
}

std::string Command::asString(const std::string& prefix) const {
  std::stringstream ss;
  ss << prefix << "Stamp: " << stamp_ << std::endl;
  ss << prefix << "Mode: " << modeEnum_ << std::endl;
  ss << prefix << "Current: " << current_ << std::endl;
  ss << prefix << "Motor position: " << motorPosition_ << std::endl;
  ss << prefix << "Motor velocity: " << motorVelocity_ << std::endl;
  ss << prefix << "Gear position: " << gearPosition_ << std::endl;
  ss << prefix << "Gear velocity: " << gearVelocity_ << std::endl;
  ss << prefix << "Joint position: " << jointPosition_ << std::endl;
  ss << prefix << "Joint velocity: " << jointVelocity_ << std::endl;
  ss << prefix << "Joint torque: " << jointTorque_ << std::endl;
  ss << prefix << "PID gains: " << pidGains_;
  return ss.str();
}

std::ostream& operator<<(std::ostream& out, const Command& command) {
  out << "Command:" << std::endl;
  out << command.asString("  ") << std::endl;
  return out;
}

}  // namespace anydrive
