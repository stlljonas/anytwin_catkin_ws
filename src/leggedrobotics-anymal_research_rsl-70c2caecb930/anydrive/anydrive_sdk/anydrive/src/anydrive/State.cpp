#include <cmath>
#include <sstream>

#include "anydrive/State.hpp"

namespace anydrive {

const any_measurements::Time& State::getStamp() const {
  return stamp_;
}

void State::setStamp(const any_measurements::Time& stamp) {
  stamp_ = stamp;
}

const Statusword& State::getStatusword() const {
  return statusword_;
}

void State::setStatusword(const Statusword& statusword) {
  statusword_ = statusword;
}

double State::getCurrent() const {
  return current_;
}

void State::setCurrent(const double current) {
  current_ = current;
}

double State::getGearPosition() const {
  return gearPosition_;
}

void State::setGearPosition(const double gearPosition) {
  gearPosition_ = gearPosition;
}

double State::getGearVelocity() const {
  return gearVelocity_;
}

void State::setGearVelocity(const double gearVelocity) {
  gearVelocity_ = gearVelocity;
}

double State::getJointPosition() const {
  return jointPosition_;
}

void State::setJointPosition(const double jointPosition) {
  jointPosition_ = jointPosition;
}

double State::getJointVelocity() const {
  return jointVelocity_;
}

void State::setJointVelocity(const double jointVelocity) {
  jointVelocity_ = jointVelocity;
}

double State::getJointAcceleration() const {
  return jointAcceleration_;
}

void State::setJointAcceleration(const double jointAcceleration) {
  jointAcceleration_ = jointAcceleration;
}

double State::getJointTorque() const {
  return jointTorque_;
}

void State::setJointTorque(const double jointTorque) {
  jointTorque_ = jointTorque;
}

const any_measurements::Imu& State::getImu() const {
  return imu_;
}

void State::setImu(const any_measurements::Imu& imu) {
  imu_ = imu;
}

bool State::isValid() const {
  return (!stamp_.isZero() && !statusword_.isEmpty() && std::isfinite(current_) && std::isfinite(gearPosition_) &&
          std::isfinite(gearVelocity_) && std::isfinite(jointPosition_) && std::isfinite(jointVelocity_) && std::isfinite(jointTorque_) &&
          std::isfinite(!imu_.time_.isZero()) && std::isfinite(imu_.linearAcceleration_.x()) &&
          std::isfinite(imu_.linearAcceleration_.y()) && std::isfinite(imu_.linearAcceleration_.z()) &&
          std::isfinite(imu_.angularVelocity_.x()) && std::isfinite(imu_.angularVelocity_.y()) && std::isfinite(imu_.angularVelocity_.z()));
}

std::string State::asString(const std::string& prefix) const {
  std::stringstream ss;
  ss << prefix << "Stamp: " << stamp_ << std::endl;
  ss << prefix << "Statusword: " << statusword_ << std::endl;
  ss << prefix << "Current: " << current_ << std::endl;
  ss << prefix << "Gear position: " << gearPosition_ << std::endl;
  ss << prefix << "Gear velocity: " << gearVelocity_ << std::endl;
  ss << prefix << "Joint position: " << jointPosition_ << std::endl;
  ss << prefix << "Joint velocity: " << jointVelocity_ << std::endl;
  ss << prefix << "Joint acceleration: " << jointAcceleration_ << std::endl;
  ss << prefix << "Joint torque: " << jointTorque_ << std::endl;
  ss << prefix << "IMU:" << std::endl;
  ss << prefix << prefix << "Stamp: " << imu_.time_ << std::endl;
  ss << prefix << prefix << "Linear acceleration: " << imu_.linearAcceleration_ << std::endl;
  ss << prefix << prefix << "Angular velocity: " << imu_.angularVelocity_;
  return ss.str();
}

std::ostream& operator<<(std::ostream& out, const State& state) {
  out << "State:" << std::endl;
  out << state.asString("  ");
  return out;
}

}  // namespace anydrive
