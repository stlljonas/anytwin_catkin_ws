/*!
 * @file    SeActuatorCommand.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */

// c++
#include <cmath>
#include <iomanip>
#include <unordered_map>

// series elastic actuator
#include "series_elastic_actuator/common.hpp"
#include "series_elastic_actuator/SeActuatorCommand.hpp"


namespace series_elastic_actuator {


static std::unordered_map<SeActuatorCommand::SeActuatorModeType, std::string> modeNames = {
    { SeActuatorCommand::SeActuatorMode::MODE_NA, "N/A" },
    { SeActuatorCommand::SeActuatorMode::MODE_FREEZE, "Freeze motor" },
    { SeActuatorCommand::SeActuatorMode::MODE_DISABLE, "Disable motor" },
    { SeActuatorCommand::SeActuatorMode::MODE_CURRENT, "Track current" },
    { SeActuatorCommand::SeActuatorMode::MODE_MOTOR_POSITION, "Track motor position" },
    { SeActuatorCommand::SeActuatorMode::MODE_MOTOR_VELOCITY, "Track motor velocity" },
    { SeActuatorCommand::SeActuatorMode::MODE_GEAR_POSITION, "Track gear position" },
    { SeActuatorCommand::SeActuatorMode::MODE_GEAR_VELOCITY, "Track gear velocity" },
    { SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION, "Track joint position" },
    { SeActuatorCommand::SeActuatorMode::MODE_JOINT_VELOCITY, "Track joint velocity" },
    { SeActuatorCommand::SeActuatorMode::MODE_JOINT_TORQUE, "Track joint torque" },
    { SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION_VELOCITY, "Track joint position with feedforward velocity" },
    { SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE, "Track joint position with feedforward velocity and torque" },
    { SeActuatorCommand::SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS, "Track joint position with feedforward velocity and torque using custom joint position gains" }
};


SeActuatorCommand::SeActuatorCommand()
: SeActuatorCommand(SeActuatorMode::MODE_FREEZE) {}

SeActuatorCommand::SeActuatorCommand(SeActuatorModeType mode)
: mode_(mode) {}

SeActuatorCommand::~SeActuatorCommand() {}

const any_measurements::Time& SeActuatorCommand::getStamp() const {
  return stamp_;
}

void SeActuatorCommand::setStamp(const any_measurements::Time& stamp) {
  stamp_ = stamp;
}

std::string SeActuatorCommand::getModeName(SeActuatorModeType mode) {
  try {
    return modeNames.at(mode);
  }
  catch (...) {
    return std::string("Unknown mode: ") + std::to_string(mode);
  }
}

std::string SeActuatorCommand::getModeName() const {
  return getModeName(static_cast<SeActuatorModeType>(mode_));
}

const SeActuatorCommand::SeActuatorModeType& SeActuatorCommand::getMode() const {
  return mode_;
}

void SeActuatorCommand::setMode(const SeActuatorCommand::SeActuatorModeType& mode) {
  mode_ = mode;
}

SeActuatorCommand::SeActuatorMode SeActuatorCommand::getModeEnum() const {
  return static_cast<SeActuatorMode>(mode_);
}

void SeActuatorCommand::setModeEnum(const SeActuatorMode& modeEnum) {
  mode_ = static_cast<SeActuatorModeType>(modeEnum);
}

const double& SeActuatorCommand::getCurrent() const {
  return current_;
}

void SeActuatorCommand::setCurrent(double current) {
  current_ = current;
}

const double& SeActuatorCommand::getMotorPosition() const {
  return motorPosition_;
}

void SeActuatorCommand::setMotorPosition(double motorPosition) {
  motorPosition_ = motorPosition;
}

const double& SeActuatorCommand::getMotorVelocity() const {
  return motorVelocity_;
}

void SeActuatorCommand::setMotorVelocity(double motorVelocity) {
  motorVelocity_ = motorVelocity;
}

const double& SeActuatorCommand::getGearPosition() const {
  return gearPosition_;
}

void SeActuatorCommand::setGearPosition(double gearPosition) {
  gearPosition_ = gearPosition;
}

const double& SeActuatorCommand::getGearVelocity() const {
  return gearVelocity_;
}

void SeActuatorCommand::setGearVelocity(double gearVelocity) {
  gearVelocity_ = gearVelocity;
}

const double& SeActuatorCommand::getJointPosition() const {
  return jointPosition_;
}

void SeActuatorCommand::setJointPosition(double jointPosition) {
  jointPosition_ = jointPosition;
}

const double& SeActuatorCommand::getJointVelocity() const {
  return jointVelocity_;
}

void SeActuatorCommand::setJointVelocity(double jointVelocity) {
  jointVelocity_ = jointVelocity;
}

const double& SeActuatorCommand::getJointTorque() const {
  return jointTorque_;
}

void SeActuatorCommand::setJointTorque(double jointTorque) {
  jointTorque_ = jointTorque;
}

const double& SeActuatorCommand::getPidGainsP() const {
  return pidGainsP_;
}

void SeActuatorCommand::setPidGainsP(double pidGainsP) {
  pidGainsP_ = pidGainsP;
}

const double& SeActuatorCommand::getPidGainsI() const {
  return pidGainsI_;
}

void SeActuatorCommand::setPidGainsI(double pidGainsI) {
  pidGainsI_ = pidGainsI;
}

const double& SeActuatorCommand::getPidGainsD() const {
  return pidGainsD_;
}

void SeActuatorCommand::setPidGainsD(double pidGainsD) {
  pidGainsD_ = pidGainsD;
}

void SeActuatorCommand::setPidGains(PidGains& gains) {
  pidGainsP_ = gains.pGain_;
  pidGainsI_ = gains.iGain_;
  pidGainsD_ = gains.dGain_;
}

const double& SeActuatorCommand::getCurrentMax() const {
  return currentMax_;
}

void SeActuatorCommand::setCurrentMax(double currentMax) {
  currentMax_ = currentMax;
}

const double& SeActuatorCommand::getCurrentMin() const {
  return currentMin_;
}

void SeActuatorCommand::setCurrentMin(double currentMin) {
  currentMin_ = currentMin;
}

const double& SeActuatorCommand::getMotorPositionMin() const {
  return motorPositionMin_;
}

void SeActuatorCommand::setMotorPositionMin(double motorPositionMin) {
  motorPositionMin_ = motorPositionMin;
}

const double& SeActuatorCommand::getMotorPositionMax() const {
  return motorPositionMax_;
}

void SeActuatorCommand::setMotorPositionMax(double motorPositionMax) {
  motorPositionMax_ = motorPositionMax;
}

const double& SeActuatorCommand::getMotorVelocityMin() const {
  return motorVelocityMin_;
}

void SeActuatorCommand::setMotorVelocityMin(double motorVelocityMin) {
  motorVelocityMin_ = motorVelocityMin;
}

const double& SeActuatorCommand::getMotorVelocityMax() const {
  return motorVelocityMax_;
}

void SeActuatorCommand::setMotorVelocityMax(double motorVelocityMax) {
  motorVelocityMax_ = motorVelocityMax;
}

const double& SeActuatorCommand::getGearPositionMin() const {
  return gearPositionMin_;
}

void SeActuatorCommand::setGearPositionMin(double gearPositionMin) {
  gearPositionMin_ = gearPositionMin;
}

const double& SeActuatorCommand::getGearPositionMax() const {
  return gearPositionMax_;
}

void SeActuatorCommand::setGearPositionMax(double gearPositionMax) {
  gearPositionMax_ = gearPositionMax;
}

const double& SeActuatorCommand::getGearVelocityMin() const {
  return gearVelocityMin_;
}

void SeActuatorCommand::setGearVelocityMin(double gearVelocityMin) {
  gearVelocityMin_ = gearVelocityMin;
}

const double& SeActuatorCommand::getGearVelocityMax() const {
  return gearVelocityMax_;
}

void SeActuatorCommand::setGearVelocityMax(double gearVelocityMax) {
  gearVelocityMax_ = gearVelocityMax;
}

const double& SeActuatorCommand::getJointPositionMin() const {
  return jointPositionMin_;
}

void SeActuatorCommand::setJointPositionMin(double jointPositionMin) {
  jointPositionMin_ = jointPositionMin;
}

const double& SeActuatorCommand::getJointPositionMax() const {
  return jointPositionMax_;
}

void SeActuatorCommand::setJointPositionMax(double jointPositionMax) {
  jointPositionMax_ = jointPositionMax;
}

const double& SeActuatorCommand::getJointVelocityMin() const {
  return jointVelocityMin_;
}

void SeActuatorCommand::setJointVelocityMin(double jointVelocityMin) {
  jointVelocityMin_ = jointVelocityMin;
}

const double& SeActuatorCommand::getJointVelocityMax() const {
  return jointVelocityMax_;
}

void SeActuatorCommand::setJointVelocityMax(double jointVelocityMax) {
  jointVelocityMax_ = jointVelocityMax;
}

const double& SeActuatorCommand::getJointTorqueMin() const {
  return jointTorqueMin_;
}

void SeActuatorCommand::setJointTorqueMin(double jointTorqueMin) {
  jointTorqueMin_ = jointTorqueMin;
}

const double& SeActuatorCommand::getJointTorqueMax() const {
  return jointTorqueMax_;
}

void SeActuatorCommand::setJointTorqueMax(double jointTorqueMax) {
  jointTorqueMax_ = jointTorqueMax;
}

void SeActuatorCommand::limit() {
  current_ = saturate(current_, currentMin_, currentMax_);
  jointPosition_ = saturate(jointPosition_, jointPositionMin_, jointPositionMax_);
  jointVelocity_ = saturate(jointVelocity_, jointVelocityMin_, jointVelocityMax_);
  jointTorque_ = saturate(jointTorque_, jointTorqueMin_, jointTorqueMax_);
}

bool SeActuatorCommand::isFinite() const {
  return (
      std::isfinite(current_) &&
      std::isfinite(motorPosition_) &&
      std::isfinite(motorVelocity_) &&
      std::isfinite(gearPosition_) &&
      std::isfinite(gearVelocity_) &&
      std::isfinite(jointPosition_) &&
      std::isfinite(jointVelocity_) &&
      std::isfinite(jointTorque_) &&
      std::isfinite(pidGainsP_) &&
      std::isfinite(pidGainsI_) &&
      std::isfinite(pidGainsD_));
}

bool SeActuatorCommand::isWithinLimits() const {
  return (
      current_ >= currentMin_ &&
      current_ <= currentMax_ &&
      motorPosition_ >= motorPositionMin_ &&
      motorPosition_ <= motorPositionMax_ &&
      motorVelocity_ >= motorVelocityMin_ &&
      motorVelocity_ <= motorVelocityMax_ &&
      gearPosition_ >= gearPositionMin_ &&
      gearPosition_ <= gearPositionMax_ &&
      gearVelocity_ >= gearVelocityMin_ &&
      gearVelocity_ <= gearVelocityMax_ &&
      jointPosition_ >= jointPositionMin_ &&
      jointPosition_ <= jointPositionMax_ &&
      jointVelocity_ >= jointVelocityMin_ &&
      jointVelocity_ <= jointVelocityMax_ &&
      jointTorque_ >= jointTorqueMin_ &&
      jointTorque_ <= jointTorqueMax_);
}

bool SeActuatorCommand::isValid() const {
  return isFinite() && isWithinLimits();
}

std::ostream& operator<<(std::ostream& out, const SeActuatorCommand& command) {
  out << "mode: "  << command.getModeName() << std::endl;
  out << "current: "  << command.current_ << " [" << command.currentMin_ << ", " << command.currentMax_ << "]" << std::endl;
  out << "motor position: "  << command.motorPosition_ << " [" << command.motorPositionMin_ << ", " << command.motorPositionMax_ << "]" << std::endl;
  out << "motor velocity: "  << command.motorVelocity_ << " [" << command.motorVelocityMin_ << ", " << command.motorVelocityMax_ << "]" << std::endl;
  out << "gear position: "  << command.gearPosition_ << " [" << command.gearPositionMin_ << ", " << command.gearPositionMax_ << "]" << std::endl;
  out << "gear velocity: "  << command.gearVelocity_ << " [" << command.gearVelocityMin_ << ", " << command.gearVelocityMax_ << "]" << std::endl;
  out << "joint position: "  << command.jointPosition_ << " [" << command.jointPositionMin_ << ", " << command.jointPositionMax_ << "]" << std::endl;
  out << "joint velocity: "  << command.jointVelocity_ << " [" << command.jointVelocityMin_ << ", " << command.jointVelocityMax_ << "]" << std::endl;
  out << "joint torque: "  << command.jointTorque_ << " [" << command.jointTorqueMin_ << ", " << command.jointTorqueMax_ << "]" << std::endl;
  out << "pid gains: "  << command.pidGainsP_ << ", " << command.pidGainsI_ << ", " << command.pidGainsD_;
  return out;
}


} // series_elastic_actuator


