/*!
 * @file    SeActuatorStateExtended.cpp
 * @author  Remo Diethelm
 * @date    Jul 29, 2016
 * @version 0.0
 *
 */
// series elastic actuator
#include "series_elastic_actuator/SeActuatorStateExtended.hpp"


namespace series_elastic_actuator {


SeActuatorStateExtended::SeActuatorStateExtended()
: SeActuatorState() {}

SeActuatorStateExtended::~SeActuatorStateExtended() {}

const double& SeActuatorStateExtended::getMotorPosition() const {
  return motorPosition_;
}

void SeActuatorStateExtended::setMotorPosition(double motorPosition) {
  motorPosition_ = motorPosition;
}

const double& SeActuatorStateExtended::getMotorVelocity() const {
  return motorVelocity_;
}

void SeActuatorStateExtended::setMotorVelocity(double motorVelocity) {
  motorVelocity_ = motorVelocity;
}

const int32_t& SeActuatorStateExtended::getGearPositionTicks() const {
  return gearPositionTicks_;
}

void SeActuatorStateExtended::setGearPositionTicks(int32_t gearPositionTicks) {
  gearPositionTicks_ = gearPositionTicks;
}

const int32_t& SeActuatorStateExtended::getJointPositionTicks() const {
  return jointPositionTicks_;
}

void SeActuatorStateExtended::setJointPositionTicks(int32_t jointPositionTicks) {
  jointPositionTicks_ = jointPositionTicks;
}

const double& SeActuatorStateExtended::getTemperature() const {
  return temperature_;
}

void SeActuatorStateExtended::setTemperature(double temperature) {
  temperature_ = temperature;
}

const double& SeActuatorStateExtended::getVoltage() const {
  return voltage_;
}

void SeActuatorStateExtended::setVoltage(double voltage) {
  voltage_ = voltage;
}

const unsigned long int& SeActuatorStateExtended::getTimestamp() const {
  return timestamp_;
}

void SeActuatorStateExtended::setTimestamp(unsigned long int timestamp) {
  timestamp_ = timestamp;
}

const double& SeActuatorStateExtended::getDesiredCurrentD() const {
  return desiredCurrentD_;
}

void SeActuatorStateExtended::setDesiredCurrentD(double current) {
  desiredCurrentD_ = current;
}

const double& SeActuatorStateExtended::getMeasuredCurrentD() const {
  return measuredCurrentD_;
}

void SeActuatorStateExtended::setMeasuredCurrentD(double current) {
  measuredCurrentD_ = current;
}

const double& SeActuatorStateExtended::getDesiredCurrentQ() const {
  return desiredCurrentQ_;
}

void SeActuatorStateExtended::setDesiredCurrentQ(double current) {
  desiredCurrentQ_ = current;
}

const double& SeActuatorStateExtended::getMeasuredCurrentQ() const {
  return measuredCurrentQ_;
}

void SeActuatorStateExtended::setMeasuredCurrentQ(double current) {
  measuredCurrentQ_ = current;
}

const double& SeActuatorStateExtended::getMeasuredCurrentPhaseU() const {
  return measuredCurrentPhaseU_;
}

void SeActuatorStateExtended::setMeasuredCurrentPhaseU(double current) {
  measuredCurrentPhaseU_ = current;
}

const double& SeActuatorStateExtended::getMeasuredCurrentPhaseV() const {
  return measuredCurrentPhaseV_;
}

void SeActuatorStateExtended::setMeasuredCurrentPhaseV(double current) {
  measuredCurrentPhaseV_ = current;
}

const double& SeActuatorStateExtended::getMeasuredCurrentPhaseW() const {
  return measuredCurrentPhaseW_;
}

void SeActuatorStateExtended::setMeasuredCurrentPhaseW(double current) {
  measuredCurrentPhaseW_ = current;
}

const double& SeActuatorStateExtended::getMeasuredVoltagePhaseU() const {
  return measuredVoltagePhaseU_;
}

void SeActuatorStateExtended::setMeasuredVoltagePhaseU(double voltage) {
  measuredVoltagePhaseU_ = voltage;
}

const double& SeActuatorStateExtended::getMeasuredVoltagePhaseV() const {
  return measuredVoltagePhaseV_;
}

void SeActuatorStateExtended::setMeasuredVoltagePhaseV(double voltage) {
  measuredVoltagePhaseV_ = voltage;
}

const double& SeActuatorStateExtended::getMeasuredVoltagePhaseW() const {
  return measuredVoltagePhaseW_;
}

void SeActuatorStateExtended::setMeasuredVoltagePhaseW(double voltage) {
  measuredVoltagePhaseW_ = voltage;
}

std::ostream& operator<<(std::ostream& out, const SeActuatorStateExtended& state) {
  out << static_cast<const SeActuatorState&>(state);
  out << "Motor position: " << state.motorPosition_ << std::endl;
  out << "Motor velocity: " << state.motorVelocity_ << std::endl;
  out << "Gear position raw ticks: " << state.gearPositionTicks_ << std::endl;
  out << "Joint position raw ticks: " << state.jointPositionTicks_ << std::endl;
  out << "Temperature: " << state.temperature_ << std::endl;
  out << "Voltage: " << state.voltage_ << std::endl;
  return out;
}


} // series_elastic_actuator

