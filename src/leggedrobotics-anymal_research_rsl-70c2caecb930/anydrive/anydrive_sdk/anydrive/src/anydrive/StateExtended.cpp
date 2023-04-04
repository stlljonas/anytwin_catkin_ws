#include <cmath>
#include <sstream>

#include "anydrive/StateExtended.hpp"

namespace anydrive {

StateExtended::StateExtended() : State() {}

double StateExtended::getMotorPosition() const {
  return motorPosition_;
}

void StateExtended::setMotorPosition(const double motorPosition) {
  motorPosition_ = motorPosition;
}

double StateExtended::getMotorVelocity() const {
  return motorVelocity_;
}

void StateExtended::setMotorVelocity(const double motorVelocity) {
  motorVelocity_ = motorVelocity;
}

int32_t StateExtended::getGearPositionTicks() const {
  return gearPositionTicks_;
}

void StateExtended::setGearPositionTicks(const int32_t gearPositionTicks) {
  gearPositionTicks_ = gearPositionTicks;
}

int32_t StateExtended::getJointPositionTicks() const {
  return jointPositionTicks_;
}

void StateExtended::setJointPositionTicks(const int32_t jointPositionTicks) {
  jointPositionTicks_ = jointPositionTicks;
}

double StateExtended::getTemperature() const {
  return temperature_;
}

void StateExtended::setTemperature(const double temperature) {
  temperature_ = temperature;
}

double StateExtended::getVoltage() const {
  return voltage_;
}

void StateExtended::setVoltage(const double voltage) {
  voltage_ = voltage;
}

uint64_t StateExtended::getTimestamp() const {
  return timestamp_;
}

void StateExtended::setTimestamp(const uint64_t timestamp) {
  timestamp_ = timestamp;
}

double StateExtended::getDesiredCurrentD() const {
  return desiredCurrentD_;
}

void StateExtended::setDesiredCurrentD(const double current) {
  desiredCurrentD_ = current;
}

double StateExtended::getMeasuredCurrentD() const {
  return measuredCurrentD_;
}

void StateExtended::setMeasuredCurrentD(const double current) {
  measuredCurrentD_ = current;
}

double StateExtended::getDesiredCurrentQ() const {
  return desiredCurrentQ_;
}

void StateExtended::setDesiredCurrentQ(const double current) {
  desiredCurrentQ_ = current;
}

double StateExtended::getMeasuredCurrentQ() const {
  return measuredCurrentQ_;
}

void StateExtended::setMeasuredCurrentQ(const double current) {
  measuredCurrentQ_ = current;
}

double StateExtended::getAlpha() const {
  return alpha_;
}

void StateExtended::setAlpha(const double alpha) {
  alpha_ = alpha;
}

double StateExtended::getBeta() const {
  return beta_;
}

void StateExtended::setBeta(const double beta) {
  beta_ = beta;
}

double StateExtended::getMeasuredCurrentPhaseU() const {
  return measuredCurrentPhaseU_;
}

void StateExtended::setMeasuredCurrentPhaseU(const double current) {
  measuredCurrentPhaseU_ = current;
}

double StateExtended::getMeasuredCurrentPhaseV() const {
  return measuredCurrentPhaseV_;
}

void StateExtended::setMeasuredCurrentPhaseV(const double current) {
  measuredCurrentPhaseV_ = current;
}

double StateExtended::getMeasuredCurrentPhaseW() const {
  return measuredCurrentPhaseW_;
}

void StateExtended::setMeasuredCurrentPhaseW(const double current) {
  measuredCurrentPhaseW_ = current;
}

double StateExtended::getMeasuredVoltagePhaseU() const {
  return measuredVoltagePhaseU_;
}

void StateExtended::setMeasuredVoltagePhaseU(const double voltage) {
  measuredVoltagePhaseU_ = voltage;
}

double StateExtended::getMeasuredVoltagePhaseV() const {
  return measuredVoltagePhaseV_;
}

void StateExtended::setMeasuredVoltagePhaseV(const double voltage) {
  measuredVoltagePhaseV_ = voltage;
}

double StateExtended::getMeasuredVoltagePhaseW() const {
  return measuredVoltagePhaseW_;
}

void StateExtended::setMeasuredVoltagePhaseW(const double voltage) {
  measuredVoltagePhaseW_ = voltage;
}

double StateExtended::getDutyCycleU() const {
  return dutyCycleU_;
}

void StateExtended::setDutyCycleU(const double dutyCycle) {
  dutyCycleU_ = dutyCycle;
}

double StateExtended::getDutyCycleV() const {
  return dutyCycleV_;
}

void StateExtended::setDutyCycleV(const double dutyCycle) {
  dutyCycleV_ = dutyCycle;
}

double StateExtended::getDutyCycleW() const {
  return dutyCycleW_;
}

void StateExtended::setDutyCycleW(const double dutyCycle) {
  dutyCycleW_ = dutyCycle;
}

std::string StateExtended::asString(const std::string& prefix) const {
  std::stringstream ss;
  ss << prefix << static_cast<const State&>(*this);
  ss << prefix << "Motor position: " << motorPosition_ << std::endl;
  ss << prefix << "Motor velocity: " << motorVelocity_ << std::endl;
  ss << prefix << "Gear position ticks: " << gearPositionTicks_ << std::endl;
  ss << prefix << "Joint position ticks: " << jointPositionTicks_ << std::endl;
  ss << prefix << "Temperature: " << temperature_ << std::endl;
  ss << prefix << "Voltage: " << voltage_ << std::endl;
  ss << prefix << "Timestamp: " << timestamp_ << std::endl;
  ss << prefix << "Desired current D: " << desiredCurrentD_ << std::endl;
  ss << prefix << "Measured current D: " << measuredCurrentD_ << std::endl;
  ss << prefix << "Desired current Q: " << desiredCurrentQ_ << std::endl;
  ss << prefix << "Measured current Q: " << measuredCurrentQ_ << std::endl;
  ss << prefix << "Alpha: " << alpha_ << std::endl;
  ss << prefix << "Beta: " << beta_ << std::endl;
  ss << prefix << "Measured current phase U: " << measuredCurrentPhaseU_ << std::endl;
  ss << prefix << "Measured current phase V: " << measuredCurrentPhaseV_ << std::endl;
  ss << prefix << "Measured current phase W: " << measuredCurrentPhaseW_ << std::endl;
  ss << prefix << "Measured voltage phase U: " << measuredVoltagePhaseU_ << std::endl;
  ss << prefix << "Measured voltage phase V: " << measuredVoltagePhaseV_ << std::endl;
  ss << prefix << "Measured voltage phase W: " << measuredVoltagePhaseW_ << std::endl;
  ss << prefix << "Duty cycle U: " << dutyCycleU_ << std::endl;
  ss << prefix << "Duty cycle V: " << dutyCycleV_ << std::endl;
  ss << prefix << "Duty cycle W: " << dutyCycleW_;
  return ss.str();
}

std::ostream& operator<<(std::ostream& out, const StateExtended& stateExtended) {
  out << "State extended:" << std::endl;
  out << stateExtended.asString("  ");
  return out;
}

}  // namespace anydrive
