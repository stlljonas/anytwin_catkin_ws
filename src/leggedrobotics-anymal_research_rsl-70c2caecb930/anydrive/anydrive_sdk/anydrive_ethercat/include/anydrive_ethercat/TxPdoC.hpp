#pragma once

#include <cstdint>

namespace anydrive_ethercat {

struct TxPdoC {
  uint32_t statusword_ = 0;
  uint16_t measuredTemperature_ = 0;
  uint16_t measuredMotorVoltage_ = 0;
  double measuredMotorPosition_ = 0.0;
  double measuredGearPosition_ = 0.0;
  double measuredJointPosition_ = 0.0;
  float measuredMotorCurrent_ = 0.0;
  float measuredMotorVelocity_ = 0.0;
  float measuredGearVelocity_ = 0.0;
  float measuredJointVelocity_ = 0.0;
  float measuredJointAcceleration_ = 0.0;
  float measuredJointTorque_ = 0.0;
  int32_t measuredGearPositionTicks_ = 0;
  int32_t measuredJointPositionTicks_ = 0;
  uint64_t timestamp_ = 0;
  float desiredCurrentD_ = 0.0;
  float measuredCurrentD_ = 0.0;
  float desiredCurrentQ_ = 0.0;
  float measuredCurrentQ_ = 0.0;
  float alpha_ = 0.0;
  float beta_ = 0.0;
  float dutyCycleU_ = 0.0;
  float dutyCycleV_ = 0.0;
  float dutyCycleW_ = 0.0;
  float measuredCurrentPhaseU_ = 0.0;
  float measuredCurrentPhaseV_ = 0.0;
  float measuredCurrentPhaseW_ = 0.0;
  float measuredVoltagePhaseU_ = 0.0;
  float measuredVoltagePhaseV_ = 0.0;
  float measuredVoltagePhaseW_ = 0.0;
  float desiredMotorVelocity_ = 0.0;
  double desiredGearPosition_ = 0.0;
  float desiredGearVelocity_ = 0.0;
  double desiredJointPosition_ = 0.0;
  float desiredJointVelocity_ = 0.0;
  float desiredJointTorque_ = 0.0;
  float measuredImuLinearAccelerationX_ = 0.0;
  float measuredImuLinearAccelerationY_ = 0.0;
  float measuredImuLinearAccelerationZ_ = 0.0;
  float measuredImuAngularVelocityX_ = 0.0;
  float measuredImuAngularVelocityY_ = 0.0;
  float measuredImuAngularVelocityZ_ = 0.0;
  float measuredImuAngularVelocityW_ = 0.0;
} __attribute__((packed));

}  // namespace anydrive_ethercat
