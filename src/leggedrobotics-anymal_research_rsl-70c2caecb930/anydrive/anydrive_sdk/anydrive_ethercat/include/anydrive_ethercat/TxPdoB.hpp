#pragma once

#include <cstdint>

namespace anydrive_ethercat {

struct TxPdoB {
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
  float measuredImuLinearAccelerationX_ = 0.0;
  float measuredImuLinearAccelerationY_ = 0.0;
  float measuredImuLinearAccelerationZ_ = 0.0;
  float measuredImuAngularVelocityX_ = 0.0;
  float measuredImuAngularVelocityY_ = 0.0;
  float measuredImuAngularVelocityZ_ = 0.0;
  float measuredImuAngularVelocityW_ = 0.0;
} __attribute__((packed));

}  // namespace anydrive_ethercat
