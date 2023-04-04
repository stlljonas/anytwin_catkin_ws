#pragma once

#include <cstdint>

namespace anydrive_ethercat {

struct TxPdoD {
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
  float measuredCurrentPhaseU_ = 0.0;
  float measuredVoltagePhaseU_ = 0.0;
  float measuredCurrentPhaseV_ = 0.0;
  float measuredVoltagePhaseV_ = 0.0;
  float measuredCurrentPhaseW_ = 0.0;
  float measuredVoltagePhaseW_ = 0.0;
  int32_t value1_ = 0;
  int32_t value2_ = 0;
  int32_t value3_ = 0;
  int32_t value4_ = 0;
  int32_t value5_ = 0;
  int32_t value6_ = 0;
} __attribute__((packed));

}  // namespace anydrive_ethercat
