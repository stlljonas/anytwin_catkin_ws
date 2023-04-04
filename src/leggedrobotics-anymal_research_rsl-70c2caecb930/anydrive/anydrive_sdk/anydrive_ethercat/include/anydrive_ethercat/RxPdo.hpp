#pragma once

#include <cstdint>

namespace anydrive_ethercat {

struct RxPdo {
  uint16_t controlword_ = 0;
  uint16_t modeOfOperation_ = 0;
  float desiredMotorCurrent_ = 0;
  float desiredVelocity_ = 0;
  float desiredJointTorque_ = 0.0;
  double desiredPosition_ = 0.0;
  float controlParameterA_ = 0.0;
  float controlParameterB_ = 0.0;
  float controlParameterC_ = 0.0;
  float controlParameterD_ = 0.0;
} __attribute__((packed));

}  // namespace anydrive_ethercat
