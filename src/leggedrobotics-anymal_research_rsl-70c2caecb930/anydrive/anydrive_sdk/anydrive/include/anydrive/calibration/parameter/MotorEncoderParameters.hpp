#pragma once

#include <cstdint>

namespace anydrive {
namespace calibration {
namespace parameter {

struct MotorEncoderParameters {
  int32_t dGain_ = 0;
  int32_t dOffs_ = 0;
  int32_t dOffc_ = 0;
  int32_t dPh_ = 0;
  int32_t aGain_ = 0;
  int32_t aOffs_ = 0;
  int32_t aOffc_ = 0;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const MotorEncoderParameters& other) const;
};

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
