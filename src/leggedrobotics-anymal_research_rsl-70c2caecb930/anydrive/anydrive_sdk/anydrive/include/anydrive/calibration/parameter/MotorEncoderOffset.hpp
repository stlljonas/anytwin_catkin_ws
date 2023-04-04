#pragma once

#include <cstdint>

namespace anydrive {
namespace calibration {
namespace parameter {

struct MotorEncoderOffset {
  int32_t value_ = 0;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const MotorEncoderOffset& other) const;
};

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
