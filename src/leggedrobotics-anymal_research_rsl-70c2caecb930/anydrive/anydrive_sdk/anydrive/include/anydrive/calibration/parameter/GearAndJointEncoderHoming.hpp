#pragma once

#include <cstdint>

namespace anydrive {
namespace calibration {
namespace parameter {

struct GearAndJointEncoderHoming {
  //! Gear encoder homing position [ticks].
  int32_t gearEncoderRawTicks_ = 0;
  //! Joint encoder homing position [ticks].
  int32_t jointEncoderRawTicks_ = 0;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const GearAndJointEncoderHoming& other) const;
};

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
