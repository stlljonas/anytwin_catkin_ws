#pragma once

#include <cstdint>

namespace anydrive {
namespace calibration {
namespace parameter {

struct GearJointEncoderOffset {
  //! Constant offset [ticks].
  int32_t constant_ = 0;
  //! Amplitude of the sinusoidal offset with period 2*pi [ticks].
  float sin1Amplitude_ = 0.0;
  //! Phaseshift of the sinusoidal offset with period 2*pi [rad].
  float sin1Phaseshift_ = 0.0;
  //! Amplitude of the sinusoidal offset with period pi [ticks].
  float sin2Amplitude_ = 0.0;
  //! Phaseshift of the sinusoidal offset with period pi [rad].
  float sin2Phaseshift_ = 0.0;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const GearJointEncoderOffset& other) const;
};

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
