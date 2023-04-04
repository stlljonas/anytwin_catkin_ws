#pragma once

#include <cstdint>

namespace anydrive {
namespace calibration {
namespace parameter {

struct FrictionEstimation {
  //! Break away friction [Nm].
  float breakAwayFriction_ = 0.0;
  //! Break away friction band [rpm].
  float breakAwayFrictionBand_ = 0.0;
  //! Negative viscous friction coefficient [Nm/rpm].
  float viscousFrictionCoeffNeg_ = 0.0;
  //! Positive viscous friction coefficient [Nm/rpm].
  float viscousFrictionCoeffPos_ = 0.0;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const FrictionEstimation& other) const;
};

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
