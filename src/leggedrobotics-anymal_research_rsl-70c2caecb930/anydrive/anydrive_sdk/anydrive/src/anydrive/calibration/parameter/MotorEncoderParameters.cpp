#include "anydrive/calibration/parameter/MotorEncoderParameters.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

bool MotorEncoderParameters::operator==(const MotorEncoderParameters& other) const {
  return (dGain_ == other.dGain_ && dOffs_ == other.dOffs_ && dOffc_ == other.dOffc_ && dPh_ == other.dPh_ && aGain_ == other.aGain_ &&
          aOffs_ == other.aOffs_ && aOffc_ == other.aOffc_);
}

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
