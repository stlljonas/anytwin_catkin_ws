#include "anydrive/calibration/parameter/GearJointEncoderOffset.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

bool GearJointEncoderOffset::operator==(const GearJointEncoderOffset& other) const {
  return (constant_ == other.constant_ && sin1Amplitude_ == other.sin1Amplitude_ && sin1Phaseshift_ == other.sin1Phaseshift_ &&
          sin2Amplitude_ == other.sin2Amplitude_ && sin2Phaseshift_ == other.sin2Phaseshift_);
}

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
