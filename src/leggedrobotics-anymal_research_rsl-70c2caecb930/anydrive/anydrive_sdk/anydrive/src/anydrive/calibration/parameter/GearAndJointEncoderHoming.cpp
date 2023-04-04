#include "anydrive/calibration/parameter/GearAndJointEncoderHoming.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

bool GearAndJointEncoderHoming::operator==(const GearAndJointEncoderHoming& other) const {
  return (gearEncoderRawTicks_ == other.gearEncoderRawTicks_ && jointEncoderRawTicks_ == other.jointEncoderRawTicks_);
}

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
