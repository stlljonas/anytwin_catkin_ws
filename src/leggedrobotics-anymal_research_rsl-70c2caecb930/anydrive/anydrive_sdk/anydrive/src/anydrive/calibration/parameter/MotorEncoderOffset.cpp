#include "anydrive/calibration/parameter/MotorEncoderOffset.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

bool MotorEncoderOffset::operator==(const MotorEncoderOffset& other) const {
  return (value_ == other.value_);
}

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
