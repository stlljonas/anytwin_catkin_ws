#include "anydrive/calibration/parameter/Calibration.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

bool Calibration::operator==(const Calibration& other) const {
  return (motorEncoderOffset_ == other.motorEncoderOffset_ && motorEncoderParameters_ == other.motorEncoderParameters_ &&
          gearJointEncoderOffset_ == other.gearJointEncoderOffset_ && gearAndJointEncoderHoming_ == other.gearAndJointEncoderHoming_ &&
          imuGyroscopeDcBias_ == other.imuGyroscopeDcBias_ && springStiffness_ == other.springStiffness_ &&
          frictionEstimation_ == other.frictionEstimation_);
}

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
