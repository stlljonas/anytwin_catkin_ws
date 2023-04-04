#include "anydrive/calibration/parameter/ImuGyroscopeDcBias.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

bool ImuGyroscopeDcBias::operator==(const ImuGyroscopeDcBias& other) const {
  return (x_ == other.x_ && y_ == other.y_ && z_ == other.z_);
}

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
