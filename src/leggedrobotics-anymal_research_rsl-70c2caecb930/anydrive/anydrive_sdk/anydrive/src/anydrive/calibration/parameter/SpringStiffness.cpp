#include "anydrive/calibration/parameter/SpringStiffness.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

bool SpringStiffness::operator==(const SpringStiffness& other) const {
  return (neg_ == other.neg_ && pos_ == other.pos_);
}

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
