#include "anydrive/calibration/parameter/FrictionEstimation.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

bool FrictionEstimation::operator==(const FrictionEstimation& other) const {
  return (breakAwayFriction_ == other.breakAwayFriction_ && breakAwayFrictionBand_ == other.breakAwayFrictionBand_ &&
          viscousFrictionCoeffNeg_ == other.viscousFrictionCoeffNeg_ && viscousFrictionCoeffPos_ == other.viscousFrictionCoeffPos_);
}

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
