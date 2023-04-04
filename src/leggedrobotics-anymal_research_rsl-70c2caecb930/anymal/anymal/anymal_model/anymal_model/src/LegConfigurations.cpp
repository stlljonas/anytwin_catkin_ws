/*!
 * @file     LegConfigurations.cpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Dec, 2014
 */

// anymal model
#include "anymal_model/LegConfigurations.hpp"

// stl
#include <stdexcept>

namespace anymal_model {

bool LegConfigurations::operator[](int iLeg) const {
  switch (iLeg) {
    case 0:
      return isLeftForeLegBentNormal_;
    case 1:
      return isRightForeLegBentNormal_;
    case 2:
      return isLeftHindLegBentNormal_;
    case 3:
      return isRightHindLegBentNormal_;
    default:
      throw std::runtime_error("LegConfig: Wrong leg index!");
  }
  return false;
}

}  // namespace anymal_model
