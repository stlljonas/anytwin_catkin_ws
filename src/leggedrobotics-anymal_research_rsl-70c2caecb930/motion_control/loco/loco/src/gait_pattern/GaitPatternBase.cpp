/*!
 * @file     GaitPatternBase.hpp
 * @author   Christian Gehring
 * @date     Aug, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

// loco
#include "loco/gait_pattern/GaitPatternBase.hpp"

namespace loco {

GaitPatternBase::GaitPatternBase() : ContactScheduleLock() {}

bool GaitPatternBase::setToInterpolated(const GaitPatternBase& gaitPattern1, const GaitPatternBase& gaitPattern2, double /*t*/) {
  return false;
}

bool GaitPatternBase::addVariablesToLog(bool /*update*/) {
  return true;
}

}  // end namespace loco
