/*
 * TimePoint.hpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Dario Bellicoso
 */

#include "loco/common/end_effectors/TimePoint.hpp"

namespace loco {

constexpr unsigned int TimePoint::Now;
constexpr unsigned int TimePoint::Previous;

constexpr unsigned int TimePointGait::LiftOff;
constexpr unsigned int TimePointGait::TouchDown;

}  // namespace loco
