/*
 * TimePoint.hpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

namespace loco {

class TimePoint {
 public:
  constexpr static unsigned int Now = 0;
  constexpr static unsigned int Previous = 1;
};

class TimePointGait : public TimePoint {
 public:
  constexpr static unsigned int LiftOff = 2;
  constexpr static unsigned int TouchDown = 3;
};

}  // namespace loco
