/*!
 * @file 	FootPlacementStrategyBase.cpp
 * @author 	Christian Gehring, Stelian Coros
 * @date		Sep 7, 2012
 * @version 	1.0
 * @ingroup 	robotTask
 * @brief
 */

#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"

namespace loco {

FootPlacementStrategyBase::FootPlacementStrategyBase() : isFirstTimeInit_(true) {}

bool FootPlacementStrategyBase::setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1,
                                                  const FootPlacementStrategyBase& footPlacementStrategy2, double t) {
  std::cout << "Warning: called unimplemented FootPlacementStrategyBase::setToInterpolated" << std::endl;
  return false;
}

bool FootPlacementStrategyBase::goToStand() {
  return false;
}

bool FootPlacementStrategyBase::resumeWalking() {
  return false;
}

}  // namespace loco
