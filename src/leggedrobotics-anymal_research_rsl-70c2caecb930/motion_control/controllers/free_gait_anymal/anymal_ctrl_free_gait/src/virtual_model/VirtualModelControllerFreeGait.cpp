/*
 * VirtualModelControllerFreeGait.cpp
 *
 *  Created on: Apr 2, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "anymal_ctrl_free_gait/virtual_model/VirtualModelControllerFreeGait.hpp"

namespace loco {

VirtualModelControllerFreeGait::VirtualModelControllerFreeGait(loco_anymal::WholeBodyAnymal& wholeBody,
                                                               ContactForceDistributionInterface& contactForceDistribution)
    : VirtualModelControllerContactInvariantDamper(wholeBody, contactForceDistribution) {}

bool VirtualModelControllerFreeGait::advance(double dt) {
  return VirtualModelControllerContactInvariantDamper::advance(dt);
}

} /* namespace loco */
