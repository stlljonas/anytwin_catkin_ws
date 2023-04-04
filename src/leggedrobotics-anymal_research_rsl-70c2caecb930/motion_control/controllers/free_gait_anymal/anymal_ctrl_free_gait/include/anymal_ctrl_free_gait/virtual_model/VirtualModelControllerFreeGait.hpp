/*
 * VirtualModelControllerFreeGait.cpp
 *
 *  Created on: Apr 2, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "loco/contact_force_distribution/ContactForceDistributionInterface.hpp"
#include "loco/motion_control/VirtualModelControllerContactInvariantDamper.hpp"

#include "loco_anymal/common/WholeBodyAnymal.hpp"
#include "loco_anymal/loco_anymal.hpp"

namespace loco {

class VirtualModelControllerFreeGait : public VirtualModelControllerContactInvariantDamper {
 public:
  VirtualModelControllerFreeGait(loco_anymal::WholeBodyAnymal& wholeBody, ContactForceDistributionInterface& contactForceDistribution);

  ~VirtualModelControllerFreeGait() override = default;

  bool advance(double dt) override;
};

} /* namespace loco */
