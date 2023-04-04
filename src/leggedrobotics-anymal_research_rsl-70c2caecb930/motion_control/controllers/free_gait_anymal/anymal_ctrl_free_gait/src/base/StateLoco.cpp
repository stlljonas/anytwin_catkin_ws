/*
 * StateLoco.cpp
 *
 *  Created on: Oct 23, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include <anymal_ctrl_free_gait/base/StateLoco.hpp>

namespace free_gait {

void StateLoco::initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches) {
  State::initialize(limbs, branches);
  for (const auto& limb : limbs) {
    swingPhase_[limb] = -1.0;
    swingDuration_[limb] = 0.0;
    legLoadFactor_[limb] = 1.0;
  }
}

double StateLoco::getSwingPhase(const LimbEnum& limb) const {
  return swingPhase_.at(limb);
}

void StateLoco::setSwingPhase(const LimbEnum& limb, double phase) {
  swingPhase_[limb] = phase;
}

double StateLoco::getSwingDuration(const LimbEnum& limb) const {
  return swingDuration_.at(limb);
}

void StateLoco::setSwingDuration(const LimbEnum& limb, double duration) {
  swingDuration_[limb] = duration;
}

double StateLoco::getLegLoadFactor(const LimbEnum& limb) const {
  return legLoadFactor_.at(limb);
}

void StateLoco::setLegLoadFactor(const LimbEnum& limb, double loadFactor) {
  legLoadFactor_[limb] = loadFactor;
}

} /* namespace free_gait */
