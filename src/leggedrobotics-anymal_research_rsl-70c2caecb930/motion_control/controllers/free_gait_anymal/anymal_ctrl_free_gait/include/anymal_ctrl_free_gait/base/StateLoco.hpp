/*
 * StateLoco.hpp
 *
 *  Created on: Oct 23, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_core/executor/State.hpp"

namespace free_gait {

class StateLoco : public State {
 public:
  StateLoco() : State() {}
  ~StateLoco() override = default;
  friend std::ostream& operator<<(std::ostream& out, const StateLoco& state);

  void initialize(const std::vector<LimbEnum>& limbs, const std::vector<BranchEnum>& branches) override;

  double getSwingPhase(const LimbEnum& limb) const;
  void setSwingPhase(const LimbEnum& limb, double phase);
  double getSwingDuration(const LimbEnum& limb) const;
  void setSwingDuration(const LimbEnum& limb, double duration);
  double getLegLoadFactor(const LimbEnum& limb) const;
  void setLegLoadFactor(const LimbEnum& limb, double loadFactor);

 private:
  std::unordered_map<LimbEnum, double, EnumClassHash> swingPhase_;
  std::unordered_map<LimbEnum, double, EnumClassHash> swingDuration_;
  std::unordered_map<LimbEnum, double, EnumClassHash> legLoadFactor_;
};

}  // namespace free_gait
