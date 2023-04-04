/*
 * FreeGaitImpedance.hpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "anymal_ctrl_free_gait/FreeGait.hpp"

namespace anymal_ctrl_free_gait {

class FreeGaitImpedance : public FreeGait {
 public:
  FreeGaitImpedance() : FreeGaitImpedance("anymal_ctrl_free_gait_impedance") {}
  explicit FreeGaitImpedance(const std::string& controllerName) : FreeGait(controllerName) { parameterName_ = "FreeGait"; }
  ~FreeGaitImpedance() override = default;

 protected:
  void setupControlModules() override;
};

}  // namespace anymal_ctrl_free_gait
