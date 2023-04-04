/*
 * FreeGaitVirtualModel.hpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "anymal_ctrl_free_gait/FreeGait.hpp"

namespace anymal_ctrl_free_gait {

class FreeGaitVirtualModel : public FreeGait {
 public:
  FreeGaitVirtualModel() : FreeGaitVirtualModel("anymal_ctrl_free_gait_virtual_model") {}
  explicit FreeGaitVirtualModel(const std::string& controllerName) : FreeGait(controllerName) { parameterName_ = "VirtualModel"; }
  ~FreeGaitVirtualModel() override = default;

 protected:
  void setupControlModules() override;
};

}  // namespace anymal_ctrl_free_gait
