/*
 * FreeGaitVirtualModelRos.hpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "anymal_ctrl_free_gait/FreeGaitVirtualModel.hpp"
#include "anymal_ctrl_free_gait_ros/FreeGaitRos.hpp"

namespace anymal_ctrl_free_gait {

class FreeGaitVirtualModelRos : public FreeGaitRos<FreeGaitVirtualModel> {
 public:
  FreeGaitVirtualModelRos() : FreeGaitRos<FreeGaitVirtualModel>("anymal_ctrl_free_gait_virtual_model_ros") {}

  ~FreeGaitVirtualModelRos() override = default;
};

}  // namespace anymal_ctrl_free_gait
