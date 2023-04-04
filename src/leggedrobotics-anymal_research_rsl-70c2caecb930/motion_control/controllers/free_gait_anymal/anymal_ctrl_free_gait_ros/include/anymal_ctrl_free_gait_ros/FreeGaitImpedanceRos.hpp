/*
 * FreeGaitImpedanceRos.hpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "anymal_ctrl_free_gait/FreeGaitImpedance.hpp"
#include "anymal_ctrl_free_gait_ros/FreeGaitRos.hpp"

namespace anymal_ctrl_free_gait {

class FreeGaitImpedanceRos : public FreeGaitRos<FreeGaitImpedance> {
 public:
  FreeGaitImpedanceRos() : FreeGaitRos<FreeGaitImpedance>("anymal_ctrl_free_gait") {}

  ~FreeGaitImpedanceRos() override = default;
};

}  // namespace anymal_ctrl_free_gait
