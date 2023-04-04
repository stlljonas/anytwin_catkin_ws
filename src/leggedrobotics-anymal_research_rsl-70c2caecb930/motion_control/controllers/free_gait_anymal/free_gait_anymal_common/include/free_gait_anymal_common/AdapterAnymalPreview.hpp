/*
 * AdapterAnymalPreview.hpp
 *
 *  Created on: Dec 2, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "free_gait_anymal_common/AdapterAnymal.hpp"

namespace free_gait {

class AdapterAnymalPreview : public AdapterAnymal
{
 private:
  using AD = anymal_description::AnymalDescription;

 public:
  AdapterAnymalPreview();
  explicit AdapterAnymalPreview(anymal_model::AnymalModel& anymalModel, std::unique_ptr<geometry_utils::TransformListener>& tfListener);
  ~AdapterAnymalPreview() override = default;

  //! Copying data from real robot to free gait state.
  bool updateExtrasAfter(const StepQueue& stepQueue, State& state);

  //! Reading state of real robot.
  bool isExecutionOk() const;
  bool isLegGrounded(const LimbEnum& limb) const;
  ControlSetup getControlSetup(const BranchEnum& branch) const;

 private:
  /*!
   * Set the joint positions for support legs from inverse kinematics.
   */
  bool writeSupportLegMotion(const StepQueue& stepQueue, State& state);
};

} /* namespace free_gait */
