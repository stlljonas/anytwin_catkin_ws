/*
 * AdapterLoco.hpp
 *
 *  Created on: Oct 22, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <anymal_model/AnymalModel.hpp>
#include <free_gait_anymal_common/AdapterAnymal.hpp>

// Loco
#include <anymal_ctrl_free_gait/base/StateLoco.hpp>
#include <loco/common/WholeBody.hpp>
#include <loco/common/torso/TorsoBase.hpp>
#include <loco_anymal/common/LegsAnymal.hpp>

// Roco
#include <anymal_motion_control/State.hpp>

// STD
#include <memory>

// TinyXML
#include <tinyxml.h>

// Boost
#include <boost/thread.hpp>

namespace free_gait {

class AdapterLoco : public AdapterAnymal {
 public:
  AdapterLoco(anymal_model::AnymalModel& anymalModel, anymal_model::AnymalModel& anymalModelDesired, loco::WholeBody& wholeBody,
              const anymal_motion_control::State& robotState);
  ~AdapterLoco() override = default;

  /*!
   * Load parameters.
   * @return true if successful
   */
  virtual bool loadParameters(const TiXmlHandle& handle);

  //! Copying data from real robot to free gait state.
  bool resetExtrasWithRobot(const StepQueue& stepQueue, State& state) override;
  bool updateExtrasBefore(const StepQueue& stepQueue, State& state) override;
  bool updateExtrasAfter(const StepQueue& stepQueue, State& state) override;

  //! Reading state of real robot.
  bool isExecutionOk() const override;
  bool isLegGrounded(const LimbEnum& limb) const override;
  ControlSetup getControlSetup(const BranchEnum& branch) const override;

  //! State depending on real robot.
  JointVelocitiesLeg getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame(
      const LimbEnum& limb, const LinearVelocity& endEffectorLinearVelocityInWorldFrame) const override;
  JointAccelerationsLeg getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame(
      const LimbEnum& limb, const LinearAcceleration& endEffectorLinearAccelerationInWorldFrame) const override;
  LinearVelocity getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb, const JointVelocitiesLeg& jointVelocities,
                                                                 const std::string& frameId) const override;
  loco::WholeBody* getWholeBodyPtr() const override;

 protected:
  bool updateGaitPattern(const StepQueue& stepQueue, StateLoco& state) const;
  bool updateLegLoadFactor(const StepQueue& stepQueue, StateLoco& state) const;
  bool updateLog(const StepQueue& stepQueue, StateLoco& state) const;

  anymal_model::AnymalModel& anymalModelDesired_;
  loco::WholeBody& wholeBody_;
  const anymal_motion_control::State& robotState_;
  loco::Legs& legs_;
  double startUnloadingLegAtPhase_;
  double loadFactorLowerBound_;
};

} /* namespace free_gait */
