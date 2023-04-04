/*
 * ImpedanceAndVirtualModelController.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// loco
#include <loco/contact_force_distribution/ContactForceDistributionInterface.hpp>
#include <loco/motion_control/VirtualModelControllerContactInvariantDamper.hpp>

// loco anymal
#include <loco_anymal/typedefs.hpp>
#include <loco_anymal/common/WholeBodyAnymal.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>

class TiXmlHandle;

namespace loco {

class ImpedanceAndVirtualModelController : public VirtualModelControllerContactInvariantDamper  {
 public:

  using AD = anymal_description::AnymalDescription;

  ImpedanceAndVirtualModelController(
      loco_anymal::WholeBodyAnymal& wholeBody,
      anymal_model::AnymalModel& anymalModel,
      anymal_model::AnymalModel& anymalModelDesired,
      ContactForceDistributionInterface& contactForceDistribution);

  ~ImpedanceAndVirtualModelController() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  bool initialize(double dt) override;
  bool advance(double dt) override;

 protected:
  void setJointPositionsFromDesiredBase();
  void setJointVelocitiesFromDesiredBase();
  bool setControlModeForLimbs() override;

 private:
  anymal_model::AnymalModel& anymalModel_;
  anymal_model::AnymalModel& anymalModelDesired_;
  double timeSinceInitialization_;
  double interpolationDuration_;
};

} /* namespace loco */
