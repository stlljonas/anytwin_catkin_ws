/*
 * FreeGaitVirtualModel.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "anymal_ctrl_free_gait/FreeGaitVirtualModel.hpp"

#include "anymal_ctrl_free_gait/base/AdapterLoco.hpp"
#include "anymal_ctrl_free_gait/base/FootPlacementStrategyFreeGait.hpp"
#include "anymal_ctrl_free_gait/base/GaitPatternFreeGait.hpp"
#include "anymal_ctrl_free_gait/base/LimbCoordinatorFreeGait.hpp"
#include "anymal_ctrl_free_gait/base/StepComputerMultiThreaded.hpp"
#include "anymal_ctrl_free_gait/base/TerrainModelFreeGait.hpp"
#include "anymal_ctrl_free_gait/base/TorsoControlFreeGait.hpp"
#include "anymal_ctrl_free_gait/virtual_model/VirtualModelControllerFreeGait.hpp"

// Loco
#include <loco/contact_force_distribution/constraints/ForceLimitsConstraint.hpp>
#include <loco/contact_force_distribution/constraints/FrictionConstraint.hpp>
#include "loco/contact_detection/ContactDetectorFeedThrough.hpp"
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include "loco/locomotion_controller/LocomotionController.hpp"
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"
#include "loco/torso_control/ComSupportControlDynamicGait.hpp"

// Numerical optimization
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>

namespace anymal_ctrl_free_gait {

void FreeGaitVirtualModel::setupControlModules() {
  auto& anymalModel = *getState().getAnymalModelPtr();
  auto& anymalModelDesired = *getState().getDesiredAnymalModelPtr();

  executorState_.reset(new free_gait::StateLoco());

  adapter_.reset(new free_gait::AdapterLoco(anymalModel, anymalModelDesired, *wholeBody_, getState()));

  stepCompleter_.reset(new free_gait::StepCompleter(*stepParameters_, *adapter_));
  stepComputer_.reset(new free_gait::StepComputerMultiThreaded());

  executor_.reset(new free_gait::Executor(*stepCompleter_, *stepComputer_, *adapter_, *executorState_));
  customCommandsManager_.reset(new CustomCommandsManager(*executor_));

  terrainModel_.reset(new loco::TerrainModelFreeGait(*executor_));
  terrainPerception_.reset(
      new loco::TerrainPerceptionFreePlane(dynamic_cast<loco::TerrainModelFreePlane&>(*terrainModel_), *wholeBody_, *headingGenerator_));
  contactDetector_.reset(new loco::ContactDetectorFeedThrough());
  gaitPattern_.reset(new loco::GaitPatternFreeGait(*legs_, *torso_, anymalModel, *executor_));
  limbCoordinator_.reset(new loco::LimbCoordinatorFreeGait(*legs_, *torso_, *executor_, *gaitPattern_));
  comControl_.reset(new loco::ComSupportControlDynamicGait(*wholeBody_->getLegsPtr()));
  torsoController_.reset(new loco::TorsoControlFreeGait(*wholeBody_, anymalModelDesired, *terrainModel_, *executor_, *comControl_));
  footPlacementStrategy_.reset(
      new loco::FootPlacementStrategyFreeGait(*legs_, *torso_, anymalModel, anymalModelDesired, *terrainModel_, *wholeBody_, *executor_));
  {
    std::unique_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer());
    contactForceDistribution_.reset(new loco::ContactForceDistribution(*wholeBody_, *terrainModel_, std::move(qpSolver)));
    contactForceDistribution_->addConstraint(loco::ConstraintInterfacePtr(new loco::FrictionConstraint(*wholeBody_, *terrainModel_)));
    contactForceDistribution_->addConstraint(loco::ConstraintInterfacePtr(new loco::ForceLimitsConstraint<>(*wholeBody_, *terrainModel_)));
  }
  motionController_.reset(new loco::VirtualModelControllerFreeGait(*wholeBody_, *contactForceDistribution_));
}

}  // namespace anymal_ctrl_free_gait
