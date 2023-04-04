/*
 * FreeGait.hpp
 *
 *  Created on: Jan 13, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Anymal model

#include "anymal_model/AnymalModel.hpp"

// anymal motion control
#include <anymal_motion_control/AnymalController.hpp>

// Free Gait
#include <free_gait_anymal_common/AdapterAnymal.hpp>
#include "anymal_ctrl_free_gait/base/StepComputerMultiThreaded.hpp"
#include "anymal_ctrl_free_gait/custom_commands/CustomCommandsManager.hpp"
#include "free_gait_core/free_gait_core.hpp"

// loco
#include <loco/common/ParameterSet.hpp>
#include <loco/common/TerrainModelBase.hpp>
#include <loco/common/WholeBody.hpp>
#include <loco/common/end_effectors/FootBase.hpp>
#include <loco/contact_detection/ContactDetectorBase.hpp>
#include <loco/foot_placement_strategy/FootPlacementStrategyBase.hpp>
#include <loco/gait_pattern/GaitPatternBase.hpp>
#include <loco/heading_generation/HeadingGenerator.hpp>
#include <loco/limb_coordinator/LimbCoordinatorBase.hpp>
#include <loco/locomotion_controller/LocomotionController.hpp>
#include <loco/locomotion_controller/LocomotionControllerBase.hpp>
#include <loco/motion_control/MotionControllerBase.hpp>
#include <loco/terrain_perception/TerrainPerceptionBase.hpp>
#include <loco/torso_control/ComSupportControlBase.hpp>
#include <loco/torso_control/TorsoControlBase.hpp>
#include "loco/contact_force_distribution/ContactForceDistributionInterface.hpp"

// loco anymal
#include <loco_anymal/common/CommandTranslator.hpp>
#include "loco_anymal/common/LegsAnymal.hpp"
#include "loco_anymal/common/WholeBodyAnymal.hpp"

// Num opt.
#include <numopt_common/QuadraticProblemSolver.hpp>

// STD
#include <unordered_map>

// Robot utils
#include <std_utils/timers/ChronoTimer.hpp>

namespace anymal_ctrl_free_gait {

class FreeGait : public anymal_motion_control::AnymalController {
 public:
  using AD = anymal_description::AnymalDescription;
  using Base = anymal_motion_control::AnymalController;

  FreeGait() : FreeGait("anymal_ctrl_free_gait") {}
  explicit FreeGait(const std::string& controllerName);
  ~FreeGait() override = default;

  //! roco implementation.
  bool create() override;
  bool initialize() override;
  bool advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) override;
  bool reset() override;
  virtual bool softReset(double dt);
  virtual bool enableCheckingStateEstimator();
  virtual bool disableCheckingStateEstimator();
  virtual void setAnymalName(const std::string& anymalName);
  bool stop() override;
  virtual bool change();

  virtual void setControllerPath(const std::string& controllerPath);
  void setParameterFile(const std::string& parameterFile);

  // Implementation of anymal controller
  anymal_motion_control::SwitchResult goToReferenceType(anymal_motion_control::ReferenceType referenceType) override;
  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override;

 private:
  std::string pathToParameterFiles_;
  std::string parameterFile_;
  bool didSetControllerPath_;

 protected:
  virtual bool loadParameters();
  virtual bool loadMoreParameters();
  virtual void setupRobot();
  virtual void setupControlModules() = 0;
  virtual void setupLocomotionController();
  virtual void setupLogger();

  std_utils::HighResolutionClockTimer timer_;
  std::string anymalName_;
  std::string parameterName_;

 public:
  //! Legs.
  std::unique_ptr<loco_anymal::LegsAnymal> legs_;
  std::unique_ptr<loco_anymal::CommandTranslator> commandTranslatorSupportLeg_;
  std::unique_ptr<loco_anymal::CommandTranslator> commandTranslatorSwingLeg_;

  //! Base.
  std::unique_ptr<loco_anymal::TorsoAnymal> torso_;
  std::unique_ptr<loco::TorsoControlBase> torsoController_;
  std::unique_ptr<loco::ComSupportControlBase> comControl_;
  std::unique_ptr<loco::HeadingGenerator> headingGenerator_;

  //! Free gait.
  std::unique_ptr<free_gait::Executor> executor_;
  std::unique_ptr<free_gait::AdapterAnymal> adapter_;
  std::unique_ptr<CustomCommandsManager> customCommandsManager_;

  std::unique_ptr<free_gait::State> executorState_;
  std::unique_ptr<free_gait::StepParameters> stepParameters_;
  std::unique_ptr<free_gait::StepCompleter> stepCompleter_;
  std::unique_ptr<free_gait::StepComputerMultiThreaded> stepComputer_;

  //! Terrain.
  std::unique_ptr<loco::TerrainModelBase> terrainModel_;
  std::unique_ptr<loco::TerrainPerceptionBase> terrainPerception_;

  //! Gaits.
  std::unique_ptr<loco::GaitPatternBase> gaitPattern_;
  std::unique_ptr<loco::LimbCoordinatorBase> limbCoordinator_;
  std::unique_ptr<loco::FootPlacementStrategyBase> footPlacementStrategy_;

  //! Strategy.
  std::unique_ptr<loco::ContactForceDistributionInterface> contactForceDistribution_;
  std::unique_ptr<loco::MotionControllerBase> motionController_;
  std::unique_ptr<loco::LocomotionController> locomotionController_;
  std::unique_ptr<loco::ParameterSet> parameterSet_;
  std::unique_ptr<loco::ContactDetectorBase> contactDetector_;
  std::unique_ptr<loco_anymal::WholeBodyAnymal> wholeBody_;
};

}  // namespace anymal_ctrl_free_gait
