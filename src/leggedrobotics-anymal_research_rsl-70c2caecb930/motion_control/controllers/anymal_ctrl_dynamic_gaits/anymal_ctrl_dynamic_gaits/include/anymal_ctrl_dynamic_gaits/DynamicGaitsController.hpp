/*
 * DynamicGaitsController.hpp
 *
 *  Created on: Aug 20, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// stl
#include <memory>

// loco
#include "loco/locomotion_controller/LocomotionControllerModules.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/torso_control/TorsoControlZmp.hpp"
#include "loco/torso_control/ComSupportControlZmp.hpp"
#include "loco/motion_control/MotionControllerBase.hpp"
#include "loco/contact_force_distribution/ContactForceDistributionInterface.hpp"
#include "loco/event_detection/EventDetectorBase.hpp"
#include "loco/mission_control/MissionControlZmp.hpp"
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/common/ParameterSet.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorModule.hpp"
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumBase.hpp"
#include "loco/foothold_generation/FootholdGeneratorBase.hpp"
#include "loco/common/TerrainModelPlane.hpp"
#include "loco/foothold_generation/foothold_generator.hpp"

// motion generation
#include "motion_generation/ContactScheduleZmp.hpp"

// motion generation utils
#include "motion_generation_utils/StateCheckerBase.hpp"

// loco anymal
#include <loco_anymal/loco_anymal.hpp>
#include <loco_anymal/common/CommandTranslator.hpp>

// loco perf
#include <loco_perf/PerformanceMonitor.hpp>

// robot_control
#include <anymal_motion_control/AnymalController.hpp>

namespace anymal_ctrl_dynamic_gaits {

 class DynamicGaitsController: public anymal_motion_control::AnymalController {
 public:
  using Base = anymal_motion_control::AnymalController;

 public:
  DynamicGaitsController();
  explicit DynamicGaitsController(const std::string& controllerName);
  ~DynamicGaitsController() override = default;

  // Implementation of abstract roco methods.
  bool create() override;
  bool initialize() override;
  bool advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) override;
  bool reset() override;
  bool preStop() override;
  bool stop() override;

  // Implementation of anymal controller
  anymal_motion_control::SwitchResult goToReferenceType(anymal_motion_control::ReferenceType referenceType) override;
  void goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) override;

  const loco::LocomotionControllerModules& getLocomotionController() const;
  loco::LocomotionControllerModules* getLocomotionControllerPtr();
  loco::ContactScheduleZmp* getContactSchedulePtr();
  loco::TorsoControlZmp* getTorsoControlPtr();
  loco::ParameterSet* getParameterSetPtr();

  const loco::ContactScheduleZmp& getContactSchedule() const;
  const loco::TorsoControlZmp& getTorsoControl() const;
  const loco::FootPlacementStrategyBase& getFootPlacementStrategy() const;
  const loco::WholeBody& getWholeBody() const;
  const loco::TerrainModelBase& getTerrainModel() const;
  loco::TerrainModelBase* getTerrainModelPtr();
  loco::TerrainPerceptionBase* getTerrainPerceptionPtr();
  const loco::HeadingGenerator& getHeadingGenerator() const;
  const loco::MotionControllerBase& getMotionController() const;
  const loco::ParameterSet& getParameterSet() const;

  bool usingWholeBodyController() const;
  bool usingOptimizedSwingTrajectory() const;

 protected:
  //! Load the controller settings. This is called once during create().
  bool loadControllerSettingsFromFile();

  //! Load the parameters from the controller settings file.
  bool loadTaskParameters(const TiXmlHandle& handle);

  //! Load the module parameters. This is called every time the controller is initialized.
  bool loadModulesParametersFromFile();

  //! When true, the whole body controller is used.
  bool useWholeBodyController_;

  //! Foothold generation technique for nominal foothold.
  loco::foothold_generator::NominalFootholdGeneration nominalFootholdGenerationTechnique_;

  //! Foothold generation optimizer.
  loco::foothold_generator::FootholdGenerationOptimizer footholdGenerationOptimizer_;

  //! If true minimize acceleration along swing trajectory, otherwise minimize spline coefficients.
  bool useOptimizedSwingTrajectory_;

  //! If true, a state checker is advanced that can be used to access the stability of the robot.
  bool useStateChecker_;

  // legs
  std::unique_ptr<loco_anymal::LegsAnymal> legs_;

  // body
  std::unique_ptr<loco_anymal::TorsoAnymal> torso_;

  std::unique_ptr<loco::ComSupportControlZmp> comControl_;
  std::unique_ptr<loco::TorsoControlZmp> torsoController_;
  std::unique_ptr<loco::WholeBody> wholeBody_;

  // terrain
  std::unique_ptr<loco::TerrainModelPlane> terrainModel_;
  std::unique_ptr<loco::TerrainPerceptionBase> terrainPerception_;

  // gaits
  std::unique_ptr<loco::ContactScheduleZmp> contactSchedule_;
  std::unique_ptr<loco::LimbCoordinatorBase> limbCoordinator_;
  std::unique_ptr<loco::FootPlacementStrategyBase> footPlacementStrategy_;

  // strategy
  std::unique_ptr<loco::ContactForceDistributionInterface> contactForceDistribution_;
  std::unique_ptr<loco::MotionControllerBase> motionControl_;
  std::unique_ptr<loco::MotionControllerBase> impedanceControl_;
  std::unique_ptr<loco::LocomotionControllerModules> locomotionController_;

  std::unique_ptr<loco::EventDetectorBase> eventDetector_;
  std::unique_ptr<loco::HeadingGenerator> headingGenerator_;
  std::unique_ptr<loco::SwingTrajectoryGeneratorModule> swingTrajectoryGenerator_;
  std::unique_ptr<loco::FootholdGeneratorBase> footholdGeneratorOptimized_;
  std::unique_ptr<loco::FootholdGeneratorInvertedPendulumBase> footholdGenerator_;

  // safety.
  std::unique_ptr<loco::state_checker::StateCheckerBase> stateChecker_;

  // external control/tuning
  std::unique_ptr<loco::MissionControlZmp> missionController_;
  std::unique_ptr<loco_anymal::CommandTranslator> commandTranslatorSupportLeg_;
  std::unique_ptr<loco_anymal::CommandTranslator> commandTranslatorSwingLeg_;
  std::unique_ptr<loco_anymal::CommandTranslator> commandTranslatorContactInvariant_;

  // parameters
  std::unique_ptr<loco::ParameterSet> modulesParameterSet_;
  std::unique_ptr<loco::ParameterSet> controllerParameterSet_;
  robot_control::ParameterPtr<double> desiredWalkBaseHeight_;

  // logging and perfomance monitoring
  std::unique_ptr<loco_perf::PerformanceMonitor> perfMonitor_;

  //! Terrain adaption dictates how the torso adapts to terrain variations.
  std::unique_ptr<loco::TerrainAdapter> terrainAdapter_;

 private:
  void desiredTrotBaseHeightCallback();
};

} // namespace
