/*!
 * @file     LocomotionControllerJump.hpp
 * @author   wko
 * @date     Jun, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

#pragma once

#include <loco/contact_force_distribution/ContactForceDistributionInterface.hpp>
#include "loco/locomotion_controller/LocomotionControllerBase.hpp"

#include "loco/common/ParameterSet.hpp"
#include "loco/common/legs/Legs.hpp"
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/contact_detection/ContactDetectorBase.hpp"
#include "loco/event_detection/EventDetector.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/motion_control/VirtualModelControllerContactInvariantDamper.hpp"
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/torso_control/TorsoControlBase.hpp"

// robot utils
#include "robot_utils/function_approximators/dynamicMovementPrimitive/GaussianKernelJumpPropagator.hpp"

namespace loco {

class LocomotionControllerJump : public LocomotionControllerBase {
 public:
  LocomotionControllerJump(Legs* legs, TorsoBase* torso, TerrainPerceptionBase* terrainPerception, ContactDetectorBase* contactDetector,
                           LimbCoordinatorBase* limbCoordinator, FootPlacementStrategyBase* footPlacementStrategy,
                           TorsoControlBase* baseController, VirtualModelControllerContactInvariantDamper* virtualModelController,
                           ContactForceDistributionInterface* contactForceDistribution, ParameterSet* parameterSet,
                           TerrainModelBase* terrainModel);
  ~LocomotionControllerJump() override;

  /*!
   * Initializes locomotion controller
   * @param dt the time step [s]
   * @return true if successful.
   */
  bool initialize(double dt) override;

  /*! Advance in time
   * @param dt  time step [s]
   */
  bool advanceMeasurements(double dt) override;
  bool advanceSetPoints(double dt) override;

  virtual TorsoBase* getTorso();
  virtual Legs* getLegs();

  TorsoControlBase* getTorsoController();
  FootPlacementStrategyBase* getFootPlacementStrategy();
  VirtualModelControllerContactInvariantDamper* getVirtualModelController();
  ContactForceDistributionInterface* getContactForceDistribution();
  LimbCoordinatorBase* getLimbCoordinator();
  TerrainPerceptionBase* getTerrainPerception();

 protected:
  GaussianKernelJumpPropagator trajectoryFollower_;
  Legs* legs_;
  TorsoBase* torso_;
  LimbCoordinatorBase* limbCoordinator_;
  TerrainPerceptionBase* terrainPerception_;
  ContactDetectorBase* contactDetector_;
  FootPlacementStrategyBase* footPlacementStrategy_;
  TorsoControlBase* torsoController_;
  VirtualModelControllerContactInvariantDamper* virtualModelController_;
  ContactForceDistributionInterface* contactForceDistribution_;
  ParameterSet* parameterSet_;
  EventDetectorBase* eventDetector_;
  TerrainModelBase* terrainModel_;
};

} /* namespace loco */
