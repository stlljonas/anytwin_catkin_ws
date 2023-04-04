/*!
 * @file     LocomotionController.hpp
 * @author   Christian Gehring
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

#pragma once

// loco
#include "loco/common/ParameterSet.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/common/legs/Legs.hpp"
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/contact_detection/ContactDetectorBase.hpp"
#include "loco/contact_force_distribution/ContactForceDistributionInterface.hpp"
#include "loco/event_detection/EventDetectorGaitPattern.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/locomotion_controller/LocomotionControllerBase.hpp"
#include "loco/motion_control/MotionControllerBase.hpp"
#include "loco/terrain_perception/TerrainPerceptionBase.hpp"
#include "loco/torso_control/TorsoControlBase.hpp"

// std utils
#include <std_utils/std_utils.hpp>

namespace loco {

class LocomotionController : public LocomotionControllerBase {
 public:
  LocomotionController(Legs& legs, TorsoBase& torso, TerrainPerceptionBase& terrainPerception, ContactDetectorBase& contactDetector,
                       LimbCoordinatorBase& limbCoordinator, FootPlacementStrategyBase& footPlacementStrategy,
                       TorsoControlBase& baseController, MotionControllerBase& motionController, ParameterSet* parameterSet,
                       ContactScheduleLock& contactSchedule, TerrainModelBase& terrainModel, WholeBody& wholeBody,
                       HeadingGenerator& headingGenerator);

  ~LocomotionController() override = default;

  /*!
   * Initializes locomotion controller
   * @param dt the time step [s]
   * @return true if successfull.
   */
  bool initialize(double dt) override;

  /** Function that is called after emergency stop. Use this function to properly shut down
   * threads, to enforce a certain state or to prepare a following initialization step.
   */
  bool stop() override;

  /*! Advance in time
   * @param dt  time step [s]
   */
  bool advanceMeasurements(double dt) override;
  bool advanceSetPoints(double dt) override;

  void setParameterSet(ParameterSet* parameterSet);

  const TorsoBase& getTorso() const;
  TorsoBase* getTorsoPtr();

  const Legs& getLegs() const;
  Legs* getLegsPtr();

  const FootPlacementStrategyBase& getFootPlacementStrategy() const;
  FootPlacementStrategyBase* getFootPlacementStrategyPtr();

  const MotionControllerBase& getMotionController() const;
  MotionControllerBase* getMotionControllerPtr();

  const ContactForceDistributionInterface& getContactForceDistribution() const;
  ContactForceDistributionInterface* getContactForceDistributionPtr();

  const LimbCoordinatorBase& getLimbCoordinator() const;
  LimbCoordinatorBase* getLimbCoordinatorPtr();

  const TorsoControlBase& getTorsoController() const;
  TorsoControlBase* getTorsoControllerPtr();

  const TerrainPerceptionBase& getTerrainPerception() const;
  TerrainPerceptionBase* getTerrainPerceptionPtr();

  TerrainModelBase& getTerrainModel() const;
  TerrainModelBase* getTerrainModelPtr();

  const ContactScheduleLock& getContactSchedule() const;
  ContactScheduleLock* getContactSchedulePtr();

  const WholeBody& getWholeBody() const;
  WholeBody* getWholeBodyPtr();

  const HeadingGenerator& getHeadingGenerator() const;
  HeadingGenerator* getHeadingGeneratorPtr();

  bool setToInterpolated(const LocomotionController& controller1, const LocomotionController& controller2, double t);

 protected:
  Legs& legs_;
  TorsoBase& torso_;
  TerrainPerceptionBase& terrainPerception_;
  ContactDetectorBase& contactDetector_;
  LimbCoordinatorBase& limbCoordinator_;
  FootPlacementStrategyBase& footPlacementStrategy_;
  TorsoControlBase& torsoController_;
  MotionControllerBase& motionController_;
  ParameterSet* parameterSet_;
  EventDetectorGaitPattern eventDetector_;
  ContactScheduleLock& contactSchedule_;
  TerrainModelBase& terrainModel_;
  WholeBody& wholeBody_;
  HeadingGenerator& headingGenerator_;

  std_utils::HighResolutionClockTimer timer_;
};

} /* namespace loco */
