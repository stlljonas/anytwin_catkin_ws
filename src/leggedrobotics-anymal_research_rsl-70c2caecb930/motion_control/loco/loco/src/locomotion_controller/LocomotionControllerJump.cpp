/*!
 * @file     LocomotionControllerJump.cpp
 * @author   wko
 * @date     Jun, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

#include "loco/locomotion_controller/LocomotionControllerJump.hpp"

namespace loco {

LocomotionControllerJump::LocomotionControllerJump(Legs* legs, TorsoBase* torso, TerrainPerceptionBase* terrainPerception,
                                                   ContactDetectorBase* contactDetector, LimbCoordinatorBase* limbCoordinator,
                                                   FootPlacementStrategyBase* footPlacementStrategy, TorsoControlBase* baseController,
                                                   VirtualModelControllerContactInvariantDamper* virtualModelController,
                                                   ContactForceDistributionInterface* contactForceDistribution, ParameterSet* parameterSet,
                                                   TerrainModelBase* terrainModel)
    : LocomotionControllerBase(),
      legs_(legs),
      torso_(torso),
      terrainPerception_(terrainPerception),
      contactDetector_(contactDetector),
      limbCoordinator_(limbCoordinator),
      footPlacementStrategy_(footPlacementStrategy),
      torsoController_(baseController),
      virtualModelController_(virtualModelController),
      contactForceDistribution_(contactForceDistribution),
      parameterSet_(parameterSet),
      eventDetector_(new EventDetector(*legs)),
      terrainModel_(terrainModel) {}

LocomotionControllerJump::~LocomotionControllerJump() {
  delete eventDetector_;
}

bool LocomotionControllerJump::initialize(double dt) {
  isInitialized_ = false;
  runtime_ = 0.0;

  for (auto leg : *legs_) {
    if (!leg->initialize(dt)) {
      return false;
    }
  }
  if (!torso_->initialize(dt)) {
    return false;
  }

  TiXmlHandle hLoco(parameterSet_->getHandle().FirstChild("LocomotionController"));

  if (!terrainModel_->initialize(dt)) {
    return false;
  }

  if (!terrainPerception_->initialize(dt)) {
    return false;
  }

  if (!contactDetector_->initialize(dt)) {
    return false;
  }
  if (!limbCoordinator_->initialize(dt)) {
    return false;
  }
  if (!footPlacementStrategy_->loadParameters(hLoco)) {
    return false;
  }
  if (!footPlacementStrategy_->initialize(dt)) {
    return false;
  }

  if (!trajectoryFollower_.loadTrajectory(hLoco)) {
    return false;
  }

  if (!trajectoryFollower_.initialize(dt)) {
    return false;
  }

  if (!torsoController_->loadParameters(hLoco)) {
    return false;
  }
  if (!torsoController_->initialize(dt)) {
    return false;
  }

  if (!contactForceDistribution_->loadParameters(hLoco)) {
    return false;
  }

  if (!virtualModelController_->loadParameters(hLoco)) {
    return false;
  }

  isInitialized_ = true;
  return isInitialized_;
}
bool LocomotionControllerJump::advanceMeasurements(double dt) {
  if (!isInitialized_) {
    return false;
  }

  //--- Update sensor measurements.
  for (auto leg : *legs_) {
    if (!leg->advance(dt)) {
      return false;
    }
  }

  if (!torso_->advance(dt)) {
    return false;
  }

  if (!contactDetector_->advance(dt)) {
    return false;
  }
  //---
  return true;
}

bool LocomotionControllerJump::advanceSetPoints(double dt) {
  if (!isInitialized_) {
    return false;
  }

  //--- Update timing.

  int iLeg = 0;
  for (auto leg : *legs_) {
    //--- defined by the "planning" / timing
    //-- NEEDS TO BE SET FOR TERRAINPERCEPTION
    //    leg->setShouldBeGrounded()

    //    leg->setShouldBeGrounded(gaitPattern_->shouldBeLegGrounded(iLeg));
    //    leg->setStanceDuration(gaitPattern_->getStanceDuration(iLeg));
    //    leg->setSwingDuration(gaitPattern_->getStrideDuration()-gaitPattern_->getStanceDuration(iLeg));
    //    //---
    //
    //    //--- timing measurements
    //    leg->setPreviousSwingPhase(leg->getSwingPhase());
    //    leg->setSwingPhase(gaitPattern_->getSwingPhaseForLeg(iLeg));
    //    leg->setPreviousStancePhase(leg->getStancePhase());
    //    leg->setStancePhase(gaitPattern_->getStancePhaseForLeg(iLeg));
    //---

    iLeg++;
  }

  //---

  /* Update legs state using the event detection */
  if (!eventDetector_->advance(dt)) {
    return false;
  }

  //--- Update knowledge about environment
  if (!terrainPerception_->advance(dt)) {
    return false;
  }
  //---

  /* Decide if a leg is a supporting one */
  if (!limbCoordinator_->advance(dt)) {
    return false;
  }

  /* Set the position or torque reference */
  if (!footPlacementStrategy_->advance(dt)) {
    return false;
  }
  if (!torsoController_->advance(dt)) {
    return false;
  }
  if (!virtualModelController_->advance(dt)) {
    return false;
  }

  runtime_ += dt;
  return true;
}

TorsoBase* LocomotionControllerJump::getTorso() {
  return torso_;
}
Legs* LocomotionControllerJump::getLegs() {
  return legs_;
}

FootPlacementStrategyBase* LocomotionControllerJump::getFootPlacementStrategy() {
  return footPlacementStrategy_;
}

TorsoControlBase* LocomotionControllerJump::getTorsoController() {
  return torsoController_;
}

VirtualModelControllerContactInvariantDamper* LocomotionControllerJump::getVirtualModelController() {
  return virtualModelController_;
}

ContactForceDistributionInterface* LocomotionControllerJump::getContactForceDistribution() {
  return contactForceDistribution_;
}

LimbCoordinatorBase* LocomotionControllerJump::getLimbCoordinator() {
  return limbCoordinator_;
}

TerrainPerceptionBase* LocomotionControllerJump::getTerrainPerception() {
  return terrainPerception_;
}

} /* namespace loco */
