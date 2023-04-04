/*!
 * @file     LocomotionController.cpp
 * @author   Christian Gehring
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

// loco
#include "loco/locomotion_controller/LocomotionController.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// stl
#include <iostream>
#include <string>

namespace loco {

LocomotionController::LocomotionController(Legs& legs, TorsoBase& torso, TerrainPerceptionBase& terrainPerception,
                                           ContactDetectorBase& contactDetector, LimbCoordinatorBase& limbCoordinator,
                                           FootPlacementStrategyBase& footPlacementStrategy, TorsoControlBase& baseController,
                                           MotionControllerBase& motionController, ParameterSet* parameterSet,
                                           ContactScheduleLock& contactSchedule, TerrainModelBase& terrainModel, WholeBody& wholeBody,
                                           HeadingGenerator& headingGenerator)
    : LocomotionControllerBase(),
      legs_(legs),
      torso_(torso),
      terrainPerception_(terrainPerception),
      contactDetector_(contactDetector),
      limbCoordinator_(limbCoordinator),
      footPlacementStrategy_(footPlacementStrategy),
      torsoController_(baseController),
      motionController_(motionController),
      parameterSet_(parameterSet),
      eventDetector_(EventDetectorGaitPattern(legs, dynamic_cast<const GaitPatternBase*>(&contactSchedule))),
      contactSchedule_(contactSchedule),
      terrainModel_(terrainModel),
      wholeBody_(wholeBody),
      headingGenerator_(headingGenerator),
      timer_("LocomotionController") {}

bool LocomotionController::initialize(double dt) {
  isInitialized_ = false;

  timer_.setTimerNamespace("[locomotion_controller]");

  auto warnInitError = [](const std::string& name) {
    MELO_WARN_STREAM("[LocomotionController::initialize] Could not initialize " << name << "!");
  };

  auto warnLoadError = [](const std::string& name) {
    MELO_WARN_STREAM("[LocomotionController::initialize] Could not load parameters for " << name << "!");
  };

  for (auto leg : legs_) {
    if (!leg->initialize(dt)) {
      warnInitError(leg->getName());
      return false;
    }
  }

  if (!torso_.initialize(dt)) {
    warnInitError(torso_.getName());
    return false;
  }

  TiXmlHandle hLoco(parameterSet_->getHandle().FirstChild("LocomotionController"));

  // Whole body container.
  if (!wholeBody_.loadParameters(hLoco)) {
    warnLoadError(wholeBody_.getName());
    return false;
  }
  if (!wholeBody_.initialize(dt)) {
    warnInitError(wholeBody_.getName());
    return false;
  }

  if (!terrainModel_.loadParameters(hLoco)) {
    warnLoadError("terrain model");
    return false;
  }
  if (!terrainModel_.initialize(dt)) {
    warnInitError("terrain model");
    return false;
  }

  if (!terrainPerception_.initialize(dt)) {
    warnInitError(terrainPerception_.getName());
    return false;
  }

  if (!contactDetector_.loadParameters(hLoco)) {
    warnLoadError(contactDetector_.getName());
    return false;
  }

  if (!contactDetector_.initialize(dt)) {
    warnInitError(contactDetector_.getName());
    return false;
  }

  if (!eventDetector_.initialize(dt)) {
    warnInitError(eventDetector_.getName());
    return false;
  }

  if (!limbCoordinator_.loadParameters(hLoco)) {
    warnLoadError(limbCoordinator_.getName());
    return false;
  }
  if (!limbCoordinator_.initialize(dt)) {
    warnInitError(limbCoordinator_.getName());
    return false;
  }

  if (!contactSchedule_.loadParameters(hLoco)) {
    warnLoadError(contactSchedule_.getName());
    return false;
  }
  if (!contactSchedule_.initialize(dt)) {
    warnInitError(contactSchedule_.getName());
    return false;
  }

  if (!footPlacementStrategy_.loadParameters(hLoco)) {
    warnLoadError(footPlacementStrategy_.getName());
    return false;
  }
  if (!footPlacementStrategy_.initialize(dt)) {
    warnInitError(footPlacementStrategy_.getName());
    return false;
  }

  if (!torsoController_.loadParameters(hLoco)) {
    warnLoadError(torsoController_.getName());
    return false;
  }
  if (!torsoController_.initialize(dt)) {
    warnInitError(torsoController_.getName());
    return false;
  }

  if (!motionController_.loadParameters(hLoco)) {
    warnLoadError(motionController_.getName());
    return false;
  }
  if (!motionController_.initialize(dt)) {
    warnInitError(motionController_.getName());
    return false;
  }

  runtime_ = 0.0;

  isInitialized_ = true;
  return isInitialized_;
}

bool LocomotionController::stop() {
  bool success = true;

  for (auto leg : legs_) {
    success &= leg->stop();
  }

  success &= torso_.stop();

  success &= wholeBody_.stop();

  success &= terrainPerception_.stop();

  success &= contactDetector_.stop();

  success &= eventDetector_.stop();

  success &= limbCoordinator_.stop();

  success &= contactSchedule_.stop();

  success &= footPlacementStrategy_.stop();

  success &= torsoController_.stop();

  success &= motionController_.stop();

  return success;
}

bool LocomotionController::advanceMeasurements(double dt) {
  if (!isInitialized_) {
    return false;
  }

  //--- Update sensor measurements.
  for (auto leg : legs_) {
    //    timer_.pinTime("leg_"+leg->getName());
    if (!leg->advance(dt)) {
      MELO_WARN_STREAM("Loco: leg->advance() returned false!");
      return false;
    }
    //    timer_.splitTime("leg_"+leg->getName());
  }

  //  timer_.pinTime("torso");
  if (!torso_.advance(dt)) {
    MELO_WARN("Loco: torso_.advance() returned false!");
    return false;
  }
  //  timer_.splitTime("torso");

  //  timer_.pinTime("contact_detector");
  if (!contactDetector_.advance(dt)) {
    MELO_WARN("Loco: contactDetector_->advance() returned false!");
    return false;
  }
  //  timer_.splitTime("contact_detector");
  //---

  // update whole body contributions
  //  timer_.pinTime("whole_body");
  if (!wholeBody_.advance(dt)) {
    MELO_WARN("Loco: wholeBody_->advance() returned false!");
    return false;
  }
  //  timer_.splitTime("whole_body");

  return true;
}
bool LocomotionController::advanceSetPoints(double dt) {
  // Update timing.
  //  timer_.pinTime("gait_pattern");
  if (!contactSchedule_.advance(dt)) {
    MELO_WARN("Loco: gaitPattern_.advance() returned false!");
    return false;
  }
  //  timer_.splitTime("gait_pattern");

  /* Update legs state using the event detection */
  //  timer_.pinTime("event_detector");

  if (!eventDetector_.advance(dt)) {
    MELO_WARN("Loco: eventDetector_.advance() returned false!");
    return false;
  }
  //  timer_.splitTime("event_detector");

  //--- Update knowledge about environment
  //  timer_.pinTime("terrain_perception");
  if (!terrainPerception_.advance(dt)) {
    MELO_WARN("Loco: terrainPerception_.advance() returned false!");
    return false;
  }
  //  timer_.splitTime("terrain_perception");
  //---

  /* Decide if a leg is a supporting one */
  //  timer_.pinTime("limb_coordinator");
  if (!limbCoordinator_.advance(dt)) {
    MELO_WARN("Loco: limbCoordinator_.advance() returned false!");
    return false;
  }
  //  timer_.splitTime("limb_coordinator");

  /* Set the position or torque reference */
  //  timer_.pinTime("foot_placement_strategy");
  if (!footPlacementStrategy_.advance(dt)) {
    MELO_WARN("Loco: footPlacementStrategy_.advance() returned false!");
    return false;
  }
  //  timer_.splitTime("foot_placement_strategy");

  //  timer_.pinTime("torso_controller");
  if (!torsoController_.advance(dt)) {
    MELO_WARN("Loco: torsoController_.advance() returned false!");
    return false;
  }
  //  timer_.splitTime("torso_controller");

  //  timer_.pinTime("motion_controller");
  if (!motionController_.advance(dt)) {
    MELO_WARN_STREAM(std::string{"Loco: motionController_.advance() returned false!"} << std::endl
                                                                                      << std::string{"Torso:"} << std::endl
                                                                                      << torso_ << std::endl
                                                                                      << std::string{"Motion Controller:"} << std::endl
                                                                                      << motionController_ << std::endl);
    return false;
  }
  //  timer_.splitTime("motion_controller");

  //  MELO_INFO_THROTTLE_STREAM(1.0, timer_.asString());

  // Safety layer: check if a control mode has been set on all legs.
  //  for (auto leg : *legs_) {
  //    bool isOk = true;
  //    if (leg->isAnyJointInControlMode(ControlMode::MODE_UNDEFINED)) {
  //      MELO_WARN_STREAM("[LocomotionController::advanceSetPoints] Leg " << leg->getName() << " has undefined control mode!");
  //      isOk &= false;
  //    }
  ////    if (!isOk) return false;
  //  }

  runtime_ += dt;
  return true;
}

void LocomotionController::setParameterSet(ParameterSet* parameterSet) {
  parameterSet_ = parameterSet;
}

const TorsoBase& LocomotionController::getTorso() const {
  return torso_;
}
TorsoBase* LocomotionController::getTorsoPtr() {
  return &torso_;
}

const Legs& LocomotionController::getLegs() const {
  return legs_;
}

Legs* LocomotionController::getLegsPtr() {
  return &legs_;
}

const FootPlacementStrategyBase& LocomotionController::getFootPlacementStrategy() const {
  return footPlacementStrategy_;
}

FootPlacementStrategyBase* LocomotionController::getFootPlacementStrategyPtr() {
  return &footPlacementStrategy_;
}

const MotionControllerBase& LocomotionController::getMotionController() const {
  return motionController_;
}

MotionControllerBase* LocomotionController::getMotionControllerPtr() {
  return &motionController_;
}

const LimbCoordinatorBase& LocomotionController::getLimbCoordinator() const {
  return limbCoordinator_;
}

LimbCoordinatorBase* LocomotionController::getLimbCoordinatorPtr() {
  return &limbCoordinator_;
}

const TorsoControlBase& LocomotionController::getTorsoController() const {
  return torsoController_;
}

TorsoControlBase* LocomotionController::getTorsoControllerPtr() {
  return &torsoController_;
}

const TerrainPerceptionBase& LocomotionController::getTerrainPerception() const {
  return terrainPerception_;
}

TerrainPerceptionBase* LocomotionController::getTerrainPerceptionPtr() {
  return &terrainPerception_;
}

TerrainModelBase& LocomotionController::getTerrainModel() const {
  return terrainModel_;
}

TerrainModelBase* LocomotionController::getTerrainModelPtr() {
  return &terrainModel_;
}

const ContactScheduleLock& LocomotionController::getContactSchedule() const {
  return contactSchedule_;
}

ContactScheduleLock* LocomotionController::getContactSchedulePtr() {
  return &contactSchedule_;
}

const WholeBody& LocomotionController::getWholeBody() const {
  return wholeBody_;
}

WholeBody* LocomotionController::getWholeBodyPtr() {
  return &wholeBody_;
}

const HeadingGenerator& LocomotionController::getHeadingGenerator() const {
  return headingGenerator_;
}

HeadingGenerator* LocomotionController::getHeadingGeneratorPtr() {
  return &headingGenerator_;
}

bool LocomotionController::setToInterpolated(const LocomotionController& controller1, const LocomotionController& controller2, double t) {
  if (!limbCoordinator_.setToInterpolated(controller1.getLimbCoordinator(), controller2.getLimbCoordinator(), t)) {
    return false;
  }
  if (!torsoController_.setToInterpolated(controller1.getTorsoController(), controller2.getTorsoController(), t)) {
    return false;
  }

  if (!footPlacementStrategy_.setToInterpolated(controller1.getFootPlacementStrategy(), controller2.getFootPlacementStrategy(), t)) {
    return false;
  }

  if (!motionController_.setToInterpolated(controller1.getMotionController(), controller2.getMotionController(), t)) {
    return false;
  }

  return true;
}

} /* namespace loco */
