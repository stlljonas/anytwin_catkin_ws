/*
 * DynamicGaitsController.cpp
 *
 *  Created on: Aug 20, 2014
 *      Author: C. Dario Bellicoso
 */


// signal logger
#include <signal_logger/signal_logger.hpp>

// message logger
#include <message_logger/message_logger.hpp>

// dynamic walk controller
#include "anymal_ctrl_dynamic_gaits/DynamicGaitsController.hpp"

// loco_anymal
#include <loco_anymal/motion_control/WholeBodyController.hpp>

// loco
#include <loco/contact_force_distribution/ContactForceDistribution.hpp>
#include <loco/contact_force_distribution/constraints/ForceLimitsConstraint.hpp>
#include <loco/contact_force_distribution/constraints/FrictionConstraint.hpp>
#include <loco/foothold_generation/FootholdGeneratorInvertedPendulum.hpp>
#include <loco/foothold_generation/FootholdGeneratorOptimizedInvPend.hpp>
#include <loco/terrain_perception/TerrainPerceptionFreePlane.hpp>
#include <loco/foothold_generation/FootholdGeneratorInvertedPendulumDynamicWalk.hpp>
#include <loco/common/TerrainModelFreePlane.hpp>

// terrain sensing
#include <terrain_sensing/TerrainSensing.hpp>

// swing_trajectory generation
#include <swing_trajectory_generation/SwingTrajectoryGeneratorSplineOptimized.hpp>

// loco anymal
#include <loco_anymal/heading_generation/HeadingGeneratorAnymal.hpp>
#include <loco_anymal/motion_control/ImpedanceAndVirtualModelController.hpp>
#include <loco_anymal/loco_anymal.hpp>

// numerical optimization
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>
#include <loco/foot_placement_strategy/FootPlacementStrategyOptimized.hpp>

// anymal_ctrl_dynamic_gaits modules
#include "anymal_ctrl_dynamic_gaits/modules/LimbCoordinatorOpt.hpp"
#include "anymal_ctrl_dynamic_gaits/modules/EventDetectorOpt.hpp"
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorSplineOpt.hpp>

// state checker
#include <state_checker/StateChecker.hpp>

// inverse kinematics control.
#include <inverse_kinematics_control/InverseKinematicsControl.hpp>

// anymal_motion_control
#include <anymal_motion_control/checks/BaseStateCheck.hpp>
#include <anymal_motion_control/checks/ContactStateCheck.hpp>
#include <anymal_motion_control/checks/JointPositionLimitsCheck.hpp>
#include <anymal_motion_control/checks/StateStatusCheck.hpp>

namespace anymal_ctrl_dynamic_gaits {

DynamicGaitsController::DynamicGaitsController() : DynamicGaitsController("dynamic_gaits") { }

DynamicGaitsController::DynamicGaitsController(const std::string& controllerName) :
    Base(),
    useWholeBodyController_(true),
    nominalFootholdGenerationTechnique_(loco::foothold_generator::NominalFootholdGeneration::Undefined),
    footholdGenerationOptimizer_(loco::foothold_generator::FootholdGenerationOptimizer::Undefined),
    useOptimizedSwingTrajectory_(true),
    useStateChecker_(false) {
}

bool DynamicGaitsController::loadControllerSettingsFromFile() {
  // Load task parameters.
  const std::string controllerSettingsFile = this->getParameterPath() + "/ControllerSettings" + ".xml";

  if (!controllerParameterSet_->loadXmlDocument(controllerSettingsFile)) {
    MELO_ERROR_STREAM("[DynamicGaitsController::loadControllerSettingsFromFile] Could not load parameter file with path: " << controllerSettingsFile);
    return false;
  }

  const TiXmlHandle controllerParameterSetHandle = controllerParameterSet_->getHandle();

  TiXmlHandle taskParametersHandle = controllerParameterSetHandle;
  if (!tinyxml_tools::getChildHandle(taskParametersHandle, controllerParameterSet_->getHandle(), "TaskParameters")) { return false; }

  if (!loadTaskParameters(taskParametersHandle)) { return false; }

  return true;
}

bool DynamicGaitsController::loadModulesParametersFromFile() {
  const std::string parameterFile = this->getParameterPath() + "/DefaultParams" + (this->isRealRobot() ? "" : "Sim") + ".xml";

  if (!modulesParameterSet_->loadXmlDocument(parameterFile)) {
    MELO_ERROR_STREAM("Could not load parameter file: " << parameterFile);
    return false;
  } else {
    MELO_INFO_STREAM("Loaded file: " << parameterFile);
  }
  return true;
}

bool DynamicGaitsController::create() {
  // Add additional state checks
  getInitialStateChecker().addStateCheck("stateStatus", std::make_shared<anymal_motion_control::StateStatusCheck>());
  getInitialStateChecker().addStateCheck("baseState", std::make_shared<anymal_motion_control::BaseStateCheck>(0.5, 0.5));
  getInitialStateChecker().addStateCheck("contactState", std::make_shared<anymal_motion_control::ContactStateCheck>(
      anymal_motion_control::ContactStateCheck::ContactState::CLOSED, anymal_motion_control::ContactStateCheck::CheckMode::EQUAL, 4));
  anymal_motion_control::JointPositionLimitsCheck::JointPositionLimits jointPositionLimits;
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::LF_HAA)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-35.0/180*M_PI, 35.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::LF_HFE)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(5.0/180*M_PI, 85.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::LF_KFE)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-85.0/180*M_PI, -5.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::RF_HAA)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-35.0/180*M_PI, 35.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::RF_HFE)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(5.0/180*M_PI, 85.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::RF_KFE)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-85.0/180*M_PI, -5.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::LH_HAA)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-35.0/180*M_PI, 35.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::LH_HFE)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-85.0/180*M_PI, -5.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::LH_KFE)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(5.0/180*M_PI, 85.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::RH_HAA)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-35.0/180*M_PI, 35.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::RH_HFE)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(-85.0/180*M_PI, -5.0/180*M_PI);
  jointPositionLimits[anymal_description::AnymalDescription::mapKeyEnumToKeyId(anymal_description::AnymalDescription::JointEnum::RH_KFE)] = std::make_shared<const anymal_motion_control::LowerAndUpperLimitCheck>(5.0/180*M_PI, 85.0/180*M_PI);
  getInitialStateChecker().addStateCheck("jointPositionLimits", std::make_shared<anymal_motion_control::JointPositionLimitsCheck>(jointPositionLimits));
  getStateChecker().addStateCheck("stateStatus", std::make_shared<anymal_motion_control::StateStatusCheck>());
  getStateChecker().addStateCheck("baseState", std::make_shared<anymal_motion_control::BaseStateCheck>(20.0, 4.0));

  // Create parameter set for controller settings.
  controllerParameterSet_.reset(new loco::ParameterSet());

  // Load the controller settings. This is done only once to select which modules should be created.
  if (!loadControllerSettingsFromFile()) { return false; }

  // Get helper references.
  auto& anymalModel = *getState().getAnymalModelPtr();
  auto& anymalModelDesired = *getState().getDesiredAnymalModelPtr();

  // Create the robot measurement containers.
  {
    boost::shared_lock<boost::shared_mutex> lock(this->getStateMutex());
    legs_ = loco_anymal::make_unique_leg_group(anymalModel, anymalModelDesired, false);
    torso_.reset(new loco_anymal::TorsoAnymal("torso", anymalModel));
    wholeBody_.reset(new loco_anymal::WholeBodyAnymal(anymalModel, *torso_, *legs_, true));
  }

  // Create the heading generator.
  headingGenerator_.reset(new loco_anymal::HeadingGeneratorAnymal(*wholeBody_));

  // Create the terrain model.
  terrainModel_.reset(new loco::TerrainModelFreePlane());

  // Create terrain perception.
  terrainPerception_.reset(new loco::TerrainPerceptionFreePlane(
      *terrainModel_, *wholeBody_, *headingGenerator_, loco::TerrainPerceptionFreePlane::EstimatePlaneInFrame::World,
      loco::TerrainPerceptionFreePlane::ControlFrameHeading::Hips, loco::EndEffectorContactEnum::Contact, false));

  // Create the event detector.
  eventDetector_.reset(new loco::EventDetectorOpt(*wholeBody_));

  // Create the gait pattern.
  contactSchedule_.reset(new loco::ContactScheduleZmp(*wholeBody_));

  // Create the limb coordinator.
  limbCoordinator_.reset(new loco::LimbCoordinatorOpt(*wholeBody_));

  // Create the terrain adapter
  terrainAdapter_.reset(new loco::TerrainAdapter());

  // Create the center of mass controller.
  comControl_.reset(new loco::ComSupportControlZmp(
      *wholeBody_, *contactSchedule_, *headingGenerator_,
      *terrainModel_, *terrainAdapter_, isRealRobot()));

  // Create the torso controller.
  torsoController_.reset(new loco::TorsoControlZmp(*wholeBody_, *terrainModel_, *comControl_));

  // Create the swing foot motion generator.
  if (useOptimizedSwingTrajectory_) {
    swingTrajectoryGenerator_.reset(new loco::SwingTrajectoryGeneratorSplineOptimized(*wholeBody_, *terrainModel_, *contactSchedule_));
  } else {
    swingTrajectoryGenerator_.reset(new loco::SwingTrajectoryGeneratorSplineOpt(*wholeBody_, *terrainModel_, *contactSchedule_));
  }

  // Create nominal foothold generator.
  switch(nominalFootholdGenerationTechnique_) {
    case loco::foothold_generator::NominalFootholdGeneration::InvertedPendulum : {
      footholdGenerator_.reset(new loco::FootholdGeneratorInvertedPendulum(*wholeBody_, *terrainModel_));
    } break;

    case loco::foothold_generator::NominalFootholdGeneration::InvertedPendulumMotionGen : {
      footholdGenerator_.reset(new loco::FootholdGeneratorInvertedPendulumDynamicWalk(*wholeBody_, *terrainModel_, *contactSchedule_));
    } break;

    default : {
      MELO_WARN_STREAM("[DynamicGaitsController::loadTaskParameters] Undefined nominal foothold generation.");
      return false;
    } break;
  }

  // Create foot-placement strategy.
  switch (footholdGenerationOptimizer_) {
    case loco::foothold_generator::FootholdGenerationOptimizer::BlindQP : {
      footholdGeneratorOptimized_.reset(new loco::FootholdGeneratorOptimizedInvPend(*wholeBody_));
      footPlacementStrategy_.reset(new loco::FootPlacementStrategyOptimized<loco::FootholdGeneratorOptimizedInvPend, loco::foothold_generator::FootholdPlanInvPend>(
        *wholeBody_, *terrainModel_, *swingTrajectoryGenerator_, *dynamic_cast<loco::FootholdGeneratorOptimizedInvPend*>(footholdGeneratorOptimized_.get()),
        *contactSchedule_, *headingGenerator_, *terrainAdapter_, *footholdGenerator_));
    } break;

    default : {
      MELO_WARN_STREAM("[DynamicGaitsController::loadTaskParameters] Undefined foothold generation technique.");
      return false;
    } break;
  }

  // Create the motion controller.
  if (useWholeBodyController_) {
    boost::shared_lock<boost::shared_mutex> lock(getStateMutex());
    motionControl_.reset(new loco_anymal::WholeBodyController(
        *wholeBody_, *getState().getAnymalModelPtr(), *terrainModel_));
  } else {
    std::unique_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer());
    contactForceDistribution_.reset(new loco::ContactForceDistribution(*wholeBody_, *terrainModel_, std::move(qpSolver)));
    contactForceDistribution_->addConstraint(loco::ConstraintInterfacePtr(new loco::FrictionConstraint(*wholeBody_, *terrainModel_)));
    contactForceDistribution_->addConstraint(loco::ConstraintInterfacePtr(new loco::ForceLimitsConstraint<>(*wholeBody_, *terrainModel_)));
    motionControl_.reset(
        new loco::ImpedanceAndVirtualModelController(
            dynamic_cast<loco_anymal::WholeBodyAnymal&>(*wholeBody_), *getState().getAnymalModelPtr(),
            *getState().getDesiredAnymalModelPtr(), *contactForceDistribution_));
  }

  // Create Impedance controller.
  impedanceControl_.reset(new inverse_kinematics_control::InverseKinematicsControl(*wholeBody_, /* useAsTrackingController = */ false, /* useVelocityTracking = */ true, /* useDesiredTorsoState = */ false));

  // Create state checker.
  if (useStateChecker_) {
    stateChecker_.reset(new loco::state_checker::StateChecker(*wholeBody_, *terrainModel_));
  } else {
    stateChecker_.reset(new loco::state_checker::StateCheckerBase());
  }

  // Create the mission controller.
  {
    boost::shared_lock<boost::shared_mutex> lock(getStateMutex());
    missionController_.reset(new loco::MissionControlZmp(
        getState().getDesiredRobotVelocityPtr(), getState().getDesiredRobotPosePtr(),
        *wholeBody_, *contactSchedule_));
  }

  // Create performance monitor.
  {
    boost::shared_lock<boost::shared_mutex> lock(getStateMutex());
    perfMonitor_.reset(new loco_perf::PerformanceMonitor(*torso_, *missionController_));
  }

  // Create the locomotion module manager.
  locomotionController_.reset(new loco::LocomotionControllerModules());

  // Add measurements modules.
  locomotionController_->pushBackMeasurementModules(
      { legs_->getLeftForeLegPtr(), legs_->getRightForeLegPtr(), legs_->getLeftHindLegPtr(), legs_->getRightHindLegPtr(),
        wholeBody_->getTorsoPtr(),
        wholeBody_.get(),
        contactSchedule_.get(),
        eventDetector_.get(),
        limbCoordinator_.get(),
        terrainPerception_.get(),
        stateChecker_.get(),
        perfMonitor_.get()
      }
  );

  // Add set point modules.
  locomotionController_->pushBackSetPointModules(
      { missionController_.get(),
        footPlacementStrategy_.get(),
        torsoController_.get(),
        impedanceControl_.get(),
        motionControl_.get()
      }
  );

  // Create the actuator command managers.
  commandTranslatorSupportLeg_.reset(new loco_anymal::CommandTranslator());
  commandTranslatorSwingLeg_.reset(new loco_anymal::CommandTranslator());
  commandTranslatorContactInvariant_.reset(new loco_anymal::CommandTranslator());


  // Create parameter set for modules.
  modulesParameterSet_.reset(new loco::ParameterSet());

  // Load the module parameters.
  if (!loadModulesParametersFromFile()) {
    return false;
  }

  // Get the parameter handle.
  const TiXmlHandle locomotionControllerHandle(
      modulesParameterSet_->getHandle().FirstChild("LocomotionController"));

  // Load and initialize the locomotion controller.
  if (!locomotionController_->loadParameters(locomotionControllerHandle)) {
    MELO_ERROR("[DynamicGaitsController::initialize]: Could not load parameters for locomotion controller.");
    return false;
  }

  desiredWalkBaseHeight_ = createAnymalParameter(0.545, 0.35, 0.60, "base_height", "walk", robot_control::ParameterMutability::DYNAMIC);
  desiredWalkBaseHeight_->setCallback(std::bind(&DynamicGaitsController::desiredTrotBaseHeightCallback, this));

  return true;
}

bool DynamicGaitsController::loadTaskParameters(const TiXmlHandle& handle) {
  MELO_DEBUG_STREAM(magenta << "[DynamicGaitsController] " << blue << "Load parameters." << def)

  TiXmlHandle modulesHandle = handle;
  if (!tinyxml_tools::getChildHandle(modulesHandle, handle, "Modules")) { return false; }

  // Whole Body Control.
  TiXmlHandle motionControlHandle = handle;
  if (!tinyxml_tools::getChildHandle(motionControlHandle, modulesHandle, "MotionControl")) { return false; }
  if (!tinyxml_tools::loadParameter(useWholeBodyController_, motionControlHandle, "useWholeBodyController")) { return false; }
  MELO_DEBUG_STREAM(message_logger::color::blue << "  Use " << message_logger::color::red
            << (useWholeBodyController_ ? "whole body" : "virtual model")  << message_logger::color::blue
            << " control." << message_logger::color::def)

  // Nominal footholds.
  TiXmlHandle footHoldsHandle = handle;
  if (!tinyxml_tools::getChildHandle(footHoldsHandle, modulesHandle, "FootholdGeneration")) { return false; }
  std::string nominalfootholdGenStr;
  if (!tinyxml_tools::loadParameter(nominalfootholdGenStr, footHoldsHandle, "nominal")) { return false; }
  if (nominalfootholdGenStr=="inverted_pendulum") {
    nominalFootholdGenerationTechnique_ = loco::foothold_generator::NominalFootholdGeneration::InvertedPendulum;
  } else if (nominalfootholdGenStr=="inverted_pendulum_motion_gen") {
    nominalFootholdGenerationTechnique_ = loco::foothold_generator::NominalFootholdGeneration::InvertedPendulumMotionGen;
  } else {
    MELO_WARN_STREAM("[DynamicGaitsController::loadTaskParameters] Unknown nominal foothold generation called " << nominalfootholdGenStr << ".");
    return false;
  }
  MELO_DEBUG_STREAM(message_logger::color::blue << "  Use " << message_logger::color::red << nominalfootholdGenStr
            << message_logger::color::blue << " as nominal footholds." << message_logger::color::def)

  // Foothold optimization.
  std::string footholdOptimizerStr;
  if (!tinyxml_tools::loadParameter(footholdOptimizerStr, footHoldsHandle, "optimizer")) { return false; }
  if (footholdOptimizerStr == "blind_qp") {
    footholdGenerationOptimizer_ = loco::foothold_generator::FootholdGenerationOptimizer::BlindQP;
  } else {
    MELO_WARN_STREAM("[DynamicGaitsController::loadTaskParameters] Unknown foothold optimizer called " << footholdOptimizerStr << ".");
    return false;
  }
  MELO_DEBUG_STREAM(message_logger::color::blue << "  Use " << message_logger::color::red << footholdOptimizerStr
            << message_logger::color::blue << " as foothold optimizer." << message_logger::color::def)

  // Swing trajectory.
  TiXmlHandle swingTrajectoryHandle = handle;
  if (!tinyxml_tools::getChildHandle(swingTrajectoryHandle, modulesHandle, "SwingTrajectory")) { return false; }
  if (!tinyxml_tools::loadParameter(useOptimizedSwingTrajectory_, swingTrajectoryHandle, "useOptimizedSwingTrajectory")) { return false; }
  MELO_DEBUG_STREAM(message_logger::color::blue << "  Use " << message_logger::color::red
            << (useOptimizedSwingTrajectory_ ? "optimized" : "un-optimized")  << message_logger::color::blue
            << " swing trajectory generation." << message_logger::color::def)

  // State checker.
  TiXmlHandle stateCheckerHandle = handle;
  if (!tinyxml_tools::getChildHandle(stateCheckerHandle, modulesHandle, "StateChecker")) { return false; }
  if (!tinyxml_tools::loadParameter(useStateChecker_, stateCheckerHandle, "useStateChecker")) { return false; }
  if (useStateChecker_) {
    MELO_DEBUG_STREAM(message_logger::color::blue << "  State checker is " << message_logger::color::red << "enabled"
              << message_logger::color::blue << "." << message_logger::color::def)
  } else {
    MELO_DEBUG_STREAM(message_logger::color::blue << "  State checker is " << message_logger::color::red << "disabled"
              << message_logger::color::blue << "." << message_logger::color::def)
  }

  return true;
}

bool DynamicGaitsController::initialize() {
  const double dt = getTime().getTimeStep();
  
  // Load the module parameters.
  if (!loadModulesParametersFromFile()) {
    return false;
  }

  // Get the parameter handle.
  const TiXmlHandle locomotionControllerHandle(
      modulesParameterSet_->getHandle().FirstChild("LocomotionController"));

  // Load and initialize terrain model.
  if (!terrainModel_->loadParameters(locomotionControllerHandle)) {
    MELO_ERROR("[DynamicGaitsController::initialize]: Could not load parameters for terrain model.");
    return false;
  }
  if (!terrainModel_->initialize(dt)) {
    MELO_ERROR("[DynamicGaitsController::initialize]: Could not initialize terrain model.");
    return false;
  }

  // Load and initialize the locomotion controller.
  if (!locomotionController_->loadParameters(locomotionControllerHandle)) {
    MELO_ERROR("[DynamicGaitsController::initialize]: Could not load parameters for locomotion controller.");
    return false;
  }

  // Initialize locomotion controller.
  if (!locomotionController_->initialize(dt)) {
    MELO_ERROR("[DynamicGaitsController::initialize]: Could not initialize locomotion controller.");
    return false;
  }

  // Get the controller handle.
  TiXmlHandle controllerHandle = locomotionControllerHandle;
  if (useWholeBodyController_) {
    if (!tinyxml_tools::getChildHandle(controllerHandle, locomotionControllerHandle, "WholeBodyControl")) { return false; }
  } else {
    if (!tinyxml_tools::getChildHandle(controllerHandle, locomotionControllerHandle, "VirtualModelController")) { return false; }
  }

  // Get the pid gains handle.
  TiXmlHandle pidGainsHandle = locomotionControllerHandle;
  if (!tinyxml_tools::getChildHandle(pidGainsHandle, controllerHandle, "PIDGains")) {
    return false;
  }

  // Load command parameter (control gains).
  if (!commandTranslatorSupportLeg_->loadParameters(pidGainsHandle, "SupportLegCommands")) {
    MELO_FATAL("[DynamicGaitsController::initialize] Could not load command parameters for support leg!");
    return false;
  }
  if (!commandTranslatorSwingLeg_->loadParameters(pidGainsHandle, "SwingLegCommands")) {
    MELO_FATAL("[DynamicGaitsController::initialize] Could not load command parameters for swing leg!");
    return false;
  }
  if (!commandTranslatorContactInvariant_->loadParameters(pidGainsHandle, "ContactInvariantCommands")) {
    MELO_FATAL("[DynamicGaitsController::initialize] Could not load command parameters for contact invariant leg!");
    return false;
  }

  // Add variables to logger
  contactSchedule_->addVariablesToLog("/" + getName() + "/contactSchedule");
  motionControl_->addVariablesToLog();
  terrainModel_->addVariablesToLog("/" + getName() + "/terrainModel");
  impedanceControl_->addVariablesToLog("/" + getName() + "/impedanceControl");
  for (auto leg : *legs_) {
    leg->addVariablesToLog(leg->getName());
  }
  torsoController_->addVariablesToLog("/" + getName() + "/torsoController");
  comControl_->addVariablesToLog("/" + getName() + "/comControl");
  wholeBody_->addVariablesToLog("");
  torso_->addVariablesToLog("");
  eventDetector_->addVariablesToLog();
  limbCoordinator_->addVariablesToLog();
  stateChecker_->addVariablesToLog();
  missionController_->addVariablesToLog("/" + getName() + "/missionController");
  perfMonitor_->addVariablesToLog("/" + getName() + "/perf");

  // Add parameters to handler
  locomotionController_->addParametersToHandler("/" + getName());

  return true;
}

bool DynamicGaitsController::advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  const double dt = getTime().getTimeStep();
  // Measure.
  if(!locomotionController_->advanceMeasurements(dt)) {
    MELO_WARN("locomotionController_->advanceMeasurements() returned false!");
    return false;
  }

  // Plan.
  if(!locomotionController_->advanceSetPoints(dt)) {
    MELO_WARN("LocomotionController::advanceSetPoints() returned false!");
    MELO_WARN_STREAM("Robot state:\n" << getState());
    return false;
  }

  // Act.
  {
    boost::unique_lock<boost::shared_mutex> lock(commandMutex);
    for (auto leg : *legs_) {
      const auto limbEnum = anymal_description::AnymalDescription::mapEnums<anymal_description::AnymalDescription::LimbEnum>(leg->getBranch());
      if (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Support) {
        commandTranslatorSupportLeg_->setPidGains(limbEnum, command);
      }
      else if (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactInvariant) {
        commandTranslatorContactInvariant_->setPidGains(limbEnum, command);
      }
      else {
        commandTranslatorSwingLeg_->setPidGains(limbEnum, command);
      }
    }

    commandTranslatorSupportLeg_->setCommands(command, wholeBody_->getLegs());
  }

  return true;
}

bool DynamicGaitsController::reset() {
  return initialize();
}

bool DynamicGaitsController::preStop() {
  locomotionController_->removeParametersFromHandler();
  return true;
}

bool DynamicGaitsController::stop() {
  MELO_INFO_STREAM("[DynamicGaitsController::stop] Stopping controller.");
  if(!locomotionController_->stop()) {
    MELO_WARN_STREAM("[DynamicGaitsController::stop] Failed to stop locomotion controller modules!");
  }

  MELO_INFO_STREAM("[DynamicGaitsController::stop] Stopping com-control.");
  if(!comControl_->stop()) {
    MELO_WARN_STREAM("[DynamicGaitsController::stop] Failed to stop CoM controller!");
  }

  MELO_INFO_STREAM("[DynamicGaitsController::stop] Stopping foot-placement strategy.");
  if(!footPlacementStrategy_->stop()) {
    MELO_WARN_STREAM("[DynamicGaitsController::stop] Failed to stop foot placement strategy!.");
  }

  return true;
}

const loco::LocomotionControllerModules& DynamicGaitsController::getLocomotionController() const {
  return *locomotionController_;
}

loco::LocomotionControllerModules* DynamicGaitsController::getLocomotionControllerPtr() {
  return locomotionController_.get();
}

loco::ContactScheduleZmp* DynamicGaitsController::getContactSchedulePtr()
{
  return contactSchedule_.get();
}
loco::TorsoControlZmp* DynamicGaitsController::getTorsoControlPtr()
{
  return torsoController_.get();
}
loco::ParameterSet* DynamicGaitsController::getParameterSetPtr()
{
  return modulesParameterSet_.get();
}

const loco::ContactScheduleZmp& DynamicGaitsController::getContactSchedule() const
{
  return *contactSchedule_;
}
const loco::TorsoControlZmp& DynamicGaitsController::getTorsoControl() const
{
  return *torsoController_;
}
const loco::FootPlacementStrategyBase& DynamicGaitsController::getFootPlacementStrategy() const
{
  return *footPlacementStrategy_;
}
const loco::WholeBody& DynamicGaitsController::getWholeBody() const
{
  return *wholeBody_;
}
const loco::TerrainModelBase& DynamicGaitsController::getTerrainModel() const
{
  return *terrainModel_;
}
loco::TerrainModelBase* DynamicGaitsController::getTerrainModelPtr()
{
  return terrainModel_.get();
}

loco::TerrainPerceptionBase* DynamicGaitsController::getTerrainPerceptionPtr() {
  return terrainPerception_.get();
}
const loco::HeadingGenerator& DynamicGaitsController::getHeadingGenerator() const
{
  return *headingGenerator_;
}
const loco::MotionControllerBase& DynamicGaitsController::getMotionController() const
{
  return *motionControl_;
}
const loco::ParameterSet& DynamicGaitsController::getParameterSet() const
{
  return *modulesParameterSet_;
}

bool DynamicGaitsController::usingWholeBodyController() const {
  return useWholeBodyController_;
}

bool DynamicGaitsController::usingOptimizedSwingTrajectory() const {
  return useOptimizedSwingTrajectory_;
}

anymal_motion_control::SwitchResult DynamicGaitsController::goToReferenceType(anymal_motion_control::ReferenceType /*referenceType*/) {
  return anymal_motion_control::SwitchResult::ERROR;
}

void DynamicGaitsController::goToOperationMode(const std::string& /*operationMode*/, anymal_motion_control::OperationModeAction* action) {
  action->setAborted(anymal_motion_control::SwitchResult::ERROR, "DynamicGaitsController has no modes");
}

void DynamicGaitsController::desiredTrotBaseHeightCallback() {
  auto* comControl = dynamic_cast<loco::ComSupportControlZmp*>(getTorsoControlPtr()->getComSupportControlPtr());
  if (comControl == nullptr) {
    MELO_WARN("[DynamicGaitsController::desiredTrotBaseHeightCallback] Failed to cast com support control module. Could not set height.")
    return;
  }
  comControl->setDesiredHeightAboveGroundOnFlatTerrain(desiredWalkBaseHeight_->getValue());
}

} /* namespace anymal_ctrl_dynamic_gaits */
