/*
 * FreeGait.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "anymal_ctrl_free_gait/FreeGait.hpp"
#include <anymal_ctrl_free_gait/base/StateLoco.hpp>
#include "loco/common/end_effectors/EndEffectorProperties.hpp"

// Signal logger
#include "signal_logger/signal_logger.hpp"

// Message logger
#include "message_logger/message_logger.hpp"

// Free Gait
#include "anymal_ctrl_free_gait/base/AdapterLoco.hpp"
#include "anymal_ctrl_free_gait/base/GaitPatternFreeGait.hpp"
#include "anymal_ctrl_free_gait/base/ImpedanceAndVirtualModelControllerFreeGait.hpp"
#include "anymal_ctrl_free_gait/base/TorsoControlFreeGait.hpp"
#include "free_gait_core/free_gait_core.hpp"

// Loco
#include "loco/common/TerrainModelFreePlane.hpp"
#include "loco/common/WholeBodyProperties.hpp"
#include "loco/torso_control/ComSupportControlDynamicGait.hpp"

// Loco anymal
#include <loco_anymal/heading_generation/HeadingGeneratorAnymal.hpp>
#include <loco_anymal/loco_anymal.hpp>

// Anymal description
#include <anymal_description/AnymalDescription.hpp>

// anymal_motion_control
#include <anymal_motion_control/checks/BaseStateCheck.hpp>
#include <anymal_motion_control/checks/StateStatusCheck.hpp>

// STD
#include <fstream>
#include <memory>

inline bool fileExists(const std::string pathToFile) {
  return static_cast<bool>(std::ifstream(pathToFile));
}

namespace anymal_ctrl_free_gait {

FreeGait::FreeGait(const std::string& controllerName) : Base(), didSetControllerPath_(false), parameterName_("FreeGait") {
  this->setAnymalName("anymal");
}

bool FreeGait::create() {
  // Add additional state checks
  getStateChecker().addStateCheck("stateStatusOK", std::make_shared<anymal_motion_control::StateStatusCheck>(), false);
  getStateChecker().addStateCheck(
      "stateStatusUnknownOrOK",
      std::make_shared<anymal_motion_control::StateStatusCheck>(std::vector<anymal_motion_control::State::StateStatus>(
          {anymal_motion_control::State::StateStatus::STATUS_OK, anymal_motion_control::State::StateStatus::STATUS_ERROR_UNKNOWN})),
      true);
  getStateChecker().addStateCheck("baseState", std::make_shared<anymal_motion_control::BaseStateCheck>(20.0, 4.0), true);

  if (!loadParameters()) {
    return false;
  }
  setupRobot();
  setupControlModules();
  setupLocomotionController();

  return loadMoreParameters();
}

bool FreeGait::loadParameters() {
  setControllerPath(this->getParameterPath());

  if (!didSetControllerPath_) {
    MELO_FATAL("[FreeGait::create] ERROR: did not set parameter path. Try to call setControllerPath(path) first.");
  }

  std::string parameterFileName = parameterName_;
  if (!isRealRobot()) {
    parameterFileName += "Sim";
  }
  MELO_INFO_STREAM("[LocoFreeGaitTask]: Reading parameter file '" << parameterFileName << "'.");
  std::string parameterFile = pathToParameterFiles_ + "/" + parameterFileName + ".xml";

  if (!fileExists(parameterFile)) {
    MELO_ERROR_STREAM("[LocoFreeGaitTask] Could not find parameter file '" << parameterFile << "'.");
    return false;
  }

  setParameterFile(parameterFile);

  parameterSet_.reset(new loco::ParameterSet());
  if (!parameterSet_->loadXmlDocument(parameterFile_)) {
    MELO_ERROR_STREAM("Could not load parameter file '" << parameterFile_ << "'.");
    return false;
  } else {
    MELO_INFO_STREAM("Loaded parameter file '" << parameterFile_ << "'.");
  }

  stepParameters_.reset(new free_gait::StepParameters());
  TiXmlHandle hDefaultStepParameters(parameterSet_->getHandle());
  if (!tinyxml_tools::getChildHandle(hDefaultStepParameters, parameterSet_->getHandle().FirstChild("LocomotionController"),
                                     "DefaultStepParameters")) {
    return false;
  }
  if (!stepParameters_->loadParameters(hDefaultStepParameters)) {
    return false;
  }

  return true;
}

bool FreeGait::loadMoreParameters() {
  // Not so nice:
  // https://bitbucket.org/leggedrobotics/loco/issue/4/idea-more-flexible-module-handling
  TiXmlHandle hLoco(parameterSet_->getHandle().FirstChild("LocomotionController"));
  try {
    auto& adapterLoco = dynamic_cast<free_gait::AdapterLoco&>(*adapter_);
    if (!adapterLoco.loadParameters(hLoco)) {
      MELO_FATAL("[ANYmal Ctrl Free Gait/init] Could not initialize locomotion controller!");
      return false;
    }
  } catch (...) {
    MELO_ERROR_STREAM("[FreeGait::Initialize] Could not cast adapter!");
    return false;
  }

  // Load command parameter (control gains).
  if (!commandTranslatorSupportLeg_->loadParameters(parameterSet_->getHandle().FirstChild("LocomotionController"), "SupportLegCommands")) {
    MELO_FATAL("[ANYmal Ctrl Free Gait/init] Could not load command parameters for support leg!");
    return false;
  }
  if (!commandTranslatorSwingLeg_->loadParameters(parameterSet_->getHandle().FirstChild("LocomotionController"), "SwingLegCommands")) {
    MELO_FATAL("[ANYmal Ctrl Free Gait/init] Could not load command parameters for swing leg!");
    return false;
  }

  return true;
}

void FreeGait::setupRobot() {
  auto& anymalModel = *getState().getAnymalModelPtr();
  auto& anymalModelDesired = *getState().getDesiredAnymalModelPtr();

  // Create a group of legs.
  {
    boost::shared_lock<boost::shared_mutex> lock(this->getStateMutex());
    legs_ = loco_anymal::make_unique_leg_group(anymalModel, anymalModelDesired);
  }

  // Create the torso.
  {
    boost::shared_lock<boost::shared_mutex> lock(this->getStateMutex());
    torso_.reset(new loco_anymal::TorsoAnymal("torso", anymalModel));
  }

  // Create the whole body container.
  {
    boost::shared_lock<boost::shared_mutex> lock(this->getStateMutex());
    wholeBody_.reset(new loco_anymal::WholeBodyAnymal(anymalModel, *torso_, *legs_, true));
  }

  // Create the heading generator.
  headingGenerator_.reset(new loco_anymal::HeadingGeneratorAnymal(*wholeBody_));

  commandTranslatorSupportLeg_.reset(new loco_anymal::CommandTranslator());
  commandTranslatorSwingLeg_.reset(new loco_anymal::CommandTranslator());
}

void FreeGait::setupLocomotionController() {
  locomotionController_.reset(new loco::LocomotionController(*wholeBody_->getLegsPtr(), *wholeBody_->getTorsoPtr(), *terrainPerception_,
                                                             *contactDetector_, *limbCoordinator_, *footPlacementStrategy_,
                                                             *torsoController_, *motionController_, parameterSet_.get(), *gaitPattern_,
                                                             *terrainModel_, *wholeBody_, *headingGenerator_));
}

void FreeGait::setupLogger() {
  for (auto leg : *legs_) {
    leg->addVariablesToLog(leg->getName());
  }
  torso_->addVariablesToLog("");
  wholeBody_->addVariablesToLog("");
  dynamic_cast<loco::TerrainModelFreePlane*>(terrainModel_.get())->addVariablesToLog("");
  motionController_->addVariablesToLog();
  motionController_->addParametersToHandler();
}

bool FreeGait::initialize() {
  const double dt = getTime().getTimeStep();
  if (!disableCheckingStateEstimator()) {
    MELO_WARN("[ANYmal Ctrl Free Gait/init] Failed to disable checking for state estimator.");
    return false;
  }
  if (!locomotionController_->initialize(dt)) {
    MELO_FATAL("[ANYmal Ctrl Free Gait/init] Could not initialize locomotion controller!");
    return false;
  }
  setupLogger();

  executor_->initialize();
  customCommandsManager_->reset();
  timer_ = std_utils::HighResolutionClockTimer("LocoFreeGait");
  timer_.setAlpha(1.0);
  return true;
}

bool FreeGait::advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  const double dt = getTime().getTimeStep();
  this->timer_.pinTime("FreeGait::advance()");
  if (!locomotionController_->advanceMeasurements(dt)) {
    return false;
  }

  timer_.pinTime("Executor::advance()");
  free_gait::Executor::Lock executorLock(this->executor_->getMutex());
  if (!executor_->advance(dt)) {
    return false;
  }  // https://bitbucket.org/leggedrobotics/loco/issue/4/idea-more-flexible-module-handling
  if (!customCommandsManager_->update()) {
    return false;
  }
  if (!customCommandsManager_->updatePreController(dt)) {
    return false;
  }
  executorLock.unlock();
  timer_.splitTime("Executor::advance()");

  timer_.pinTime("FreeGait::advanceSetPoints()");
  if (!locomotionController_->advanceSetPoints(dt)) {
    return false;
  }
  if (!customCommandsManager_->updatePostController(dt, *legs_)) {
    return false;
  }
  timer_.splitTime("FreeGait::advanceSetPoints()");

  {
    boost::unique_lock<boost::shared_mutex> lock(commandMutex);
    for (auto leg : *legs_) {
      if ((leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Support) ||
          (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactInvariant)) {
        commandTranslatorSupportLeg_->setPidGains(
            anymal_description::AnymalDescription::mapEnums<anymal_description::AnymalDescription::LimbEnum>(leg->getBranch()), command);
      } else {
        commandTranslatorSwingLeg_->setPidGains(
            anymal_description::AnymalDescription::mapEnums<anymal_description::AnymalDescription::LimbEnum>(leg->getBranch()), command);
      }
    }

    commandTranslatorSupportLeg_->setCommands(command, locomotionController_->getWholeBodyPtr()->getLegs());
  }

  timer_.splitTime("FreeGait::advance()");
  return true;
}

bool FreeGait::change() {
  return true;
}

bool FreeGait::stop() {
  return true;
}

bool FreeGait::reset() {
  return initialize();
}

bool FreeGait::softReset(double dt) {
  if (!locomotionController_->initialize(dt)) {
    std::cout << "[LocoFreeGaitTask/init] "
              << "Could not initialize locomotion controller!" << std::endl;
    return false;
  }

  executor_->reset();
  customCommandsManager_->reset();
  return true;
}

bool FreeGait::enableCheckingStateEstimator() {
  MELO_INFO("[FreeGait] Enabling checking for state estimator.");
  getStateChecker().disableStateCheck("stateStatusUnknownOrOK");
  getStateChecker().enableStateCheck("stateStatusOK");

  try {
    if (!dynamic_cast<loco::GaitPatternFreeGait&>(*gaitPattern_).reset()) {
      return false;
    }
    if (!dynamic_cast<loco::TorsoControlFreeGait&>(*torsoController_).reset()) {
      return false;
    }
  } catch (...) {
    MELO_ERROR_STREAM("[FreeGait] Cannot execute dynamic cast!");
    return false;
  }

  return true;
}

bool FreeGait::disableCheckingStateEstimator() {
  MELO_INFO("[FreeGait] Disabling checking for state estimator.");
  getStateChecker().disableStateCheck("stateStatusOK");
  getStateChecker().enableStateCheck("stateStatusUnknownOrOK");
  return true;
}

void FreeGait::setAnymalName(const std::string& anymalName) {
  anymalName_ = anymalName;
}

void FreeGait::setParameterFile(const std::string& parameterFile) {
  std::cout << "Setting parameter file: " << parameterFile << std::endl;
  parameterFile_ = parameterFile;
}

void FreeGait::setControllerPath(const std::string& controllerPath) {
  didSetControllerPath_ = true;
  pathToParameterFiles_ = controllerPath;
}

anymal_motion_control::SwitchResult FreeGait::goToReferenceType(anymal_motion_control::ReferenceType /*referenceType*/) {
  return anymal_motion_control::SwitchResult::ERROR;
}

void FreeGait::goToOperationMode(const std::string& /*operationMode*/, anymal_motion_control::OperationModeAction* action) {
  action->setAborted(anymal_motion_control::SwitchResult::ERROR, "FreeGait has no modes");
}

}  // namespace anymal_ctrl_free_gait
