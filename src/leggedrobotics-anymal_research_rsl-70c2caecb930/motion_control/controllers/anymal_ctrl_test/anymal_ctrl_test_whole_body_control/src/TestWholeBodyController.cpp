/*!
 * @author  Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief   Implementation of WBC test controller
 * @date    Jul 12, 2019
 */

// anymal_ctrl_test_whole_body_control
#include "anymal_ctrl_test_whole_body_control/TestWholeBodyController.hpp"

#include <Eigen/Core>
#include <memory>

#include <anymal_motion_control/checks/StateStatusCheck.hpp>
#include <loco/common/TerrainModelFreePlane.hpp>
#include <loco/terrain_perception/TerrainPerceptionFreePlane.hpp>
#include <loco_anymal/heading_generation/HeadingGeneratorAnymal.hpp>
#include <loco_anymal/loco_anymal.hpp>
#include <loco_anymal/motion_control/WholeBodyController.hpp>
#include <loco_anymal/typedefs.hpp>

namespace anymal_ctrl_test_whole_body_control {

bool TestWholeBodyController::loadParameterSet() {
  // Load parameter file
  parameterSet_ = std::make_unique<loco::ParameterSet>();

  std::string parameterFile = this->getParameterPath() + "/TestWholeBodyController.xml";

  if (!parameterSet_->loadXmlDocument(parameterFile)) {
    MELO_ERROR_STREAM("Could not load parameter file: " << parameterFile);
    return false;
  } else {
    MELO_INFO_STREAM("Loaded file: " << parameterFile);
  }

  return true;
}

bool TestWholeBodyController::create() {
  // Load parameters
  if (!loadParameterSet()) {
    return false;
  }

  // Create a group of legs, torso and whole body container
  {
    boost::shared_lock<boost::shared_mutex> lock(this->getStateMutex());
    auto& anymalModel = *getState().getAnymalModelPtr();
    auto& anymalModelDesired = *getState().getDesiredAnymalModelPtr();
    legs_ = loco_anymal::make_unique_leg_group(anymalModel, anymalModelDesired);
    torso_ = std::make_unique<loco_anymal::TorsoAnymal>("torso", *this->getState().getAnymalModelPtr());
    wholeBody_ = std::make_unique<loco_anymal::WholeBodyAnymal>(*this->getState().getAnymalModelPtr(), *torso_, *legs_, true);
  }

  // create the heading generator
  headingGenerator_ = std::make_unique<loco_anymal::HeadingGeneratorAnymal>(*wholeBody_);

  //  create terrain model
  terrainModel_ = std::make_unique<loco::TerrainModelFreePlane>();

  // create terrain perception.
  terrainPerception_ = std::make_unique<loco::TerrainPerceptionFreePlane>(
      dynamic_cast<loco::TerrainModelFreePlane&>(*terrainModel_), *wholeBody_, *headingGenerator_,
      loco::TerrainPerceptionFreePlane::EstimatePlaneInFrame::World, loco::TerrainPerceptionFreePlane::ControlFrameHeading::Feet);

  // create torso control
  torsoControl_ = std::make_unique<TorsoControlTestWholeBodyController>(*torso_, *headingGenerator_, *terrainModel_);

  // create swingTrajectory
  swingTrajectoryGenerator_ = std::make_unique<SwingTrajectoryTestWholeBodyController>(*wholeBody_);

  motionControl_ = std::make_unique<loco_anymal::WholeBodyController>(*wholeBody_, getState().getAnymalModel(), *terrainModel_);

  /* create mission controller */
  {
    boost::shared_lock<boost::shared_mutex> lock(this->getStateMutex());
    missionController_ = std::make_unique<MissionControlTestWholeBodyController>(
        this->getState().getDesiredRobotPosePtr(), *torso_, *swingTrajectoryGenerator_, *torsoControl_,
        dynamic_cast<loco_anymal::WholeBodyController&>(*motionControl_));
  }

  locomotionController_ = std::make_unique<loco::LocomotionControllerModules>();

  // Add measurements modules.
  locomotionController_->pushBackMeasurementModules({legs_->getLeftForeLegPtr(), legs_->getRightForeLegPtr(), legs_->getLeftHindLegPtr(),
                                                     legs_->getRightHindLegPtr(), torso_.get(), wholeBody_.get(),
                                                     terrainPerception_.get()});

  // Add set point modules.
  locomotionController_->pushBackSetPointModules({torsoControl_.get(), swingTrajectoryGenerator_.get(), motionControl_.get()});

  commandTranslator_ = std::make_unique<loco_anymal::CommandTranslator>();

  motionControl_->addParametersToHandler();

  // Initialize tracking error maps
  jointTrackingErrors_.emplace(ControlLevel::POSITION, JointErrors::Zero());
  jointTrackingErrors_.emplace(ControlLevel::VELOCITY, JointErrors::Zero());
  jointTrackingErrors_.emplace(ControlLevel::ACCELERATION, JointErrors::Zero());
  eeTrackingErrors_.emplace(ControlLevel::POSITION, EndEffectorsErrors::Zero());
  eeTrackingErrors_.emplace(ControlLevel::VELOCITY, EndEffectorsErrors::Zero());
  eeTrackingErrors_.emplace(ControlLevel::ACCELERATION, EndEffectorsErrors::Zero());
  baseTrackingErrors_.emplace(ControlLevel::POSITION, BaseErrors::Zero());
  baseTrackingErrors_.emplace(ControlLevel::VELOCITY, BaseErrors::Zero());
  baseTrackingErrors_.emplace(ControlLevel::ACCELERATION, BaseErrors::Zero());

  this->setAvailableOperationModesForReferenceType(anymal_motion_control::ReferenceType::ACTION,
                                                   {"joint_motion", "ee_motion", "base_motion"});

  getStateChecker().addStateCheck("stateStatusOK", std::make_shared<anymal_motion_control::StateStatusCheck>(), false);
  getStateChecker().addStateCheck(
      "stateStatusUnknown",
      std::make_shared<anymal_motion_control::StateStatusCheck>(
          std::vector<anymal_motion_control::State::StateStatus>({anymal_motion_control::State::StateStatus::STATUS_ERROR_UNKNOWN})),
      false);

  return true;
}

bool TestWholeBodyController::isRobotStateValid() {
  boost::shared_lock<boost::shared_mutex> lock(getStateMutex());
  return (getState().getStatus() == anymal_motion_control::State::StateStatus::STATUS_OK);
}

bool TestWholeBodyController::initialize() {
  const double dt = getTime().getTimeStep();

  // Load module parameters and initialize them
  if (!loadParameterSet()) {
    return false;
  }

  // Get locomotion controller handle
  const TiXmlHandle locomotionControllerHandle(parameterSet_->getHandle().FirstChild("LocomotionController"));

  // Load and initialize terrain model.
  if (!terrainModel_->loadParameters(locomotionControllerHandle)) {
    MELO_ERROR("Could not load parameters for terrain model.");
    return false;
  }
  if (!terrainModel_->initialize(dt)) {
    MELO_ERROR("Could not initialize terrain model.");
    return false;
  }

  // Load and initialize the locomotion controller.
  if (!locomotionController_->loadParameters(locomotionControllerHandle)) {
    MELO_ERROR("Could not load parameters for locomotion controller.");
    return false;
  }

  // Initialize locomotion controller.
  if (!locomotionController_->initialize(dt)) {
    MELO_ERROR("Could not initialize locomotion controller.");
    return false;
  }

  // Get the controller handle.
  TiXmlHandle controllerHandle = locomotionControllerHandle;
  if (!tinyxml_tools::getChildHandle(controllerHandle, locomotionControllerHandle, "WholeBodyControl")) {
    return false;
  }

  // Get the pid gains handle.
  TiXmlHandle pidGainsHandle = locomotionControllerHandle;
  if (!tinyxml_tools::getChildHandle(pidGainsHandle, controllerHandle, "PIDGains")) {
    return false;
  }

  // Load impedance gains.
  if (!commandTranslator_->loadParameters(pidGainsHandle, "ActuatorCommands")) {
    MELO_FATAL("Could not load command parameters for swing legs!");
    return false;
  }

  if (!missionController_->loadParameters(locomotionControllerHandle)) {
    MELO_ERROR("Could not find parameters for mission controller!");
    return false;
  }

  if (!missionController_->initialize(dt)) {
    MELO_ERROR("Could not initialize mission controller!");
    return false;
  }

  /**
   *  Add variables to signal logger.
   */
  locomotionController_->addVariablesToLog("");

  return true;
}

bool TestWholeBodyController::advance(anymal_motion_control::Command& command, robot_control::SharedMutex& commandMutex) {
  const double dt = getTime().getTimeStep();

  // Update measurements
  if (!locomotionController_->advanceMeasurements(dt)) {
    MELO_WARN("locomotionController_->advanceMeasurements() returned false!");
    return false;
  }

  // Advance mission control
  if (!missionController_->advance(dt)) {
    MELO_WARN("missionController->advance() returned false!");
    return false;
  }

  if (missionController_->getCurrentLocomotionState() != "freeze") {
    logTrackingErrors();

    // Advance set-points.
    if (!locomotionController_->advanceSetPoints(dt)) {
      MELO_WARN("locomotionController_->advanceSetPoints() returned false!");
      MELO_WARN_STREAM("Robot state:\n" << getState());
      return false;
    }

  } else {
    // Send freeze modes
    loco::JointControlModes jointFreezeModes(3);
    jointFreezeModes << loco::JointControlMode::MODE_FREEZE, loco::JointControlMode::MODE_FREEZE, loco::JointControlMode::MODE_FREEZE;
    for (auto leg : *wholeBody_->getLegsPtr()) {
      leg->getLimbStateDesiredPtr()->setJointControlModes(jointFreezeModes);
    }
  }

  {
    boost::unique_lock<boost::shared_mutex> lock(commandMutex);
    for (auto leg : *legs_) {
      const auto limbEnum =
          anymal_description::AnymalDescription::mapEnums<anymal_description::AnymalDescription::LimbEnum>(leg->getBranch());
      commandTranslator_->setPidGains(limbEnum, command);
    }
    commandTranslator_->setCommands(command, wholeBody_->getLegs());
  }

  return true;
}

bool TestWholeBodyController::reset() {
  return initialize();
}

anymal_motion_control::SwitchResult TestWholeBodyController::goToReferenceType(anymal_motion_control::ReferenceType referenceType) {
  if (referenceType != anymal_motion_control::ReferenceType::ACTION) {
    return anymal_motion_control::SwitchResult::NOT_FOUND;
  }
  return anymal_motion_control::SwitchResult::SWITCHED;
}

void TestWholeBodyController::goToOperationMode(const std::string& operationMode, anymal_motion_control::OperationModeAction* action) {
  Mode mode = missionController_->getLocomotionModeFromName(operationMode);

  clearTrackingErrors();

  switch (mode) {
    case (Mode::ModeJointMotion): {
      MELO_DEBUG("Call to execute joint-space motion");
      if (isRealRobot() && getState().getStatus() == Base::State::StateStatus::STATUS_OK) {
        MELO_ERROR("Joint motion shall not be started with a running estimator! Aborting mode switch.");
        action->setAborted(anymal_motion_control::OperationModeAction::Result::ERROR,
                           "Joint motion cannot be started on the robot with a running estimator.");
        return;
      } else {
        disableStateEstimatorCheck();
        if (!resetStateEstimator()) {
          MELO_ERROR("Could not reset state estimator! Aborting joint motion.");
          action->setAborted(anymal_motion_control::OperationModeAction::Result::ERROR, "State estimator could not be reset.");
          return;
        }
      }
      if (missionController_->executeJointMotion()) {
        action->setSucceeded(anymal_motion_control::OperationModeAction::Result::SWITCHED, "Executed joint-space motion.");
      } else {
        action->setAborted(anymal_motion_control::OperationModeAction::Result::ERROR, "Error during execution of joint-space motion.");
      }
    } break;

    case (Mode::ModeEEMotion): {
      MELO_DEBUG("Call to execute end-effector motion");
      if (isRealRobot() && getState().getStatus() == Base::State::StateStatus::STATUS_OK) {
        MELO_ERROR("End-effector motion shall not be started with a running estimator! Aborting mode switch.");
        action->setAborted(anymal_motion_control::OperationModeAction::Result::ERROR,
                           "End-effector motion cannot be started on the robot with a running estimator.");
        return;
      } else {
        disableStateEstimatorCheck();
        if (!resetStateEstimator()) {
          MELO_ERROR("Could not reset state estimator! Aborting end-effector motion.");
          action->setAborted(anymal_motion_control::OperationModeAction::Result::ERROR, "State estimator could not be reset.");
          return;
        }
      }
      if (missionController_->executeEEMotion()) {
        action->setSucceeded(anymal_motion_control::OperationModeAction::Result::SWITCHED, "Executed end-effector motion.");
      } else {
        action->setAborted(anymal_motion_control::OperationModeAction::Result::ERROR, "Error during execution of end-effector motion.");
      }
    } break;

    case (Mode::ModeBaseMotion): {
      if (getState().getStatus() != Base::State::StateStatus::STATUS_OK) {
        MELO_ERROR("Base motion shall not be started without a running estimator! Aborting mode switch.");
        action->setAborted(anymal_motion_control::OperationModeAction::Result::ERROR,
                           "Base motion cannot be started on the robot with a running estimator.");
        return;
      } else {
        enableStateEstimatorCheck();
      }
      if (missionController_->executeBaseMotion()) {
        action->setSucceeded(anymal_motion_control::OperationModeAction::Result::SWITCHED, "Executed base motion.");
      } else {
        action->setAborted(anymal_motion_control::OperationModeAction::Result::ERROR, "Error during execution of base motion.");
      }

    } break;

    default: {
      MELO_WARN_STREAM("Requested operation mode " << operationMode << " is undefined.");
      action->setAborted(anymal_motion_control::OperationModeAction::Result::NOT_FOUND,
                         "Could not find operation mode " + operationMode + ".");
    } break;
  }
}

bool TestWholeBodyController::resetStateEstimator() {
  MELO_ERROR_STREAM("resetStateEstimator() is not implemented in TestWholeBodyController!");
  return false;
}

void TestWholeBodyController::logTrackingErrors() {
  numOfErrorSamples_++;

  JointErrors jointPositionErrors;
  JointErrors jointVelocityErrors;
  JointErrors jointAccelerationErrors;
  EndEffectorsErrors eePositionErrors;
  EndEffectorsErrors eeVelocityErrors;
  EndEffectorsErrors eeAccelerationErrors;

  // Compute errors
  for (const auto& leg : *legs_) {
    jointPositionErrors.segment(leg->getLimbUInt() * 3, 3) =
        (leg->getLimbStateDesired().getJointPositions() - leg->getLimbStateMeasured().getJointPositions());
    jointVelocityErrors.segment(leg->getLimbUInt() * 3, 3) =
        (leg->getLimbStateDesired().getJointVelocities() - leg->getLimbStateMeasured().getJointVelocities());
    jointAccelerationErrors.segment(leg->getLimbUInt() * 3, 3) =
        (leg->getLimbStateDesired().getJointAccelerations() - leg->getLimbStateMeasured().getJointAccelerations());
    eePositionErrors.col(leg->getLimbUInt()) = (leg->getEndEffector().getStateDesired().getPositionWorldToEndEffectorInWorldFrame() -
                                                leg->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame())
                                                   .toImplementation();
    eeVelocityErrors.col(leg->getLimbUInt()) = (leg->getEndEffector().getStateDesired().getLinearVelocityEndEffectorInWorldFrame() -
                                                leg->getEndEffector().getStateMeasured().getLinearVelocityEndEffectorInWorldFrame())
                                                   .toImplementation();
    eeAccelerationErrors.col(leg->getLimbUInt()) = (leg->getEndEffector().getStateDesired().getLinearAccelerationEndEffectorInWorldFrame() -
                                                    leg->getEndEffector().getStateMeasured().getLinearAccelerationEndEffectorInWorldFrame())
                                                       .toImplementation();
  }

  BaseErrors basePositionErrors =
      (torso_->getDesiredState().getPositionWorldToBaseInWorldFrame() - torso_->getMeasuredState().getPositionWorldToBaseInWorldFrame())
          .toImplementation();
  BaseErrors baseVelocityErrors = (torso_->getDesiredState().getLinearVelocityErrorInControlFrame()).toImplementation();

  // Store sum of squares
  jointTrackingErrors_.at(ControlLevel::POSITION) += jointPositionErrors.cwiseProduct(jointPositionErrors);
  jointTrackingErrors_.at(ControlLevel::VELOCITY) += jointVelocityErrors.cwiseProduct(jointVelocityErrors);
  jointTrackingErrors_.at(ControlLevel::ACCELERATION) += jointAccelerationErrors.cwiseProduct(jointAccelerationErrors);
  eeTrackingErrors_.at(ControlLevel::POSITION) += eePositionErrors.cwiseProduct(eePositionErrors);
  eeTrackingErrors_.at(ControlLevel::VELOCITY) += eeVelocityErrors.cwiseProduct(eeVelocityErrors);
  eeTrackingErrors_.at(ControlLevel::ACCELERATION) += eeAccelerationErrors.cwiseProduct(eeAccelerationErrors);
  baseTrackingErrors_.at(ControlLevel::POSITION) += basePositionErrors.cwiseProduct(basePositionErrors);
  baseTrackingErrors_.at(ControlLevel::VELOCITY) += baseVelocityErrors.cwiseProduct(baseVelocityErrors);
  // TODO (fgiraldez): find a way to get acceleration error in torso.
}

void TestWholeBodyController::clearTrackingErrors() {
  jointTrackingErrors_.at(ControlLevel::POSITION) = JointErrors::Zero();
  jointTrackingErrors_.at(ControlLevel::VELOCITY) = JointErrors::Zero();
  jointTrackingErrors_.at(ControlLevel::ACCELERATION) = JointErrors::Zero();
  eeTrackingErrors_.at(ControlLevel::POSITION) = EndEffectorsErrors::Zero();
  eeTrackingErrors_.at(ControlLevel::VELOCITY) = EndEffectorsErrors::Zero();
  eeTrackingErrors_.at(ControlLevel::ACCELERATION) = EndEffectorsErrors::Zero();
  baseTrackingErrors_.at(ControlLevel::POSITION) = BaseErrors::Zero();
  baseTrackingErrors_.at(ControlLevel::VELOCITY) = BaseErrors::Zero();
  baseTrackingErrors_.at(ControlLevel::ACCELERATION) = BaseErrors::Zero();
  numOfErrorSamples_ = 0;
}

TestWholeBodyController::JointErrors TestWholeBodyController::getJointsRmsError(TestWholeBodyController::ControlLevel controlLevel) {
  if (numOfErrorSamples_ == 0) {
    return JointErrors::Zero();
  }
  return (jointTrackingErrors_.at(controlLevel) / static_cast<double>(numOfErrorSamples_)).cwiseSqrt();
}
TestWholeBodyController::EndEffectorErrors TestWholeBodyController::getEndEffectorRmsError(
    TestWholeBodyController::ControlLevel controlLevel, unsigned int limbId) {
  if (numOfErrorSamples_ == 0) {
    return EndEffectorErrors::Zero();
  }
  return (eeTrackingErrors_.at(controlLevel).col(limbId) / static_cast<double>(numOfErrorSamples_)).cwiseSqrt();
}
TestWholeBodyController::BaseErrors TestWholeBodyController::getBaseRmsError(TestWholeBodyController::ControlLevel controlLevel) {
  if (numOfErrorSamples_ == 0) {
    return BaseErrors::Zero();
  }
  return (baseTrackingErrors_.at(controlLevel) / static_cast<double>(numOfErrorSamples_)).cwiseSqrt();
}

bool TestWholeBodyController::stop() {
  missionController_->stop();
  locomotionController_->stop();
  getStateChecker().disableStateCheck("stateStatusUnknown");
  getStateChecker().disableStateCheck("stateStatusOK");
  return true;
}
void TestWholeBodyController::enableStateEstimatorCheck() {
  getStateChecker().disableStateCheck("stateStatusUnknown");
  getStateChecker().enableStateCheck("stateStatusOK");
}
void TestWholeBodyController::disableStateEstimatorCheck() {
  getStateChecker().disableStateCheck("stateStatusOK");
  getStateChecker().enableStateCheck("stateStatusUnknown");
}

} /* namespace anymal_ctrl_test_whole_body_control */
