/*
 * WholeBodyController.cpp
 *
 *  Created on: Jun 5, 2015
 *      Author: Dario Bellicoso
 */

// loco_anymal
#include "whole_body_control_romo/motion_control/WholeBodyController.hpp"

// loco
#include <loco/common/loco_common.hpp>
#include <loco/common/topology_conversions.hpp>

// robo utils
#include <robot_utils/math/math.hpp>

// console log
#include "message_logger/message_logger.hpp"

// algebra
#include <kindr/Core>

// signal logger
#include <signal_logger/signal_logger.hpp>

// hierarchical optimization
#include <hierarchical_optimization/HierarchicalOptimizationNullSpaceProjection.hpp>
#include <hierarchical_optimization/HierarchicalOptimizationQpCascade.hpp>
#include <hierarchical_optimization/HierarchicalOptimizationQpNullSpaceProjection.hpp>

// numopt
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>


namespace whole_body_control_romo {

template<typename ConcreteDescription_, typename RobotState_>
WholeBodyController<ConcreteDescription_, RobotState_>::WholeBodyController(
    loco::WholeBody& wholeBody,
    const RobotModel& model,
    loco::TerrainModelBase& terrainModel)
    : Base(wholeBody),
      model_(model),
      terrainModel_(terrainModel),
      hierarchicalOptimization_(nullptr),
      ddqDes_(),
      tauDes_(),
      lambdaDes_(),
      contactFlags_(),
      supportJacobian_(model, CoordinateFrameEnum::BASE, false),
      optimizationMethod_(),
      dt_(0.0),
      startRampingDownAtStancePhase_(0.96, 0.00, 1.00),
      endRampingUpAtStancePhase_(0.04, 0.00, 1.00),
      timer_(),
      timeSinceSupport_(),
      legLoad_(),
      adaptLegLoad_(false),
      optimalSolution_(),
      optimizationFailureCount_(0),
      didFail_(false),
      limbSupportControlMode_(loco::ControlMode::MODE_FREEZE),
      limbNonSupportControlMode_(loco::ControlMode::MODE_FREEZE),
      solutionSpaceDimension_(0),
      computeJointVelocities_(false),
      wholeBodyState_(model_, wholeBody, supportJacobian_, terrainModel_, contactFlags_),
      taskManager_(std_utils::make_unique<TaskLoader>(wholeBodyState_))
{
  TaskLoader::registerTask(BaseTranslationTaskType::getTaskTypeName(), &BaseTranslationTaskType::createTask);
  TaskLoader::registerTask(BaseRotationTaskType::getTaskTypeName(), &BaseRotationTaskType::createTask);
  TaskLoader::registerTask(BodyTranslationTaskType::getTaskTypeName(), &BodyTranslationTaskType::createTask);
  TaskLoader::registerTask(BodyRotationTaskType::getTaskTypeName(), &BodyRotationTaskType::createTask);
  TaskLoader::registerTask(ContactTaskType::getTaskTypeName(), &ContactTaskType::createTask);
  TaskLoader::registerTask(DistributeAccelerationsTaskType::getTaskTypeName(), &DistributeAccelerationsTaskType::createTask);
  TaskLoader::registerTask(DistributeContactForcesTaskType::getTaskTypeName(), &DistributeContactForcesTaskType::createTask);
  TaskLoader::registerTask(EndEffectorTranslationTaskType::getTaskTypeName(), &EndEffectorTranslationTaskType::createTask);
  TaskLoader::registerTask(EndEffectorRotationTaskType::getTaskTypeName(), &EndEffectorRotationTaskType::createTask);
  TaskLoader::registerTask(EoMTaskType::getTaskTypeName(), &EoMTaskType::createTask);
  TaskLoader::registerTask(FrictionPyramidTaskType::getTaskTypeName(), &FrictionPyramidTaskType::createTask);
  TaskLoader::registerTask(JointAccelerationTaskType::getTaskTypeName(), &JointAccelerationTaskType::createTask);
  TaskLoader::registerTask(JointFreezeTaskType::getTaskTypeName(), &JointFreezeTaskType::createTask);
  TaskLoader::registerTask(JointPositionTaskType::getTaskTypeName(), &JointPositionTaskType::createTask);
  TaskLoader::registerTask(JointPositionLimitsTaskType::getTaskTypeName(), &JointPositionLimitsTaskType::createTask);
  TaskLoader::registerTask(JointTorqueLimitsTaskType::getTaskTypeName(), &JointTorqueLimitsTaskType::createTask);
  TaskLoader::registerTask(JointVelocityTaskType::getTaskTypeName(), &JointVelocityTaskType::createTask);
  TaskLoader::registerTask(JointVelocityLimitsTaskType::getTaskTypeName(), &JointVelocityLimitsTaskType::createTask);
  TaskLoader::registerTask(ContactWrenchTaskType::getTaskTypeName(), &ContactWrenchTaskType::createTask);
  TaskLoader::registerTask(JointPositionVelocityTaskType::getTaskTypeName(), &JointPositionVelocityTaskType::createTask);

  ddqDes_.setZero();
  tauDes_.setZero();
  lambdaDes_.setZero();

  timeSinceSupport_.fill(10.0);
  legLoad_.fill(1.0);

  timer_ = std_utils::HighResolutionClockTimer("[WholeBodyController]");
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::loadParameters(const TiXmlHandle& handle) {
  // Load the parameter handles.
  TiXmlHandle motionControlHandle = handle;
  if (!tinyxml_tools::getChildHandle(motionControlHandle, handle, "MotionController")) { return false; }

  TiXmlHandle defaultControlModeHandle = handle;
  if (!tinyxml_tools::getChildHandle(defaultControlModeHandle, motionControlHandle, "DefaultControlModeForLeg")) { return false; }

  TiXmlHandle wbcHandle = handle;
  if (!tinyxml_tools::getChildHandle(wbcHandle, handle, "WholeBodyControl")) { return false; }

  TiXmlHandle configurationHandle = wbcHandle;
  if (!tinyxml_tools::getChildHandle(configurationHandle, wbcHandle, "Configuration")) { return false; }

  // Helper string vector.
  const std::vector<std::string> vectorParameterNames = {"x", "y", "z"};

  // Read the desired default control mode for support and non-support mode.
  std::string supportControlMode;
  std::string nonSupportControlMode;
  if (!tinyxml_tools::loadParameter(supportControlMode, defaultControlModeHandle, "supportMode")) { return false; }
  if (!tinyxml_tools::loadParameter(nonSupportControlMode, defaultControlModeHandle, "nonSupportMode")) { return false; }
  limbSupportControlMode_ = loco::topology_conversions::getControlModeEnumFromControlModeString(supportControlMode);
  limbNonSupportControlMode_ = loco::topology_conversions::getControlModeEnumFromControlModeString(nonSupportControlMode);

  std::string optMethod;
  if (!tinyxml_tools::loadParameter(optMethod, configurationHandle, "optimizationMethod")) { return false; }

  std::string qpSolver;
  if (!tinyxml_tools::loadParameter(qpSolver, configurationHandle, "qpSolver")) { return false; }

  std::string qpCascadeAlgorithm;
  if (!tinyxml_tools::loadParameter(qpCascadeAlgorithm, configurationHandle, "cascadeAlgorithm")) { return false; }

  if (!tinyxml_tools::loadParameter(adaptLegLoad_, configurationHandle, "adaptLegLoad")) { return false; }

  bool useThreads = false;
  if (!tinyxml_tools::loadParameter(useThreads, configurationHandle, "useThreads", false)) { return false; }

  double zeroThreshold = 0.0;
  if (!tinyxml_tools::loadParameter(zeroThreshold, configurationHandle, "zeroThreshold", 0.0)) { return false; }

  double regularizer = 0.0;
  tinyxml_tools::loadParameter(regularizer, configurationHandle, "regularizer", 1e-10);

  if (!tinyxml_tools::loadParameter(computeJointVelocities_, configurationHandle, "computeJointVelocities")) { return false; }

  if (optMethod == "ns") {
    MELO_DEBUG_STREAM(message_logger::color::magenta << "[WholeBodyControl::loadParameters]" <<
                     message_logger::color::blue << " Using the" <<
                     message_logger::color::cyan << " null space projection" <<
                     message_logger::color::blue << " method to solve the task prioritization.")
    optimizationMethod_ = OptimizationMethod::NullspaceProjection;
    hierarchicalOptimization_.reset(new hopt::HierarchicalOptimizationNullSpaceProjection());
  } else if (optMethod == "qp") {
    // Formulate the wbc problem using QPs.
    optimizationMethod_ = OptimizationMethod::QPCascade;
    std::string msg = message_logger::color::magenta + "[WholeBodyControl::loadParameters]" +
                      message_logger::color::blue    + " Using the" +
                      message_logger::color::cyan    + " QP cascade" +
                      message_logger::color::blue    + " method to solve the task prioritization.";

    constexpr auto qpProblemDimension = RD::getNumDof() + RD::getJointsDimension();

    // Setup the numerical solver.
    std::unique_ptr<numopt_common::QuadraticProblemSolver> minimizer;
    if (qpSolver == "quadprog++") {
      minimizer.reset(new numopt_quadprog::ActiveSetFunctionMinimizer());
    } else {
      MELO_WARN_STREAM("[WholeBodyControl::loadParameters] Unknown QP solver '" << qpSolver << "'.")
      return false;
    }
    msg.append(" Using the " + message_logger::color::cyan + qpSolver + message_logger::color::blue + " QP solver.");

    // Setup the cascade type.
    if (qpCascadeAlgorithm == "cascade") {
      hierarchicalOptimization_.reset(new hopt::HierarchicalOptimizationQpCascade(qpProblemDimension, std::move(minimizer)));
      msg.append(" Using the" + message_logger::color::cyan + " qp cascade" + message_logger::color::blue + " formulation.");
    } else if (qpCascadeAlgorithm == "ns_cascade") {
      hierarchicalOptimization_.reset(new hopt::HierarchicalOptimizationQpNullSpaceProjection(qpProblemDimension, std::move(minimizer), useThreads, zeroThreshold, regularizer));
      msg.append(" Using the" + message_logger::color::cyan + " ns + qp cascade" + message_logger::color::blue + " formulation.");
    } else {
      MELO_WARN_STREAM("[WholeBodyControl::loadParameters] No valid cascade method was specified. Choose from 'cascade' or 'ns_cascade'.")
      return false;
    }

    msg.append(message_logger::color::def);
    MELO_DEBUG_STREAM(msg)

  } else {
    MELO_WARN_STREAM("[WholeBodyControl::loadParameters] No valid task prioritization solver was specified.")
    return false;
  }

  if (!taskManager_.loadTaskSetups(wbcHandle)) {
    MELO_ERROR_STREAM(message_logger::color::magenta << "[WholeBodyControl::loadParameters]"
                                                     << message_logger::color::red << " Could not load tasks for task manager!"
                                                     << message_logger::color::def)
    return false;
  }

  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::addParametersToHandler(const std::string & ns) {
  bool success = true;
  success &= parameter_handler::handler->addParam(ns + "/wbc/legLoadingRampDownAt", startRampingDownAtStancePhase_);
  success &= parameter_handler::handler->addParam(ns + "/wbc/legLoadingRampUpUntil", endRampingUpAtStancePhase_);
  return success;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::removeParametersFromHandler() {
  bool success = true;
  success &= parameter_handler::handler->removeParam(startRampingDownAtStancePhase_.getName());
  success &= parameter_handler::handler->removeParam(endRampingUpAtStancePhase_.getName());
  return success;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::addVariablesToLog(const std::string& ns) const {
  if (!wholeBodyState_.addVariablesToLog(ns)) {
    MELO_ERROR_STREAM(message_logger::color::magenta <<
                      "[WholeBodyControl::loadParameters]"
                      << message_logger::color::red << " Could not add variables and signals to logger for WholeBodyStateRomo!"
                      << message_logger::color::def)
        return false;
  }

  if (!taskManager_.addVariablesToLog(ns)) {
    MELO_ERROR_STREAM(message_logger::color::magenta <<
                      "[WholeBodyControl::loadParameters]"
                      << message_logger::color::red << " Could not add variables and signals to logger for task manager!"
                      << message_logger::color::def)
    return false;
  }

  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::initialize(double dt) {
  dt_ = dt;

  ddqDes_.setZero();
  tauDes_.setZero();
  lambdaDes_.setZero();

  contactFlags_.fill(ContactStateEnum::ContactOpen);
  supportJacobian_.setContactFlags(contactFlags_, true);

  std::fill(timeSinceSupport_.begin(), timeSinceSupport_.end(), 10.0);
  std::fill(legLoad_.begin(), legLoad_.end(), 1.0);

  startRampingDownAtStancePhase_.resetToDefault();
  endRampingUpAtStancePhase_.resetToDefault();

  optimalSolution_ = Eigen::VectorXd::Zero(RD::getNumDof());
  optimizationFailureCount_ = 0;

  return taskManager_.initialize(dt_);
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::addOptimizationTasks() {
  if (!taskManager_.addOptimizationTasks(dt_, solutionSpaceDimension_, hierarchicalOptimization_)) {
    MELO_WARN_STREAM("[WholeBodyController::addOptimizationTasks] Could not add all task optimizations!")
    return false;
  }
  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
void WholeBodyController<ConcreteDescription_, RobotState_>::updateContactFlags() {
  for (const auto limb : *wholeBody_.getLimbsPtr()) {
    const auto limbEnum = RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getId());
    const auto limbStrategy = limb->getLimbStrategy().getLimbStrategyEnum();

    if ((limbStrategy == loco::LimbStrategyEnum::Support)
     || (limbStrategy == loco::LimbStrategyEnum::ContactInvariant) ) {
     const auto numberOfContactConstraints = limb->getEndEffector().getProperties().getNumberOfContactConstraints();

      if (numberOfContactConstraints == 3) {
        contactFlags_[limbEnum] = ContactStateEnum::ContactClosed3Dof;
      } else if (numberOfContactConstraints == 6) {
        contactFlags_[limbEnum] = ContactStateEnum::ContactClosed6Dof;
      } else {
        MELO_FATAL_STREAM(
            "[WholeBodyController::updateContactFlags] Unhandled number of contact constraints for the "
            "end effector mounted on the limb named " << limb->getName()
             << ". The number of constraints specified was " << numberOfContactConstraints << ".")
      }
    } else {
      contactFlags_[limbEnum] = ContactStateEnum::ContactOpen;
    }
  }
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::advance(double dt) {
  // Store the control sample time.
  dt_ = dt;

  // Set contact flags.
  updateContactFlags();

  // Adapt the leg load.
  for (const auto leg : *wholeBody_.getLegsPtr()) {
    if (adaptLegLoad_) {
      computeLegLoad(*leg, startRampingDownAtStancePhase_.getValue(),
                     endRampingUpAtStancePhase_.getValue(), 0.01);
    }
  }

  // Update support jacobian.
  supportJacobian_.setContactFlags(contactFlags_, true);
  solutionSpaceDimension_ = RD::getNumDof() + supportJacobian_.getNumberOfTotalContactConstraints();

  hierarchicalOptimization_->setSolutionDimension(solutionSpaceDimension_);
  hierarchicalOptimization_->resetOptimization();
  optimalSolution_.setZero();

  wholeBodyState_.advance(dt);

  addOptimizationTasks();

  if (hierarchicalOptimization_->solveOptimization(optimalSolution_)) {
    optimizationFailureCount_ = 0;
    if (didFail_) {
      MELO_INFO_STREAM("[WholeBodyController::advance] Current optimization succeeded. Last one failed, failure count was reset.")
      didFail_ = false;
    }
    setOptimizedReferences(dt);
    setControlReferencesToLimbs(dt);
  } else {
    MELO_INFO_STREAM("[WholeBodyController::advance] Solve optimization returned false!")
    ++optimizationFailureCount_;
    MELO_INFO_STREAM("[WholeBodyController::advance] Failure count: " << optimizationFailureCount_)
    didFail_ = true;
  }

  constexpr unsigned int maxFailures = 5;
  if (optimizationFailureCount_ >= maxFailures) {
    MELO_INFO_THROTTLE_STREAM(1.0, "[WholeBodyController::advance] Optimization failed for " << maxFailures << " times! Returning false.")
    return false;
  }

  setControlModeForLimbs();

  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::setOptimizedReferences(const double /*dt*/) {
  ddqDes_ = optimalSolution_.head<RD::getNumDof()>();
  lambdaDes_ = optimalSolution_.tail(supportJacobian_.getNumberOfTotalContactConstraints());
  constexpr auto jointsDimension = RD::getJointsDimension();
  tauDes_ = wholeBody_.getWholeBodyMassMatrix().template bottomRows<jointsDimension>()*ddqDes_
          + wholeBody_.getWholeBodyNonlinearEffects().template bottomRows<jointsDimension>()
          - supportJacobian_.getSupportJacobianTransposeInForceFrame().template bottomRows<jointsDimension>()*lambdaDes_;
  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::setControlReferencesToLimbs(const double dt) {
  // Set control signals in leg containers.
  unsigned int j = 0;
  for (const auto limbKey : RD::getLimbKeys()) {
    const auto limbEnum = limbKey.getEnum();
    const auto limbId = limbKey.getId();
    const auto limbIdJ = RD::getLimbStartIndexInJ(limbEnum);

    auto limb = wholeBody_.getLimbsPtr()->getPtr(limbId);
    const auto numDofLimb = RD::getNumDofLimb(limbEnum);

    // Set joint torques.
    limb->getLimbStateDesiredPtr()->setJointTorques(loco::JointTorques(tauDes_.segment(limbIdJ, numDofLimb)));

    // Set joint velocities.
    if (computeJointVelocities_) {
      if (contactFlags_[limbEnum] != ContactStateEnum::ContactOpen) {
        limb->getLimbStateDesiredPtr()->setJointVelocities(
            limb->getLimbStateMeasured().getJointVelocities()
                + loco::JointVelocities(dt * ddqDes_.segment(RD::getBaseGeneralizedVelocitiesDimension() + limbIdJ, numDofLimb)));
      } else {
        limb->getLimbStateDesiredPtr()->setJointVelocities(
            limb->getLimbStateDesired().getJointVelocities()
                + loco::JointVelocities(dt * ddqDes_.segment(RD::getBaseGeneralizedVelocitiesDimension() + limbIdJ, numDofLimb)));
      }
    }

    // Set contact forces.
    if (contactFlags_[limbEnum] != ContactStateEnum::ContactOpen) {
      const auto numContactConstraints = supportJacobian_.getNumberOfContactConstraintsFromContactEnum(contactFlags_[limbEnum]);
      if (supportJacobian_.getForceFrame() == CoordinateFrameEnum::WORLD) {
        limb->getEndEffectorPtr()->getStateDesiredPtr()->setForceAtEndEffectorInWorldFrame(
            loco::Force(lambdaDes_.segment<3>(j)));
      } else {
        limb->getEndEffectorPtr()->getStateDesiredPtr()->setForceAtEndEffectorInWorldFrame(
            model_.getState().getOrientationBaseToWorld().rotate(loco::Force(lambdaDes_.segment<3>(j))));
      }
      j += numContactConstraints;
    } else {
      limb->getEndEffectorPtr()->getStateDesiredPtr()->setForceAtEndEffectorInWorldFrame(romo::Force::Zero());
    }
  }

  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
void WholeBodyController<ConcreteDescription_, RobotState_>::computeLegLoad(
    loco::LegBase& leg, const double startRampingDownAtStancePhase,
    const double endRampingUpAtStancePhase, const double minLegLoad) {
  const auto legId = leg.getId();
  const auto limbEnum = RD::template mapKeyIdToKeyEnum<LimbEnum>(legId);

  if (contactFlags_[limbEnum] != ContactStateEnum::ContactOpen) {
    timeSinceSupport_[limbEnum] += dt_;
    if (timeSinceSupport_[limbEnum] <= endRampingUpAtStancePhase * leg.getContactSchedule().getStanceDuration()) {
      // ramping up - leg has just landed
      legLoad_[limbEnum] = minLegLoad + (1.0 - minLegLoad)
                         * robot_utils::mapTo01Range(timeSinceSupport_[limbEnum], 0.0,
                                                     endRampingUpAtStancePhase*leg.getContactSchedule().getStanceDuration());
    } else if (leg.getContactSchedule().getStancePhase() >= startRampingDownAtStancePhase) {
      // ramping down - leg is going to take off
      legLoad_[limbEnum] = minLegLoad + (1.0 - minLegLoad) * (1.0 - robot_utils::mapTo01Range(leg.getContactSchedule().getStancePhase(), startRampingDownAtStancePhase, 1.0));
    } else {
      // mid stance phase - reset max force to the default value
      legLoad_[limbEnum] = 1.0;
    }
  } else {
    // leg is not supporting the main body
    timeSinceSupport_[limbEnum] = 0.0;
    legLoad_[limbEnum] = minLegLoad;
  }

  leg.setLoadFactor(legLoad_[limbEnum]);
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::setToInterpolated(
    const loco::MotionControllerBase& motionController1,
    const loco::MotionControllerBase& motionController2, double /*t*/) {
  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::setControlModeForLimbs() {
  for (auto limb : *wholeBody_.getLimbsPtr()) {
    setControlModeForLimb(limb, limbSupportControlMode_, limbNonSupportControlMode_);
  }
  return true;
}

template<typename ConcreteDescription_, typename RobotState_>
bool WholeBodyController<ConcreteDescription_, RobotState_>::switchTaskSetup(const std::string& newSetupName){
  if (!taskManager_.switchTaskSetup(newSetupName)) {
    MELO_WARN_STREAM("Could not switch task setup to " << newSetupName << "!")
    return false;
  }
  // Initialize the all the tasks in the current task setup.
  return taskManager_.initialize(dt_);
}

} /* namespace whole_body_control_romo */
