/*
 * ZmpOptimizer.cpp
 *
 *  Created on: Nov 25, 2014
 *      Author: C. Dario Bellicoso
 */

// messagge logger
#include "message_logger/message_logger.hpp"

// numerical optimization
#include <numopt_common/ParameterizationIdentity.hpp>

// robot utils
#include "robot_utils/math/math.hpp"

// loco
#include <zmp_optimizer/zmp_optimizer.hpp>
#include "zmp_optimizer/MotionPlan.hpp"
#include "zmp_optimizer/ZmpOptimizerDynamicWalk.hpp"

#define EIGEN_INITIALIZE_MATRICES_BY_NAN

using namespace message_logger::color;

namespace loco {

ZmpOptimizerDynamicWalk::ZmpOptimizerDynamicWalk()
    : ZmpOptimizerBase(),
      optimizedSolution_(),
      objectiveHandler_(),
      optimizationDofs_(),
      isNonlinearOptimization_(false),
      zmpOptimizationProblemQP_(nullptr),
      zmpOptimizationProblem_(nullptr),
      qpSolver_(nullptr),
      sqpSolver_(nullptr),
      lineSearchOpts_(),
      maxIter_(10u),
      verbose_(false),
      hessianRegularizer_(1e-7),
      enableInequalityConstraints_(false),
      finalMaxStateBox_(0.05),
      delta_(0.2),
      lineToRectangleOffset_(0.001),
      pointToRectangleOffset_(0.01),
      numOfVertecesPerCircleConstraint_(8u),
      motionPlan_(nullptr),
      useConstraintHessian_(false),
      numOfPreviousCoeffs_(0u),
      numOfCurrentCoeffs_(0u),
      zmpInfo_(),
      numOfSamplesPerSpline_(6u),
      expZAlpha_(4.0),
      expZLambda_(30.0) {}

bool ZmpOptimizerDynamicWalk::initialize(double wholeBodyMass, const Eigen::Matrix3d& torsoInertiaTensorInBaseFrame) {
  if (!objectiveHandler_.initialize(wholeBodyMass, torsoInertiaTensorInBaseFrame)) {
    return false;
  }
  if (!initializeSolvers()) {
    return false;
  }
  optimizedSolution_.resize(0);
  numOfPreviousCoeffs_ = 0u;
  numOfCurrentCoeffs_ = 0u;
  optimizationDofs_.clear();
  return true;
}

void ZmpOptimizerDynamicWalk::stop() {
  if (qpSolver_ != nullptr) {
    qpSolver_->stop();
  }
  if (sqpSolver_ != nullptr) {
    sqpSolver_->stop();
  }
}

bool ZmpOptimizerDynamicWalk::initialize() {
  return false;
}

bool ZmpOptimizerDynamicWalk::initializeSolvers() {
  // Initialize QP solver.
  hessianRegularizer_ = 1.0e-7;
  qpSolver_.reset(new numopt_quadprog::ActiveSetFunctionMinimizer(200, true));

  // Initialize zmp optimization problems.
  zmpOptimizationProblemQP_.reset(new zmp::ZmpOptimizationProblemQP(objectiveHandler_, hessianRegularizer_, finalMaxStateBox_,
                                                                    enableInequalityConstraints_, useConstraintHessian_));

  zmpOptimizationProblem_.reset(new zmp::ZmpOptimizationProblem(objectiveHandler_, hessianRegularizer_, finalMaxStateBox_,
                                                                enableInequalityConstraints_, useConstraintHessian_));

  // Initialize callbacks.
  std::function<void(const numopt_common::Parameterization&)> optimizationStepInitCallback =
      std::bind(&zmp::ZmpOptimizationProblem::registerOptimizationStepInitCallback, zmpOptimizationProblem_.get(), std::placeholders::_1);

  // Initialize NP solver.
  sqpSolver_.reset(new numopt_sqp::SqpFunctionMinimizerLineSearch(qpSolver_, static_cast<int>(maxIter_), 0.005, -DBL_MAX, verbose_, false));
  sqpSolver_->setLineSearchOptions(lineSearchOpts_);
  sqpSolver_->registerOptimizationStepInitCallback(optimizationStepInitCallback);

  if (useConstraintHessian_) {
    MELO_WARN_STREAM(
        "[ZmpOptimizerDynamicWalk::initializeSolvers] We provide a Hessian of the constraints. However, the SQP will not incorporate it!")
  }

  return true;
}

bool ZmpOptimizerDynamicWalk::loadParameters(const TiXmlHandle& handle) {
  MELO_DEBUG_STREAM(magenta << "[ZmpOptimizerDynamicWalk] " << blue << "Load parameters." << def)

  TiXmlHandle zmpHandle = handle;
  if (!tinyxml_tools::getChildHandle(zmpHandle, handle, "ZmpOptimizer")) {
    return false;
  }
  TiXmlHandle comSupportHandle = handle;
  if (!tinyxml_tools::getChildHandle(comSupportHandle, handle, "ComSupportControl")) {
    return false;
  }

  // Solver.
  TiXmlHandle solverHandle = handle;
  if (!tinyxml_tools::getChildHandle(solverHandle, zmpHandle, "Solver")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(verbose_, solverHandle, "verbose")) {
    return false;
  }

  // QP solver.
  TiXmlHandle qpHandle = handle;
  if (!tinyxml_tools::getChildHandle(qpHandle, solverHandle, "QPSolver")) {
    return false;
  }
  std::string qpMethodStr = "as";
  if (!tinyxml_tools::loadParameter(qpMethodStr, qpHandle, "method")) {
    return false;
  }

  // Nonlinear solver.
  TiXmlHandle nlsolverHandle = handle;
  if (!tinyxml_tools::getChildHandle(nlsolverHandle, solverHandle, "NonLinSolver")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(maxIter_, nlsolverHandle, "max_iter")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(useConstraintHessian_, nlsolverHandle, "use_hessian_of_constraints")) {
    return false;
  }

  // Line Search.
  TiXmlHandle lineSearchHandle = handle;
  if (!tinyxml_tools::getChildHandle(lineSearchHandle, solverHandle, "SQPLineSearch")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(lineSearchOpts_.maxLineSearchIterations_, lineSearchHandle, "max_iter_back_tracing")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(lineSearchOpts_.alpha_, lineSearchHandle, "alpha")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(lineSearchOpts_.beta_, lineSearchHandle, "beta")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(lineSearchOpts_.muEqConstraints_, lineSearchHandle, "w_eq")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(lineSearchOpts_.muIneqConstraints_, lineSearchHandle, "w_ineq")) {
    return false;
  }
  lineSearchOpts_.directionalDerivativeResidual_ = 1e-7;
  lineSearchOpts_.adaptEqualityWeight_ = false;

  // Z spline adjustment.
  TiXmlHandle zSplineHandle = handle;
  if (!tinyxml_tools::getChildHandle(zSplineHandle, zmpHandle, "ZSpline")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(expZAlpha_, zSplineHandle, "exp_alpha")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(expZLambda_, zSplineHandle, "exp_lambda")) {
    return false;
  }

  // Max final state for translation.
  TiXmlHandle ineqConstraintHandle = handle;
  if (!tinyxml_tools::getChildHandle(ineqConstraintHandle, zmpHandle, "InequalityConstraints")) {
    return false;
  }
  bool enableInequalityConstraint;

  TiXmlHandle finalBoxHandle = handle;
  if (!tinyxml_tools::getChildHandle(finalBoxHandle, ineqConstraintHandle, "FinalCogBox")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(enableInequalityConstraint, finalBoxHandle, "enable")) {
    return false;
  }
  enableInequalityConstraints_[zmp::Ineq::FinalCogBox] = enableInequalityConstraint;
  double boxSize;
  if (!tinyxml_tools::loadParameter(boxSize, finalBoxHandle, "x_y_z")) {
    return false;
  }
  for (auto& dim : zmp::optimizationTranslationalDofs) {
    finalMaxStateBox_[dim] = boxSize;
  }
  if (!tinyxml_tools::loadParameter(boxSize, finalBoxHandle, "roll_pitch_yaw")) {
    return false;
  }
  for (auto& dim : zmp::optimizationRotationalDofs) {
    finalMaxStateBox_[dim] = boxSize;
  }

  // Max Force.
  TiXmlHandle forceHandle = handle;
  if (!tinyxml_tools::getChildHandle(forceHandle, ineqConstraintHandle, "ForceModel")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(enableInequalityConstraint, forceHandle, "enable")) {
    return false;
  }
  enableInequalityConstraints_[zmp::Ineq::ForceModel] = enableInequalityConstraint;

  // Support polygons.
  TiXmlHandle supportPolygonsHandle = handle;
  if (!tinyxml_tools::getChildHandle(supportPolygonsHandle, comSupportHandle, "SupportPolygons")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(delta_, supportPolygonsHandle, "safety_margin")) {
    return false;
  }
  robot_utils::boundToRange(&delta_, 0.0, 1.0);
  if (!tinyxml_tools::loadParameter(lineToRectangleOffset_, supportPolygonsHandle, "line_to_rectangle_offset")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(pointToRectangleOffset_, supportPolygonsHandle, "point_to_rectangle_offset")) {
    return false;
  }

  // Objective handler.
  return objectiveHandler_.loadParameters(zmpHandle);
}

bool ZmpOptimizerDynamicWalk::addToLogger() {
  return true;
}

bool ZmpOptimizerDynamicWalk::readMotionPlan(zmp::MotionPlan& motionPlan) {
  // Read new motion plan.
  motionPlan_ = &motionPlan;
  optimizationDofs_ = motionPlan_->getOptimizationDofs();

  // Update objective handler.
  objectiveHandler_.setOptimizationDofs(optimizationDofs_);
  const auto& virtualPlaneFrame = motionPlan_->getVirtualPlaneFrame();
  const Eigen::Vector3d gravityVectorInWorldFrame =
      Eigen::Vector3d(0.0, 0.0, -robot_utils::physical_definitions::getAbsoluteGravityAcceleration());
  const Eigen::Vector3d gravityVectorInPlaneFrame =
      virtualPlaneFrame.getPosePlaneToWorld().getRotation().inverseRotate(gravityVectorInWorldFrame);
  objectiveHandler_.setGravityAndPlaneNormalInPlaneFrame(virtualPlaneFrame.getPlaneNormalInPlaneFrame().toImplementation(),
                                                         gravityVectorInPlaneFrame);

  // Post process support polygons.
  for (auto& supportPolygon : *motionPlan_->getSupportPolygonsInPlaneFramePtr()) {
    if (!supportPolygon.getPolygonPtr()->reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise)) {
      MELO_WARN("Could not re-order support polygon vertices in counter clockwise direction!")
      return false;
    }
    if (!supportPolygon.getPolygonPtr()->resizeAreaTriangleBased(delta_)) {
      MELO_WARN("Could not resize support polygon!")
      return false;
    }
    if (supportPolygon.getPolygonType() == zmp::PolygonType::Point || supportPolygon.getPolygonType() == zmp::PolygonType::Line) {
      if (!supportPolygon.convertToRectangle(lineToRectangleOffset_, pointToRectangleOffset_, numOfVertecesPerCircleConstraint_)) {
        MELO_WARN_STREAM("Could not convert polygon pf type " << zmp::PolygonTypeNamesMap[supportPolygon.getPolygonType()]
                                                              << " to a rectangle!")
        return false;
      }
      if (!supportPolygon.getPolygonPtr()->reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise)) {
        MELO_WARN_STREAM(
            "Could not reorder support polygon vertices; Polygon type: " << zmp::PolygonTypeNamesMap[supportPolygon.getPolygonType()])
        return false;
      }
    }
    if (!supportPolygon.getPolygonPtr()->updateLineCoefficients()) {
      MELO_WARN("Could not update support polygon line coefficients!")
      return false;
    }

    // Check if we have a gait with full flight phases but optimize only for a 2D trajectory.
    if (!std_utils::containsEnum(optimizationDofs_, zmp::CogDim::z) && supportPolygon.getPolygonType() == zmp::PolygonType::Empty) {
      MELO_WARN_STREAM(
          "[ZmpOptimizerDynamicWalk::readMotionPlan] Gait contains full flight phases but optimization does does not include height.")
      return false;
    }
  }

  // Set up final box constraints (for visualization only).
  motionPlan_->setComFinalBox(zmp::Box(finalMaxStateBox_, motionPlan_->getPlaneToFinalPositionInPlaneFrame()));

  return true;
}

bool ZmpOptimizerDynamicWalk::setupConstraintProperties() {
  if (motionPlan_->getSupportPolygonsInPlaneFrame().empty()) {
    MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::setupConstraintProperties] Support polygon size was zero!")
    return false;
  }

  // Reset optimization variables.
  zmpInfo_.reset();

  // Skip initial acceleration conditions.
  zmpInfo_.skipInitialAccelConditionsHard_ = motionPlan_->getSkipHardInitialAccelConstraints();

  // If we optimize only for x,y components, we use a planer zmp model, otherwise a full state zmp model.
  isNonlinearOptimization_ = !std_utils::consistsOfEnums(optimizationDofs_, zmp::optimizationXYDofs);

  // Set the solution space dimension.
  numOfPreviousCoeffs_ = numOfCurrentCoeffs_;
  numOfCurrentCoeffs_ = optimizationDofs_.size() * zmp::splineCoeffs * zmpInfo_.splineInfoSequence_.size();
  zmpInfo_.numOfUnknowns_ += numOfCurrentCoeffs_;

  /*******************************
   * Linear Equality Constraints *
   *******************************/
  const unsigned int numBoundConstr =
      optimizationDofs_.size() * static_cast<unsigned int>(zmp::Derivative::SIZE);  // ([x,y,z,...] times [p v a])
  const unsigned int numBoundConstrNoAccel =
      optimizationDofs_.size() * (static_cast<unsigned int>(zmp::Derivative::SIZE) - 1u);  // ([x,y,z,...] times [p v])

  // Initial conditions.
  if (zmpInfo_.skipInitialAccelConditionsHard_) {
    zmpInfo_.numOfEqualityConstraintsQP_ += numBoundConstrNoAccel;
  } else {
    zmpInfo_.numOfEqualityConstraintsQP_ += numBoundConstr;
  }

  // Final conditions (Note: We do not set final acceleration conditions).
  if (motionPlan_->getEnforceHardFinalConstraints()) {
    zmpInfo_.numOfEqualityConstraintsQP_ += numBoundConstrNoAccel;
  }

  // Junction Constraints.
  for (auto splineId = 0u; splineId + 1u < zmpInfo_.splineInfoSequence_.size(); ++splineId) {
    if (zmpInfo_.splineInfoSequence_[splineId].skipJunctionAccelAtNextPhase_) {
      zmpInfo_.numOfEqualityConstraintsQP_ += numBoundConstrNoAccel;
    } else {
      zmpInfo_.numOfEqualityConstraintsQP_ += numBoundConstr;
    }
  }
  // For full flight phases we have one additional accel constraint at a junction.
  auto numOfFullFlightPhases = 0u;
  for (auto polygonId = 1u; polygonId + 1u < motionPlan_->getSupportPolygonsInPlaneFrame().size(); ++polygonId) {
    if (motionPlan_->getSupportPolygonsInPlaneFrame()[polygonId].getPolygonType() == zmp::PolygonType::Empty) {
      numOfFullFlightPhases += 1u;
    }
  }
  zmpInfo_.numOfEqualityConstraintsQP_ += numOfFullFlightPhases * optimizationDofs_.size();

  /*******************************/

  /**************************
   * Inequality Constraints *
   **************************/
  auto numSamplePoints = 0u;
  auto numSamplePointsInActiveGait = 0u;
  auto zmpInequalityConstraints = 0u;
  double timeAtSplineStart = 0.0;
  auto numZmpRelaxationSamples = 0u;
  bool skipInitialSample = true;
  const double zmpRelaxationHorizon = motionPlan_->getOptimizationHorizon() * 0.5 *
                                      (motionPlan_->getZmpParams().getActiveParams().zmpRelativeRelaxationHorizon_ +
                                       motionPlan_->getZmpParams().getDesiredParams().zmpRelativeRelaxationHorizon_);

  zmpInfo_.timeInstantsPerSplineId_.resize(zmpInfo_.splineInfoSequence_.size());
  for (const auto& splineInfo : zmpInfo_.splineInfoSequence_) {
    const double samplingTime = std::fmax(splineInfo.duration_ / static_cast<double>(numOfSamplesPerSpline_), 1.0e-5);
    const double finalSplineTime = splineInfo.duration_ - 1.0e-5;

    // Skip very first sample (and first sample of each polygon that follows after a full flight phase).
    const double initialSplineTime = (skipInitialSample ? samplingTime : 1.0e-5);

    const double splineDuration = finalSplineTime - initialSplineTime;
    if (splineDuration <= 0.0) {
      MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::setupConstraintProperties] Negative spline duration!")
      continue;
    }

    // Add sample point.
    zmpInfo_.timeInstantsPerSplineId_[splineInfo.id_].clear();
    zmpInfo_.timeInstantsPerSplineId_[splineInfo.id_].reserve(numOfSamplesPerSpline_);
    for (double tk = initialSplineTime; tk < finalSplineTime; tk += samplingTime) {
      if (numZmpRelaxationSamples == 0u && zmpRelaxationHorizon > 0.0 && (timeAtSplineStart + tk) >= zmpRelaxationHorizon) {
        numZmpRelaxationSamples = numSamplePoints;
      }

      zmpInfo_.timeInstantsPerSplineId_[splineInfo.id_].push_back(tk);
      ++numSamplePoints;
    }

    // Add final sample.
    zmpInfo_.timeInstantsPerSplineId_[splineInfo.id_].push_back(finalSplineTime);
    ++numSamplePoints;

    if (motionPlan_->getZmpParams().isSplineIdInActiveGait(splineInfo.id_)) {
      numSamplePointsInActiveGait = numSamplePoints;
    }

    // Add zmp constraints.
    zmpInequalityConstraints +=
        zmpInfo_.timeInstantsPerSplineId_[splineInfo.id_].size() * splineInfo.supportPolygon.getPolygon().getLineCoefficients().size();

    timeAtSplineStart += splineInfo.duration_;
    skipInitialSample = (splineInfo.flightDurationOfNextPhase_ > 0.0);
  }

  // If relaxation horizon > optimization horizon, the above sampling will not work.
  if (numZmpRelaxationSamples == 0u && zmpRelaxationHorizon > 0.0) {
    numZmpRelaxationSamples = numSamplePoints;
  }

  // Number of samples at which we soften ZMP constraints.
  // (Note: We add one slack variable per sample time, not per zmp-constraint!)
  zmpInfo_.numZmpEspilonStates_ = numZmpRelaxationSamples;
  zmpInfo_.numOfUnknowns_ += numZmpRelaxationSamples;

  // Final inequality constraint for CoG position: 2 constraints (upper and lower bound) for each 2d box
  if (!motionPlan_->getEnforceHardFinalConstraints() && enableInequalityConstraints_[zmp::Ineq::FinalCogBox]) {
    zmpInfo_.numOfInequalityConstraintsQP_ += 2u * optimizationDofs_.size();
  }

  if (isNonlinearOptimization_) {
    zmpInfo_.numOfInequalityConstraintsSQP_ += zmpInequalityConstraints;  // nonlinear zmp constraints
  } else {
    zmpInfo_.numOfInequalityConstraintsQP_ += zmpInequalityConstraints;  // linear zmp constraints
  }

  if (std_utils::containsEnums(optimizationDofs_, zmp::optimizationTranslationalDofs)) {
    zmpInfo_.numOfInequalityConstraintsQP_ += numSamplePoints;  // 1 push constraint
    if (enableInequalityConstraints_[zmp::Ineq::ForceModel]) {
      zmpInfo_.numOfInequalityConstraintsQP_ += numSamplePoints;         // 1 max normal contact force constraint
      zmpInfo_.numOfInequalityConstraintsQP_ += (4u * numSamplePoints);  // friction pyramid constraints
    }
  }

  // Add additional states for minimizing max deviation w.r.t to path regularizer.
  motionPlan_->getZmpParamsPtr()->resetAddEpsilonStateMinDeviation();
  unsigned int numOfMinDeviationConstraints = 0u;
  for (const auto& dim : optimizationDofs_) {
    // Add epsilon states for active gait.
    if (motionPlan_->getZmpParams().getSplineIdAtSwitch() != 0) {
      if (motionPlan_->getZmpParams().getActiveParams().weightsMinDeviation_[dim][zmp::Objective::Lin] > 0.0 ||
          motionPlan_->getZmpParams().getActiveParams().weightsMinDeviation_[dim][zmp::Objective::Quad] > 0.0) {
        motionPlan_->getZmpParamsPtr()->addEpsilonStateMinDeviationActiveGait(dim, true);
        numOfMinDeviationConstraints += numSamplePointsInActiveGait;
      }
    }

    // Add epsilon states for desired gait.
    if (motionPlan_->getZmpParams().getSplineIdAtSwitch() >= 0) {
      if (motionPlan_->getZmpParams().getDesiredParams().weightsMinDeviation_[dim][zmp::Objective::Lin] > 0.0 ||
          motionPlan_->getZmpParams().getDesiredParams().weightsMinDeviation_[dim][zmp::Objective::Quad] > 0.0) {
        motionPlan_->getZmpParamsPtr()->addEpsilonStateMinDeviationDesiredGait(dim, true);
        numOfMinDeviationConstraints += (numSamplePoints - numSamplePointsInActiveGait);
      }
    }
  }

  if (numOfMinDeviationConstraints > 0u) {
    zmpInfo_.numDeviationEpsilonStates_ = (motionPlan_->getZmpParams().getNumOfEpsilonStateMinDeviationActiveGait() +
                                           motionPlan_->getZmpParams().getNumOfEpsilonStateMinDeviationDesiredGait());
    zmpInfo_.numOfUnknowns_ += zmpInfo_.numDeviationEpsilonStates_;
    zmpInfo_.numOfInequalityConstraintsQP_ += 2u * numOfMinDeviationConstraints;  // upper and lower bound
  }

  /**************************/

  return initializeSplineCoefficients();
}

bool ZmpOptimizerDynamicWalk::initializeSplineCoefficients() {
  /*
   * The number of SQP iteration depends heavily on the initial guess. If we can, we take the previous
   * optimized spline vector as initial guess for the current solution. If not, we construtct a spline vector
   * that satisfied all equality constraints and some inequality constraints (however, the zmp inequaluty constraints
   * are not satisfied).
   */

  // Check if we can warm-start the solver.
  const bool isExternalWarmStartedSolver = ((isNonlinearOptimization_ && sqpSolver_->isExternalWarmStarted()) ||
                                            (!isNonlinearOptimization_ && qpSolver_->isExternalWarmStarted()));

  if (!isExternalWarmStartedSolver) {
    optimizedSolution_.setZero(zmpInfo_.numOfUnknowns_);
    return true;
  }

  // Check if we can use the previous solution.
  if (!motionPlan_->isFirstSupportPolygonNew()) {
    if (numOfPreviousCoeffs_ == numOfCurrentCoeffs_ && numOfPreviousCoeffs_ > 0u) {
      optimizedSolution_.conservativeResize(zmpInfo_.numOfUnknowns_);
      optimizedSolution_.tail(zmpInfo_.numDeviationEpsilonStates_ + zmpInfo_.numZmpEspilonStates_).setZero();
      return true;
    }
  }

  /*
   * Whenever we cannot use the previous solution -> Find approximate trajectory connecting
   * initial and final point s.t. all equality constraints are satisfied.
   */
  optimizedSolution_.setZero(zmpInfo_.numOfUnknowns_);
  std::vector<double> positionKnot(zmpInfo_.splineInfoSequence_.size() + 1u);
  std::vector<double> timeKnot(zmpInfo_.splineInfoSequence_.size() + 1u);
  curves::PolynomialSplineContainerQuintic initialTrajectory;

  for (const auto& dim : optimizationDofs_) {
    if (zmp::isTranslation(dim)) {
      // Set first and last knot point.
      positionKnot.front() = motionPlan_->getInitialRobotStateInPlaneFrame()[zmp::Derivative::Zero][dim];
      positionKnot.back() = motionPlan_->getFinalRobotStateInPlaneFrame()[zmp::Derivative::Zero][dim];
      timeKnot.front() = 0.0;
      timeKnot.back() = motionPlan_->getOptimizationHorizon();

      if (timeKnot.back() <= 0.0) {
        MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::initializeSplineCoefficients] Optimization horizon is zero!")
        return false;
      }

      // Add intermediate knots (assuming constant velocity).
      for (unsigned int splineId = 0u; splineId + 1u < zmpInfo_.splineInfoSequence_.size(); ++splineId) {
        timeKnot[splineId + 1u] = timeKnot[splineId] + zmpInfo_.splineInfoSequence_[splineId].duration_;
        positionKnot[splineId + 1u] = robot_utils::linearlyInterpolate(positionKnot.front(), positionKnot.back(), timeKnot.front(),
                                                                       timeKnot.back(), timeKnot[splineId + 1u]);
      }

      // Establish trajectory
      if (!initialTrajectory.setData(timeKnot, positionKnot)) {
        MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::initializeSplineCoefficients] Failed to set initial trajectory!")
        return false;
      }

      // Convert to to Eigen-vector.
      for (unsigned int splineId = 0u; splineId < zmpInfo_.splineInfoSequence_.size(); ++splineId) {
        const unsigned int coeffStartId = objectiveHandler_.getCoeffStartIndex(splineId, dim);
        optimizedSolution_.segment(coeffStartId, zmp::splineCoeffs) = Eigen::Map<ZmpSplineCoeff>(
            initialTrajectory.getSpline(static_cast<int>(splineId))->getCoefficientsPtr()->data(), zmp::splineCoeffs, 1u);
      }

    }

    else if (zmp::isRotation(dim)) {
      // zero spline coeffs.
    }

    else {
      MELO_FATAL_STREAM("[ZmpOptimizerDynamicWalk::initializeSplineCoefficients] Unhandled dimension")
      return false;
    }
  }

  return true;
}

bool ZmpOptimizerDynamicWalk::createSplineInfoSequence() {
  int splineId = 0u;
  zmpInfo_.splineInfoSequence_.clear();
  motionPlan_->getZmpParamsPtr()->setSplineIdAtSwitch(-1);

  zmp::SplineInfo splineInfo;
  for (unsigned int polygonId = 0u; polygonId < getSizeOfSupportPolygonForSplineRepresentation(); ++polygonId) {
    const auto& supportPolygon = motionPlan_->getSupportPolygonsInPlaneFrame()[polygonId];
    const auto splinesPerPolygon =
        motionPlan_->getZmpParams().getParamsBySpline(splineId).splinesPerPolygon_[supportPolygon.getPolygonType()];

    // Skip full flight phases.
    if (supportPolygon.getPolygonType() == zmp::PolygonType::Empty) {
      continue;
    }

    // Convert polygon index to spline index.
    if (static_cast<int>(polygonId) >= motionPlan_->getZmpParams().getPolygonIdAtSwitch() &&
        motionPlan_->getZmpParams().getSplineIdAtSwitch() == -1 && motionPlan_->getZmpParams().getPolygonIdAtSwitch() != -1) {
      motionPlan_->getZmpParamsPtr()->setSplineIdAtSwitch(splineId);
    }

    // Determine if this polygon intersects the next one.
    bool skipJunctionAccelAtNextPhase = false;
    double flightDurationOfNextPolygon = -1.0;  // -1 means no flight phase

    if (polygonId < getSizeOfSupportPolygonForSplineRepresentation() - 1) {
      const auto& supportPolygonNext = motionPlan_->getSupportPolygonsInPlaneFrame()[polygonId + 1];

      // Remember if the next support polygon corresponds to a flight phase.
      if (supportPolygonNext.getPolygonType() == zmp::PolygonType::Empty) {
        flightDurationOfNextPolygon = supportPolygonNext.getDuration();
      }

      // Check if we should skip junction acceleration.
      const auto currentType = supportPolygon.getPolygonType();
      const auto nextType = supportPolygonNext.getPolygonType();
      const bool containsLine = (currentType == zmp::PolygonType::Line || nextType == zmp::PolygonType::Line);
      const bool containsPoint = (currentType == zmp::PolygonType::Point || nextType == zmp::PolygonType::Point);

      // Skip if we know that polygons do not intersect.
      if (flightDurationOfNextPolygon > 0.0) {
        skipJunctionAccelAtNextPhase = false;
      }

      else if (containsLine || containsPoint) {
        skipJunctionAccelAtNextPhase = true;
      }

      // Otherwise, check geometrically if the polygons intersect.
      else {
        skipJunctionAccelAtNextPhase =
            !robot_utils::geometry::Polygon::doPolygonsIntersect(supportPolygon.getPolygon(), supportPolygonNext.getPolygon());
      }
    }  // end if final polygon.

    // Add spline.
    splineInfo.skipJunctionAccelAtNextPhase_ = false;
    splineInfo.duration_ = supportPolygon.getDuration() / static_cast<double>(splinesPerPolygon);
    splineInfo.supportPolygon = supportPolygon;
    splineInfo.flightDurationOfNextPhase_ = -1.0;

    zmpInfo_.splineInfoSequence_.reserve(zmpInfo_.splineInfoSequence_.size() + splinesPerPolygon);
    for (unsigned int splineSegmentId = 0u; splineSegmentId < splinesPerPolygon; ++splineSegmentId) {
      if (splineSegmentId == splinesPerPolygon - 1u) {
        splineInfo.skipJunctionAccelAtNextPhase_ = skipJunctionAccelAtNextPhase;
        splineInfo.flightDurationOfNextPhase_ = flightDurationOfNextPolygon;
      }
      splineInfo.id_ = splineId;
      zmpInfo_.splineInfoSequence_.push_back(splineInfo);
      ++splineId;
    }

  }  // end for polygonId

  return true;
}

size_t ZmpOptimizerDynamicWalk::getSizeOfSupportPolygonForSplineRepresentation() {
  return motionPlan_->getSupportPolygonsInPlaneFrame().size();
}

bool ZmpOptimizerDynamicWalk::runOptimization() {
  double cost = 0.0;

  // Initial conditions for solver.
  numopt_common::ParameterizationIdentity params;
  params.getParams() = optimizedSolution_;

  bool didOptimizationSucceeded = true;
  if (!isNonlinearOptimization_) {
    if (!zmpOptimizationProblemQP_->initializeParameters(zmpInfo_, motionPlan_)) {
      return false;
    }
    if (verbose_) {
      timer_.pinTime();
    }
    didOptimizationSucceeded &= qpSolver_->minimize(zmpOptimizationProblemQP_.get(), params, cost, 0u);
    if (verbose_) {
      timer_.splitTime();
      MELO_INFO_THROTTLE_STREAM(0.5, std::string{"qp solver"} + timer_.asString())
    }
  }

  else {
    // Initialize zmp problem.
    if (!zmpOptimizationProblem_->initializeParameters(zmpInfo_, motionPlan_)) {
      return false;
    }
    if (verbose_) {
      timer_.pinTime();
    }
    didOptimizationSucceeded &= sqpSolver_->minimize(zmpOptimizationProblem_.get(), params, cost);
    if (verbose_) {
      timer_.splitTime();
      MELO_INFO_THROTTLE_STREAM(0.5, std::string{"qp solver"} + timer_.asString())
    }
  }

  if (didOptimizationSucceeded) {
    optimizedSolution_ = params.getParams();
  }
  return didOptimizationSucceeded;
}

bool ZmpOptimizerDynamicWalk::createSolutionSplines() {
  curves::PolynomialSplineQuintic::SplineCoefficients coeffsSegment;
  curves::PolynomialSplineQuintic splineSegment;
  curves::PolynomialSplineQuintic nextSplineSegment;
  curves::SplineOptions opts;

  for (const auto& dim : optimizationDofs_) {
    if (!motionPlan_->getComStateInPlaneFramePtr()->reset(dim)) {
      return false;
    }

    // Initialization.
    unsigned int startIndex = objectiveHandler_.getCoeffStartIndex(0u, dim);
    Eigen::Map<ZmpSplineCoeff>(coeffsSegment.data(), zmp::splineCoeffs, 1u) = optimizedSolution_.segment(startIndex, zmp::splineCoeffs);
    nextSplineSegment.setCoefficientsAndDuration(coeffsSegment, zmpInfo_.splineInfoSequence_[0u].duration_);

    /*
     * We start with a flight phase. Include a virtual flight trajectory that connects the
     * initial point with the first point of the solution trajectory.
     */
    if (motionPlan_->getSupportPolygonsInPlaneFrame().front().getPolygonType() == zmp::PolygonType::Empty) {
      opts.tf_ = motionPlan_->getSupportPolygonsInPlaneFrame().front().getDuration();
      opts.pos0_ = motionPlan_->getInitialRobotStateInPlaneFrame()[zmp::Derivative::Zero][dim];
      opts.vel0_ = motionPlan_->getInitialRobotStateInPlaneFrame()[zmp::Derivative::First][dim];
      opts.acc0_ = objectiveHandler_.getGravityInPlaneFrame(dim);
      opts.posT_ = nextSplineSegment.getPositionAtTime(0.0);
      opts.velT_ = nextSplineSegment.getVelocityAtTime(0.0);
      opts.accT_ = opts.acc0_;
      if (!motionPlan_->getComStateInPlaneFramePtr()->addSpline(curves::PolynomialSplineQuintic(opts), dim)) {
        return false;
      }
    }

    for (unsigned int splineId = 0u; splineId < zmpInfo_.splineInfoSequence_.size(); ++splineId) {
      const unsigned int nextSplineId = splineId + 1u;

      // Add current spline segment (which we have already computed in the previous step).
      splineSegment = nextSplineSegment;
      const double splineDuration = zmpInfo_.splineInfoSequence_[splineId].duration_;
      if (!motionPlan_->getComStateInPlaneFramePtr()->addSpline(splineSegment, dim)) {
        return false;
      }

      // Get the next spline segment (if there is one).
      if (nextSplineId < zmpInfo_.splineInfoSequence_.size()) {
        startIndex = objectiveHandler_.getCoeffStartIndex(nextSplineId, dim);
        Eigen::Map<ZmpSplineCoeff>(coeffsSegment.data(), zmp::splineCoeffs, 1u) = optimizedSolution_.segment(startIndex, zmp::splineCoeffs);
        nextSplineSegment.setCoefficientsAndDuration(coeffsSegment, zmpInfo_.splineInfoSequence_[nextSplineId].duration_);
      }

      /*
       * After the current spline follows a flight phase. Include a virtual flight trajectory that
       * connect the end-point of the current spline with the start-point of the next spline.
       */
      if (zmpInfo_.splineInfoSequence_[splineId].flightDurationOfNextPhase_ > 0.0) {
        opts.tf_ = zmpInfo_.splineInfoSequence_[splineId].flightDurationOfNextPhase_;
        opts.pos0_ = splineSegment.getPositionAtTime(splineDuration);
        opts.vel0_ = splineSegment.getVelocityAtTime(splineDuration);
        opts.acc0_ = objectiveHandler_.getGravityInPlaneFrame(dim);
        opts.accT_ = opts.acc0_;

        if (nextSplineId < zmpInfo_.splineInfoSequence_.size()) {  // there exists a next spline segment
          opts.posT_ = nextSplineSegment.getPositionAtTime(0.0);
          opts.velT_ = nextSplineSegment.getVelocityAtTime(0.0);
        } else {  // connect the flight phase with the predicted final point
          objectiveHandler_.predictTouchDownState(opts.posT_, opts.tf_, zmp::Derivative::Zero, dim, opts.pos0_, opts.vel0_);
          objectiveHandler_.predictTouchDownState(opts.velT_, opts.tf_, zmp::Derivative::First, dim, opts.pos0_, opts.vel0_);
        }
        if (!motionPlan_->getComStateInPlaneFramePtr()->addSpline(curves::PolynomialSplineQuintic(opts), dim)) {
          return false;
        }
      }
    }  // end for splineId

  }  // end for dim

  // For DoF that were not optimized, we use the path regularizer.
  for (const auto& dim : std_utils::enum_iterator<zmp::CogDim>()) {
    if (std_utils::containsEnum(optimizationDofs_, dim)) {
      continue;
    }

    // To avoid jumps in z while standing (e.g. due to changing reference height, reseting of elevation map etc): fit spline.
    if (dim == zmp::CogDim::z) {
      opts.tf_ = motionPlan_->getOptimizationHorizon();
      if (opts.tf_ < 1.0e-3) {
        return false;
      }

      opts.pos0_ = motionPlan_->getInitialRobotStateInPlaneFrame()[zmp::Derivative::Zero][dim];
      opts.posT_ = motionPlan_->getFinalRobotStateInPlaneFrame()[zmp::Derivative::Zero][dim];

      const double posDifference = opts.posT_ - opts.pos0_;
      const double velGain = expZAlpha_ * std::exp(-expZLambda_ * posDifference * posDifference);  // more aggressive around nominal height.
      opts.vel0_ = velGain * posDifference;
      opts.velT_ = 0.0;

      opts.acc0_ = 0.0;
      opts.accT_ = 0.0;

      if (!motionPlan_->getComStateInPlaneFramePtr()->setMotionPlan(curves::PolynomialSplineQuintic(opts), dim)) {
        return false;
      }
    }

    // In any other case: Copy path regularizer.
    else {
      motionPlan_->getComStateInPlaneFramePtr()->setMotionPlan(motionPlan_->getPathRegularizerInPlaneFrame().getMotionPlan()[dim], dim);
    }
  }

  // Store realized container duration.
  motionPlan_->getComStateInPlaneFramePtr()->setContainerDuration(motionPlan_->getOptimizationHorizon());

  return true;
}

bool ZmpOptimizerDynamicWalk::computeTrajectory(zmp::MotionPlan& motionPlan) {
  if (!readMotionPlan(motionPlan)) {
    MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::readMotionPlan] Failed to read the motion plan!")
    motionPlan.setTerminationState(zmp::TerminationState::failedToReadMotionPlan);
    return false;
  }

  if (!createSplineInfoSequence()) {
    MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::createSplineInfoSequence] Failed to create spline info sequence!")
    motionPlan.setTerminationState(zmp::TerminationState::failedToSetUpOptimizationProblem);
    return false;
  }

  if (!setupConstraintProperties()) {
    MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::setupConstraintProperties] Failed to set up constraint properties!")
    motionPlan.setTerminationState(zmp::TerminationState::failedToSetUpOptimizationProblem);
    return false;
  }

  if (!runOptimization()) {
    MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::runOptimization] Optimization failed!")
    motionPlan.setTerminationState(zmp::TerminationState::failedToSolveOptimization);
    return false;
  }

  if (!createSolutionSplines()) {
    MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::createSolutionSplines] Failed to extract the solution!")
    motionPlan.setTerminationState(zmp::TerminationState::failedToCreateSolution);
    return false;
  }

  if (motionPlan.checkMotionPlan()) {
    if (!motionPlan.getComStateInPlaneFrame().checkTrajectoryStateHandler()) {
      MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::computeTrajectory] Com trajectory appears to be invalid!")
      motionPlan.setTerminationState(zmp::TerminationState::invalidSolution);
      return false;
    }
  } else {
    MELO_WARN_STREAM("[ZmpOptimizerDynamicWalk::computeTrajectory] Motion plan appears to be invalid!")
    motionPlan.setTerminationState(zmp::TerminationState::invalidSolution);
    return false;
  }

  motionPlan.setTerminationState(zmp::TerminationState::returnOptimizedSolution);

  return true;
}

} /* namespace loco */
