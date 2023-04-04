/*
 * SwingTrajectoryOptimizer.cpp
 *
 *  Created on: Jan. 15, 2017
 *      Author: Fabian Jenelten
 */

// swing trajectory generation
#include "swing_trajectory_generation/SwingTrajectoryOptimizer.hpp"

// message logger
#include <message_logger/message_logger.hpp>


namespace loco {

SwingTrajectoryOptimizer::SwingTrajectoryOptimizer():
    costFunctionHessian_(),
    costFunctionLinearTerm_(),
    equalityConstraintJacobian_(),
    equalityConstraintTargetValues_(),
    inequalityConstraintJacobian_(),
    inequalityConstraintMinValues_(),
    eqConstraintIdx_(0u),
    numEqConstraints_(0u),
    ineqConstraintIdx_(0u),
    numOfTotSwingSamples_(0u),
    solutionDimension_(0u),
    solutionCoeffs_(),
    motionPlan_(nullptr),
    optimizationProblemQP_(nullptr),
    qpSolver_(nullptr),
    taskHandler_(),
    hessianRegularizer_(1e-8),
    setHardInitAccel_(true),
    weightMinAccel_(-1.0),
    weightTrackingKnotHeightPos_(-1.0),
    weightPreviousSolution_(-1.0),
    weightSwingHeight_(-1.0),
    desiredKnotHeightOverPlane_(),
    desiredKnotTime_(),
    desiredKnotTimeScaled_(),
    timeInstantsPerSplineId_(),
    minNumOfSamplesPerSpline_(2u),
    desiredSwingHeightAbovePlane_(1.0)
{

}

bool SwingTrajectoryOptimizer::loadParameters(const TiXmlHandle& SwingTrajectoryGeneratorHandle) {
  bool success = true;

  // Height trajectory
  const TiXmlHandle heightTrajHandle = tinyxml_tools::getChildHandle(SwingTrajectoryGeneratorHandle, "HeightTrajectory");
  success &= tinyxml_tools::loadParameter(desiredSwingHeightAbovePlane_, heightTrajHandle, "swing_height", 1.0);

  std::vector<TiXmlElement*> swingHeightElements;
  success &= tinyxml_tools::getChildElements(swingHeightElements, heightTrajHandle, "Knot");

  // Parse the child elements.
  const unsigned int numOfKnotPoints = swingHeightElements.size()+2u;
  desiredKnotTimeScaled_.clear();
  desiredKnotHeightOverPlane_.clear();
  desiredKnotTimeScaled_.reserve(numOfKnotPoints);
  desiredKnotHeightOverPlane_.reserve(numOfKnotPoints);
  double knotTime; double knotValue;

  desiredKnotTimeScaled_.push_back(0.0);
  desiredKnotHeightOverPlane_.push_back(0.0);

  for (const auto& knot : swingHeightElements) {
    success &= tinyxml_tools::loadParameter(knotTime,  knot, "t");
    success &= tinyxml_tools::loadParameter(knotValue, knot, "v");


    if (knotTime<=desiredKnotTimeScaled_.back() || knotTime>=1.0 || knotValue<0.0) {
      MELO_FATAL_STREAM("[SwingTrajectoryOptimizer::loadParameters] Wrong knot point definition.");
      return false;
    }

    desiredKnotTimeScaled_.push_back(knotTime);
    desiredKnotHeightOverPlane_.push_back(knotValue);
  }

  desiredKnotTimeScaled_.push_back(1.0);
  desiredKnotHeightOverPlane_.push_back(0.0);

  // Weights.
  const TiXmlHandle weightHandle = tinyxml_tools::getChildHandle(SwingTrajectoryGeneratorHandle, "Weights");
  success &= tinyxml_tools::loadParameter(weightMinAccel_,    weightHandle, "min_accel",    -1.0);
  success &= tinyxml_tools::loadParameter(weightSwingHeight_, weightHandle, "swing_height", -1.0);
  success &= tinyxml_tools::loadParameter(weightTrackingKnotHeightPos_, weightHandle, "knot_pos", -1.0);
  success &= tinyxml_tools::loadParameter(weightPreviousSolution_[zmp::Derivative::Zero], weightHandle,  "previous_pos", -1.0);
  success &= tinyxml_tools::loadParameter(weightPreviousSolution_[zmp::Derivative::First], weightHandle, "previous_vel", -1.0);

  // Equality constraints.
  const TiXmlHandle eqConstraintsHandle = tinyxml_tools::getChildHandle(SwingTrajectoryGeneratorHandle, "EqualityConstraints");
  success &= tinyxml_tools::loadParameter(setHardInitAccel_, eqConstraintsHandle, "enable_hard_init_accel", true);

  // Sampling.
  const TiXmlHandle samplingHandle = tinyxml_tools::getChildHandle(SwingTrajectoryGeneratorHandle, "Sampling");
  success &= tinyxml_tools::loadParameter(numOfTotSwingSamples_, samplingHandle, "num_of_samples", 10u);

  return success;
}

bool SwingTrajectoryOptimizer::initialize() {
  optimizationProblemQP_.reset(new numopt_common::QuadraticProblem(
      std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new numopt_common::QuadraticObjectiveFunction),
      std::shared_ptr<numopt_common::LinearFunctionConstraints>(new numopt_common::LinearFunctionConstraints)
    )
  );
  qpSolver_.reset(new numopt_quadprog::ActiveSetFunctionMinimizer());
  const std::vector<zmp::CogDim> optimizationTranslationalDofs = {zmp::CogDim::x, zmp::CogDim::y, zmp::CogDim::z};
  if(!taskHandler_.initialize()) { return false; }
  taskHandler_.setOptimizationDofs(optimizationTranslationalDofs);
  return true;
}


bool SwingTrajectoryOptimizer::computeTrajectory(sto::MotionPlan& motionPlan) {
  // Get motion plan (there is no need for a deep copy).
  motionPlan_ = &motionPlan;

  // Compute QP matrices.
  if (!setupConstraintProperties()) {
    MELO_WARN_STREAM("[SwingTrajectoryOptimizer::computeTrajectory] Failed to set up constraint properties.");
    motionPlan_->setTerminationState(zmp::TerminationState::failedToSetUpOptimizationProblem);
    return false;
  }

  if (!addTasks()) {
    MELO_WARN_STREAM("[SwingTrajectoryOptimizer::computeTrajectory] Failed to add tasks.");
    motionPlan_->setTerminationState(zmp::TerminationState::failedToSetUpOptimizationProblem);
    return false;
  }

  // Set up QP problem.
  optimizationProblemQP_->setOptimizationMatrices(
      costFunctionHessian_.sparseView(),
      costFunctionLinearTerm_,
      equalityConstraintJacobian_.sparseView(),
      inequalityConstraintJacobian_.sparseView(),
      equalityConstraintTargetValues_,
      inequalityConstraintMinValues_);

  // Solve QP.
  double cost;
  if (!qpSolver_->minimize(optimizationProblemQP_.get(), solutionCoeffs_, cost)) {
    MELO_WARN_STREAM("[SwingTrajectoryOptimizer::computeTrajectory] Failed to solve optimization.");
    motionPlan_->setTerminationState(zmp::TerminationState::failedToSolveOptimization);
    return false;
  }

  // Store solution.
  if(!createSolutionSplines()) {
    MELO_WARN_STREAM("[SwingTrajectoryOptimizer::computeTrajectory] Failed to create solution splines.");
    motionPlan_->setTerminationState(zmp::TerminationState::failedToCreateSolution);
    return false;
  }

  motionPlan_->setTerminationState(zmp::TerminationState::returnOptimizedSolution);

  return true;
}

bool SwingTrajectoryOptimizer::setupConstraintProperties() {

  // Reset constraints.
  numEqConstraints_   = 0u;
  eqConstraintIdx_    = 0u;
  ineqConstraintIdx_  = 0u;

  // Read motion plan.
  const auto swingDuration       = motionPlan_->getSwingDuration();
  const auto trajectoryStartTime = motionPlan_->getTrajectoryStartTime();

  /***********
   * Splines *
   ***********/
  std::vector<double> swingSplineDurations(0u);
  double previousKnotTime = -1.0;

  desiredKnotTime_.clear();
  desiredKnotTime_.reserve(desiredKnotTimeScaled_.size());

  for (const auto knotTimeScaled : desiredKnotTimeScaled_) {
    // Add knot time.
    const double knotTime = knotTimeScaled*swingDuration;
    desiredKnotTime_.push_back(knotTime);

    // Add first spline of the trajectory.
    if (previousKnotTime != -1.0) {
      if (previousKnotTime<=trajectoryStartTime && knotTime>trajectoryStartTime) {
        swingSplineDurations.push_back(knotTime-trajectoryStartTime);
      }

      // Add following splines.
      else if (knotTime>trajectoryStartTime) {
        swingSplineDurations.push_back(knotTime-previousKnotTime);
      }
    }

    previousKnotTime = knotTime;
  }

  // Set spline duration vector and number of full splines.
  const unsigned int numOfTotSwingSplines = desiredKnotTimeScaled_.size()-1u;
  const unsigned int numOfSplines = swingSplineDurations.size();
  motionPlan_->setSplineDurations(swingSplineDurations, numOfTotSwingSplines);
  /***********/


  /******************
   * Discretization *
   *****************/
  timeInstantsPerSplineId_.resize(numOfSplines);
  const auto sampleTimeIdeal = swingDuration / static_cast<double>(numOfTotSwingSamples_);
  const auto minSplineDuration = static_cast<double>(minNumOfSamplesPerSpline_)*sampleTimeIdeal;
  auto numOfSamples = 0u;

  for (unsigned int splineId=0u; splineId<numOfSplines; ++splineId) {
    timeInstantsPerSplineId_[splineId].clear();
    const double splineEnd   = swingSplineDurations[splineId];
    const double sampleTime  = (swingSplineDurations[splineId]>minSplineDuration ? sampleTimeIdeal : swingSplineDurations[splineId]/static_cast<double>(minNumOfSamplesPerSpline_));
    const double splineStart = (splineId==0u ? sampleTime : 0.0); // Skip initial conditions

    // Add discrete samples.
    for (double tk=splineStart; tk<splineEnd; tk+=sampleTime) {
      timeInstantsPerSplineId_[splineId].push_back(tk);
    }
    numOfSamples += timeInstantsPerSplineId_[splineId].size();
  }
  /******************/

  // Number of equality constraints.
  numEqConstraints_ += zmp::dofTransl*(setHardInitAccel_ ? (unsigned int)zmp::Derivative::SIZE : (unsigned int)zmp::Derivative::SIZE-1u);   // initial constraints
  numEqConstraints_ += zmp::dofTransl*(unsigned int)zmp::Derivative::SIZE;                                                                  // final constraints
  numEqConstraints_ += zmp::dofTransl*(unsigned int)zmp::Derivative::SIZE*(numOfSplines-1u);                                                // junction constraints

  // Number of unknown spline coefficients.
  solutionDimension_ = zmp::coeffsTransl*numOfSplines;

  // Resize.
  costFunctionHessian_.setZero(solutionDimension_, solutionDimension_);
  costFunctionLinearTerm_.setZero(solutionDimension_);

  equalityConstraintJacobian_.setZero(numEqConstraints_, solutionDimension_);
  equalityConstraintTargetValues_.setZero(numEqConstraints_);

  return true;
}

bool SwingTrajectoryOptimizer::addContinuousTimeTrackingObjectives(
    unsigned int splineId,
    unsigned int coeffId,
    zmp::Derivative derivative,
    zmp::CogDim dim) {

  bool success = true;

  // Track knot points at spline ends
  if (weightTrackingKnotHeightPos_>0.0 &&
      derivative == zmp::Derivative::Zero && dim == zmp::CogDim::z &&
      splineId<motionPlan_->getSwingSplineDurations().size()-1u)
  {
    const unsigned int knotId = splineId+1u+motionPlan_->getSplineIdOffset();
    success &= taskHandler_.getFinalStateObjective(
        costFunctionHessian_.block<zmp::splineCoeffs, zmp::splineCoeffs>(coeffId, coeffId),
        costFunctionLinearTerm_.segment<zmp::splineCoeffs>(coeffId),
        desiredKnotHeightOverPlane_[knotId],
        derivative,
        weightTrackingKnotHeightPos_);
  }

  return success;
}

bool SwingTrajectoryOptimizer::addDiscreteTimeTrackingObjectives(
    double splineTimePreviousSolution,
    double sampleTime,
    unsigned int coeffId,
    zmp::Derivative derivative,
    zmp::CogDim dim) {

  bool success = true;

  // Track previous solution.
  if (weightPreviousSolution_[derivative]>0.0 &&
      motionPlan_->isPreviousOptimizationAvailable() &&
      splineTimePreviousSolution<motionPlan_->getSwingTrajectoryInPlaneFrame().getContainerDuration() &&
      sampleTime>hessianRegularizer_) {


    // Compute desired state in current virtual plane frame.
    Eigen::Vector3d desiredState;
    const auto& posePreviousPlaneToWorld = motionPlan_->getPosePreviousPlaneToWorld();
    const auto& posePlaneToWorld         = motionPlan_->getVirtualPlaneFrame().getPosePlaneToWorld();

    if (derivative == zmp::Derivative::Zero) {
      const Position positionWorldToTargetInWorldFrame = posePreviousPlaneToWorld.transform(motionPlan_->getSwingTrajectoryInPlaneFrame().getPositionPlaneToComInPlaneFrameAtTime(splineTimePreviousSolution));
      desiredState = posePlaneToWorld.inverseTransform(positionWorldToTargetInWorldFrame).toImplementation();
    } else if (derivative == zmp::Derivative::First) {
      const LinearVelocity velocityTargetInWorldFrame = posePreviousPlaneToWorld.getRotation().rotate(motionPlan_->getSwingTrajectoryInPlaneFrame().getLinearVelocityComInPlaneFrameAtTime(splineTimePreviousSolution));
      desiredState = posePlaneToWorld.getRotation().inverseRotate(velocityTargetInWorldFrame).toImplementation();
    } else if (derivative == zmp::Derivative::Second) {
      const LinearAcceleration accelerationTargetInWorldFrame = posePreviousPlaneToWorld.getRotation().rotate(motionPlan_->getSwingTrajectoryInPlaneFrame().getLinearAccelerationComInPlaneFrameAtTime(splineTimePreviousSolution));
      desiredState = posePlaneToWorld.getRotation().inverseRotate(accelerationTargetInWorldFrame).toImplementation();
    } else {
      return false;
    }

    success &= taskHandler_.getStateObjective(
        costFunctionHessian_.block<zmp::splineCoeffs, zmp::splineCoeffs>(coeffId, coeffId),
        costFunctionLinearTerm_.segment<zmp::splineCoeffs>(coeffId),
        desiredState(zmp::toIndex(dim)),
        derivative,
        weightPreviousSolution_[derivative]*sampleTime);
  }

  // Track desired swing height.
  if (weightSwingHeight_>0.0 && dim == zmp::CogDim::z && derivative == zmp::Derivative::Zero) {
    success &= taskHandler_.getStateObjective(
        costFunctionHessian_.block<zmp::splineCoeffs, zmp::splineCoeffs>(coeffId, coeffId),
        costFunctionLinearTerm_.segment<zmp::splineCoeffs>(coeffId),
        desiredSwingHeightAbovePlane_,
        derivative,
        weightSwingHeight_*sampleTime);
  }

  return success;
}

bool SwingTrajectoryOptimizer::addAccelerationObjective(
    unsigned int splineId,
    unsigned int coeffId,
    zmp::CogDim dim) {
  bool success = true;

  // Minimize acceleration along spline.
  if (weightMinAccel_>0.0) {
    success &= taskHandler_.getAccelerationObjective(
        costFunctionHessian_.block<zmp::splineCoeffs-2, zmp::splineCoeffs-2>(coeffId, coeffId),
        weightMinAccel_);

    // Regularization (render Hessian positive definite).
    if (hessianRegularizer_>0.0) {
      costFunctionHessian_.block<2, 2>(coeffId+4,  coeffId+4) += Eigen::Matrix2d::Identity()*hessianRegularizer_;
    }
  }

  // Pure Regularization.
  else if (hessianRegularizer_>0.0) {
    costFunctionHessian_.block<zmp::splineCoeffs, zmp::splineCoeffs>(coeffId,  coeffId) +=
        Eigen::Matrix<double,zmp::splineCoeffs,zmp::splineCoeffs>::Identity()* hessianRegularizer_;
  }

  return success;
}

bool SwingTrajectoryOptimizer::addHardInitialAndFinalConstraints(
    unsigned int splineId,
    unsigned int coeffId,
    zmp::Derivative derivative,
    zmp::CogDim dim) {

  // Hard initial conditions.
  if (splineId==0u && !(derivative == zmp::Derivative::Second && !setHardInitAccel_)) {
    taskHandler_.getInitialStateEqualityConstraints(
        equalityConstraintJacobian_.block<1u, zmp::splineCoeffs>(eqConstraintIdx_, coeffId),
        derivative);

    equalityConstraintTargetValues_(eqConstraintIdx_) = motionPlan_->getInitialRobotStateInPlaneFrame()[derivative][dim];
    ++eqConstraintIdx_;
  }

  // Hard final Conditions.
  if (splineId==motionPlan_->getSwingSplineDurations().size()-1u) {
    taskHandler_.getFinalStateEqualityConstraints(
        equalityConstraintJacobian_.block<1u, zmp::splineCoeffs>(eqConstraintIdx_, coeffId),
        derivative);
    equalityConstraintTargetValues_(eqConstraintIdx_) = motionPlan_->getFinalRobotStateInPlaneFrame()[derivative][dim];
    ++eqConstraintIdx_;
  }

  return true;
}


bool SwingTrajectoryOptimizer::addJunctionConstraints(
    unsigned int splineId,
    unsigned int coeffId,
    unsigned int nextCoeffId,
    zmp::Derivative derivative) {
  bool success = true;

  if (motionPlan_->getSwingSplineDurations().size()>1u && splineId<motionPlan_->getSwingSplineDurations().size()-1) {
    success &= taskHandler_.getJunctionConstraints(
        equalityConstraintJacobian_.block<1,zmp::splineCoeffs>(eqConstraintIdx_, coeffId),
        equalityConstraintJacobian_.block<1,zmp::splineCoeffs>(eqConstraintIdx_, nextCoeffId),
        equalityConstraintTargetValues_(eqConstraintIdx_),
        derivative);
    ++eqConstraintIdx_;
  }

  return success;
}

bool SwingTrajectoryOptimizer::addTasks() {
  bool success = true;

  /*******************
   * Continuous Time *
   *******************/
  for (unsigned int splineId = 0u; splineId < motionPlan_->getSwingSplineDurations().size(); ++splineId) {
    success &= taskHandler_.setActiveSplineId(splineId, motionPlan_->getSwingSplineDurations()[splineId]);

    for (const auto dim : zmp::optimizationTranslationalDofs) {
      const unsigned int coeffId     = taskHandler_.getCoeffStartIndex(splineId,   dim);
      const unsigned int nextCoeffId = taskHandler_.getCoeffStartIndex(splineId+1, dim);

      for (const auto derivative : std_utils::enum_iterator<zmp::Derivative>()) {
        // Task 1: Initial and final conditions.
        success &= addHardInitialAndFinalConstraints(splineId, coeffId, derivative, dim);

        // Task 2: Junction conditions.
        success &= addJunctionConstraints(splineId,coeffId, nextCoeffId, derivative);

        // Task 3: Approach to knot points.
        success &= addContinuousTimeTrackingObjectives(splineId, coeffId, derivative, dim);
      }

      // Task 3: Minimize acceleration.
      success &= addAccelerationObjective(splineId, coeffId, dim);
    }
  }
  /*******************/


  /*****************
   * Discrete Time *
   *****************/
  double timeAtSplineStart = 0.0;
  double tk_previous = 0.0;

  for (unsigned int splineId = 0u; splineId < motionPlan_->getSwingSplineDurations().size(); ++splineId) {
    success &= taskHandler_.setActiveSplineId(splineId, motionPlan_->getSwingSplineDurations()[splineId]);

    for (const auto& tk: timeInstantsPerSplineId_[splineId]) {
      taskHandler_.setTime(tk);

      const double splineTimePreviousSolution = timeAtSplineStart+tk+motionPlan_->getSwingTrajectoryInPlaneFramePtr()->getContainerTime();
      const double sampleTime = tk - tk_previous;
      assert(sampleTime>0.0);

      for (const auto dim : zmp::optimizationTranslationalDofs) {
        // todo: getCoeffStartIndex should return unsigned int
        const unsigned int coeffId = taskHandler_.getCoeffStartIndex(splineId,   dim);

        for (const auto derivative : std_utils::enum_iterator<zmp::Derivative>()) {
          //! Track previous solution.
          success &=  addDiscreteTimeTrackingObjectives(splineTimePreviousSolution, sampleTime, coeffId, derivative, dim);
        }
      }

      tk_previous = tk;
    }

    tk_previous       -= motionPlan_->getSwingSplineDurations()[splineId];
    timeAtSplineStart += motionPlan_->getSwingSplineDurations()[splineId];
  }
  /*****************/

  // Check constraints
  if (eqConstraintIdx_ != numEqConstraints_) {
    MELO_FATAL_STREAM("[SwingTrajectoryOptimizer::addTasks] Wrong number of equality constraints. eqConstraintIdx_ = " << eqConstraintIdx_ << ", numEqConstraints_ = " << numEqConstraints_ << ".");
    return false;
  }

  // Fill in also the upper triangular part.
  const Eigen::MatrixXd fullHessianQP = costFunctionHessian_.selfadjointView<Eigen::Lower>();
  costFunctionHessian_ = fullHessianQP;

  return success;
}

bool SwingTrajectoryOptimizer::createSolutionSplines() {

  // Clear previous solution.
  if(!motionPlan_->resetSwingTrajectory()) { return false; }
  curves::PolynomialSplineQuintic::SplineCoefficients coeffsSegment;
  curves::PolynomialSplineQuintic splineSegment;

  for (const auto dim : zmp::optimizationTranslationalDofs) {
    for (unsigned int splineId=0u; splineId<motionPlan_->getSwingSplineDurations().size(); ++splineId) {
      const double splineDuration = motionPlan_->getSwingSplineDurations()[splineId];
      const unsigned int startIndex = taskHandler_.getCoeffStartIndex(splineId, dim);

      // Convert Eigen::Vector to std::array
      Eigen::Map<Eigen::Matrix<double, zmp::splineCoeffs, 1u>>(coeffsSegment.data(), zmp::splineCoeffs, 1u) = solutionCoeffs_.getParams().segment(startIndex, zmp::splineCoeffs);

      // Creat spline segment and add it to the overall solution.
      splineSegment.setCoefficientsAndDuration(coeffsSegment, splineDuration);
      if(!motionPlan_->getSwingTrajectoryInPlaneFramePtr()->addSpline(splineSegment, dim)) { return false; }
    }
  }

  return motionPlan_->updateContainerDuration();
}



} /* namespace sto */

