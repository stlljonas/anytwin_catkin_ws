/*
 * NonLinearZmpFunctionConstraints.cpp
 *
 *  Created on: 03.26, 2017
 *      Author: Fabian Jenelten
 */

// zmp optimizer
#include <zmp_optimizer/NonLinearZmpFunctionConstraints.hpp>

// logger
#include <message_logger/message_logger.hpp>

namespace zmp {

NonLinearZmpFunctionConstraints::NonLinearZmpFunctionConstraints(ZmpOptimizerObjectiveHandler& objectiveHandler,
                                                                 const std_utils::EnumArray<CogDim, double>& finalMaxState,
                                                                 const std_utils::EnumArray<Ineq, bool>& enableInequalityConstraints,
                                                                 bool useConstraintHessian)
    : LinearZmpFunctionConstraints(objectiveHandler, finalMaxState, enableInequalityConstraints, useConstraintHessian),
      hessianZmpInequality_(),
      ineqConstraintIdxSQP_(0u),
      numOfInequalityConstraintsSQP_(0u),
      numOfInequalityConstraintsQP_(0u),
      InequalityGradientSQP_(),
      InequalityConstraintValuesSQP_(),
      currentEqualityConstraintValues_(),
      currentInequalityConstraintValues_(),
      useConstraintHessian_(useConstraintHessian) {}

bool NonLinearZmpFunctionConstraints::initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) {
  if (!LinearZmpFunctionConstraints::initialize(zmpInfo, motionPlan)) {
    return false;
  }

  numOfInequalityConstraintsQP_ = zmpInfo.numOfInequalityConstraintsQP_;
  numOfInequalityConstraintsSQP_ = zmpInfo.numOfInequalityConstraintsSQP_;

  // Add nonlinear constraints.
  nEqualityConstraints_ += static_cast<int>(zmpInfo.numOfEqualityConstraintsSQP_);
  nInequalityConstraints_ += static_cast<int>(zmpInfo.numOfInequalityConstraintsSQP_);

  // Initialize nonlinear constraints.
  InequalityGradientSQP_.setZero(numOfInequalityConstraintsSQP_, numOfUnknowns_);
  InequalityConstraintValuesSQP_.setZero(numOfInequalityConstraintsSQP_);

  if (useConstraintHessian_) {
    hessianZmpInequality_.resize(numOfInequalityConstraintsSQP_);
    for (auto& hessian : hessianZmpInequality_) {
      hessian.setZero(numOfUnknowns_, numOfUnknowns_);
    }
  }

  return true;
}

bool NonLinearZmpFunctionConstraints::doComputationBeforeIteration(const numopt_common::Parameterization& params) {
  // Compute inequality constraint gradient.
  ineqConstraintIdxSQP_ = 0u;
  zmpEpsilonStatesCounter_ = 0u;
  InequalityGradientSQP_.setZero();
  InequalityConstraintValuesSQP_.setZero();
  if (useConstraintHessian_) {
    for (auto& hessian : hessianZmpInequality_) {
      hessian.setZero(numOfUnknowns_, numOfUnknowns_);
    }
  }

  robot_utils::geometry::Polygon::LineCoefficientList lineCoeffs;
  double timeAtSplineStart = 0.0;
  robot_utils::geometry::Polygon::LineCoefficientList lineCoefficients;

  // Skip flight phase at start
  if (isFirstSupportFlightPhase_) {
    timeAtSplineStart += supportPolygons_->front().getDuration();
  }

  for (const auto& splineInfo : *splineInfoSequence_) {
    // Skip splines that were not accessed a sample time
    if (timeInstantsPerSplineId_->operator[](splineInfo.id_).empty()) {
      timeAtSplineStart += splineInfo.duration_;
      if (splineInfo.flightDurationOfNextPhase_ > 0.0) {
        timeAtSplineStart += splineInfo.flightDurationOfNextPhase_;
      }
      continue;
    }

    if (!objectiveHandler_.setActiveSplineId(splineInfo.id_, splineInfo.duration_)) {
      return false;
    }
    const unsigned int coeffStartId = objectiveHandler_.getCoeffStartIndex(splineInfo.id_);

    for (const auto& tk : timeInstantsPerSplineId_->operator[](splineInfo.id_)) {
      objectiveHandler_.setTime(tk);
      getLineCoefficients(lineCoeffs, splineInfo);
      const double splineTime = timeAtSplineStart + tk;
      if (!addZmpConstraints(coeffStartId, splineInfo.id_, splineTime, lineCoeffs, params.getParams())) {
        MELO_WARN_STREAM("[NonLinearZmpFunctionConstraints::doComputationBeforeIteration] Failed to add nonlinear zmp constraints.")
        return false;
      }
    }

    // Skip flight phases.
    timeAtSplineStart += splineInfo.duration_;
    if (splineInfo.flightDurationOfNextPhase_ > 0.0) {
      timeAtSplineStart += splineInfo.flightDurationOfNextPhase_;
    }
  }

  // Safety check.
  if (nInequalityConstraints_ != static_cast<int>(ineqConstraintIdxQP_ + ineqConstraintIdxSQP_)) {
    MELO_FATAL_STREAM("[NonLinearZmpFunctionConstraints::doComputationBeforeIteration] Wrong number of inequality constraints.")
    return false;
  }

  return computeConstraintValues(params);
}

bool NonLinearZmpFunctionConstraints::getLocalInequalityConstraintJacobian(numopt_common::SparseMatrix& jacobian,
                                                                           const numopt_common::Parameterization& /*params*/,
                                                                           bool /*newParams*/) {
  if (ineqConstraintIdxSQP_ > 0) {
    /*
     * ToDo: When this code was written, the used version of Eigen had a bug,
     * leading to a memory leak using the following lines of code. If this bug has been fixed
     * (e.g. when the version has been updated), the code can be uncommented again.
     */

    //    jacobian.resize(nInequalityConstraints_, numUnknownCoeffs_);
    //    jacobian.topRows(ineqConstraintIdxQP_)     = ineqMatQP_.sparseView();
    //    jacobian.bottomRows(ineqConstraintIdxSQP_) = InequalityGradientSQP_.sparseView();

    Eigen::MatrixXd ineqMatQP;
    ineqMatQP.resize(nInequalityConstraints_, numOfUnknowns_);
    ineqMatQP.topRows(numOfInequalityConstraintsQP_) = ineqMatQP_.sparseView();
    ineqMatQP.bottomRows(numOfInequalityConstraintsSQP_) = InequalityGradientSQP_.sparseView();
    jacobian = ineqMatQP.sparseView();
  }

  // Jacobian of linear inequality constraints Ax>=b -> Jacobian = A
  else {
    jacobian = ineqMatQP_.sparseView();
  }

  return true;
}

bool NonLinearZmpFunctionConstraints::getLocalInequalityConstraintHessian(numopt_common::SparseMatrix& hessian,
                                                                          const numopt_common::Parameterization& /*params*/,
                                                                          int iConstraint, bool /*newParams*/) {
  const int zmpConstraintIndex = mapToZmpConstaintIndex(iConstraint);

  // Constrain index does not correspond to a zmp constraint
  if (!useConstraintHessian_ || zmpConstraintIndex == -1) {
    hessian.resize(numOfUnknowns_, numOfUnknowns_);
    hessian.setZero();
    return true;
  }

  // Hessian of zmp inequality constraints.
  if (useConstraintHessian_ && zmpConstraintIndex >= 0 && static_cast<size_t>(zmpConstraintIndex) < hessianZmpInequality_.size()) {
    hessian = hessianZmpInequality_[zmpConstraintIndex].sparseView();
    return true;
  }

  MELO_WARN_STREAM("[NonLinearZmpFunctionConstraints::getLocalInequalityConstraintHessian] Failed to find zmp constraint.")
  return false;
}

bool NonLinearZmpFunctionConstraints::getInequalityConstraintMinValues(numopt_common::Vector& minValues) {
  // Min values of nonlinear inequality constraints c(x)>=0 -> min values = 0
  minValues.setZero(nInequalityConstraints_);

  // Min values of linear inequality constraints Ax>=b -> min values = b
  minValues.head(numOfInequalityConstraintsQP_) = ineqConMinValues_;

  return true;
}

bool NonLinearZmpFunctionConstraints::getInequalityConstraintMaxValues(numopt_common::Vector& maxValues) {
  // No inequality max values. Enforce numerical limits instead.
  maxValues.setConstant(nInequalityConstraints_, std::numeric_limits<double>::max());
  return true;
}

bool NonLinearZmpFunctionConstraints::getInequalityConstraintValues(numopt_common::Vector& values,
                                                                    const numopt_common::Parameterization& /*params*/, bool /*newParams*/) {
  values = currentInequalityConstraintValues_;
  return true;
}

bool NonLinearZmpFunctionConstraints::getLocalEqualityConstraintJacobian(numopt_common::SparseMatrix& jacobian,
                                                                         const numopt_common::Parameterization& /*params*/,
                                                                         bool /*newParams*/) {
  // Jacobian of linear equality constraints Ax=b -> Jacobian = A
  jacobian = eqMatQP_.sparseView();
  return true;
}

bool NonLinearZmpFunctionConstraints::getLocalEqualityConstraintHessian(numopt_common::SparseMatrix& hessian,
                                                                        const numopt_common::Parameterization& /*params*/,
                                                                        int /*iConstraint*/, bool /*newParams*/) {
  // Hessian of linear equality constraints Ax=b -> Hessian = 0
  hessian.resize(numOfUnknowns_, numOfUnknowns_);
  hessian.setZero();
  return true;
}

bool NonLinearZmpFunctionConstraints::getEqualityConstraintTargetValues(numopt_common::Vector& values) {
  values = eqConTargetValues_;
  return true;
}

bool NonLinearZmpFunctionConstraints::getEqualityConstraintValues(numopt_common::Vector& values,
                                                                  const numopt_common::Parameterization& /*params*/, bool /*newParams*/) {
  values = currentEqualityConstraintValues_;
  return true;
}

bool NonLinearZmpFunctionConstraints::getGlobalBoundConstraintMinValues(numopt_common::Vector& values) {
  // x >= minBound
  values = globalMinBounds_;
  return true;
}

bool NonLinearZmpFunctionConstraints::getGlobalBoundConstraintMaxValues(numopt_common::Vector& values) {
  // x <= maxBound
  values = globalMaxBounds_;
  return true;
}

bool NonLinearZmpFunctionConstraints::addZmpConstraints(unsigned int coeffStartId, unsigned int /*splineId*/, double splineTime,
                                                        const robot_utils::geometry::Polygon::LineCoefficientList& lineCoefficients,
                                                        const Eigen::VectorXd& previousSplineCoefficients) {
  /*
   * Zmp constraints: Zmp sample must lay within a polygon that is spanned by all the
   * grounded legs at that time instance.
   * Compute gradient and function evaluation of the nonlinear zmp constraints
   *    c(x) >= 0
   */
  bool success = true;

  // Extract path regularizer state.
  const auto pathPosition = pathRegularizer_->getPositionPlaneToComInPlaneFrameAtTime(splineTime);
  const auto pathAcceleration = pathRegularizer_->getLinearAccelerationComInPlaneFrameAtTime(splineTime);

  const auto eulerAnglesPathZyx = pathRegularizer_->getAnglesZyxBaseToPlaneAtTime(splineTime);
  const auto eulerAnglesPathZyx_dot = pathRegularizer_->getEulerRatesZyxBaseInPlaneFrameAtTime(splineTime);
  const auto eulerAnglesPathZyx_ddot = pathRegularizer_->getEulerAccelerationZyxBaseInPlaneFrameAtTime(splineTime);

  // constraints c(x) >= 0.
  const unsigned int coeffIdEpsilon = getFirstZmpEpsilonIndex() + zmpEpsilonStatesCounter_;
  Eigen::VectorXd inequalityZmpGradientSQP;
  for (const auto& lineConstraint : lineCoefficients) {
    // Optimization for some translational states.
    if (objectiveHandler_.getNumOfTranslationalStates() > 0u && objectiveHandler_.getNumOfRotationalStates() == 0u) {
      inequalityZmpGradientSQP.resize(coeffsTransl);

      // Gradient and Hessian (Assumption: translational states come first).
      if (!useConstraintHessian_) {
        success &= objectiveHandler_.getTranslationalZmpInequalityConstraintObjective(
            inequalityZmpGradientSQP, InequalityConstraintValuesSQP_(ineqConstraintIdxSQP_), previousSplineCoefficients, lineConstraint,
            pathPosition, pathAcceleration);
      } else {
        success &= objectiveHandler_.getTranslationalZmpInequalityConstraintObjective(
            inequalityZmpGradientSQP,
            hessianZmpInequality_[ineqConstraintIdxSQP_].block<coeffsTransl, coeffsTransl>(coeffStartId, coeffStartId),
            InequalityConstraintValuesSQP_(ineqConstraintIdxSQP_), previousSplineCoefficients, lineConstraint, pathPosition,
            pathAcceleration);
      }
    } else if (objectiveHandler_.getNumOfTranslationalStates() > 0u && objectiveHandler_.getNumOfRotationalStates() > 0u) {
      // Optimization for some translational states and some rotational states.
      inequalityZmpGradientSQP.resize(coeffsFullState);

      success &= objectiveHandler_.getRotationalTranslationalZmpInequalityConstraintObjective(
          inequalityZmpGradientSQP, InequalityConstraintValuesSQP_(ineqConstraintIdxSQP_), previousSplineCoefficients, lineConstraint,
          pathPosition, pathAcceleration, eulerAnglesPathZyx, eulerAnglesPathZyx_dot, eulerAnglesPathZyx_ddot);
    } else {
      MELO_FATAL_STREAM("[NonLinearZmpFunctionConstraints::addZmpConstraints] Zmp constraints not implemented for this case.")
      return false;
    }

    // Remove gradient for dofs we do not optimize for.
    auto offsetId = 0u;
    for (const auto& dim : objectiveHandler_.getOptimizationDofs()) {
      InequalityGradientSQP_.block<1, splineCoeffs>(ineqConstraintIdxSQP_, coeffStartId + offsetId) =
          inequalityZmpGradientSQP.segment<splineCoeffs>(toStackedIndex(dim) * splineCoeffs).transpose();
      offsetId += splineCoeffs;
    }

    // Softening c(x) >= -epsilon.
    if (zmpEpsilonStatesCounter_ < numZmpEspilonStates_) {
      InequalityGradientSQP_(ineqConstraintIdxSQP_, coeffIdEpsilon) = 1.0;
      InequalityConstraintValuesSQP_(ineqConstraintIdxSQP_) += previousSplineCoefficients(coeffIdEpsilon);
    }
    ++ineqConstraintIdxSQP_;
  }

  // Note: there is one epsilon state per sample (and not per line constraint).
  if (zmpEpsilonStatesCounter_ < numZmpEspilonStates_) {
    ++zmpEpsilonStatesCounter_;
  }

  return success;
}

bool NonLinearZmpFunctionConstraints::computeConstraintValues(const numopt_common::Parameterization& params) {
  // Evaluate linear inequality constraint values Ax>=b at x*  -> constraint value = Ax*.
  currentInequalityConstraintValues_.resize(nInequalityConstraints_);
  currentInequalityConstraintValues_.head(numOfInequalityConstraintsQP_) = ineqMatQP_ * params.getParams();

  // Evaluate nonlinear inequality constraint values c(x)>=0 at x*  -> constraint value = c(x*)
  currentInequalityConstraintValues_.tail(numOfInequalityConstraintsSQP_) = InequalityConstraintValuesSQP_;

  // Evaluate linear equality constraint values  Ax=b at x*  -> constraint value = Ax*.
  currentEqualityConstraintValues_ = eqMatQP_ * params.getParams();

  return true;
}

int NonLinearZmpFunctionConstraints::mapToZmpConstaintIndex(unsigned int constraintIndex) const {
  /* Assumption on numbering the constraints:
   * 1. Inequality constraints imposed on COG -> QP constraints.
   * 2. Inequality constraints imposed on ZMP -> SQP constraints.
   * Non zmp constraints map to -1.
   */
  return std::max(-1, static_cast<int>(constraintIndex) - static_cast<int>(numOfInequalityConstraintsQP_));
}

bool NonLinearZmpFunctionConstraints::useConstraintHessian() const noexcept {
  return useConstraintHessian_;
}

}  // namespace zmp
