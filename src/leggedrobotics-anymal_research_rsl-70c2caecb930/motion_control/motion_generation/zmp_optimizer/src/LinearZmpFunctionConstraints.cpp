/*
 * LinearZmpFunctionConstraints.cpp
 *
 *  Created on: 03.26, 2017
 *      Author: Fabian Jenelten
 */

// zmp optimizer
#include <zmp_optimizer/LinearZmpFunctionConstraints.hpp>

// logger
#include <message_logger/message_logger.hpp>

namespace zmp {

LinearZmpFunctionConstraints::LinearZmpFunctionConstraints(ZmpOptimizerObjectiveHandler& objectiveHandler,
                                                           const std_utils::EnumArray<CogDim, double>& finalMaxState,
                                                           const std_utils::EnumArray<Ineq, bool>& enableInequalityConstraints,
                                                           bool /*useConstraintHessian*/)
    : LinearFunctionConstraints(),
      ZmpTaskWrapper(objectiveHandler),
      finalMaxState_(finalMaxState),
      enableInequalityConstraints_(enableInequalityConstraints),
      eqMatQP_(),
      ineqMatQP_(),
      eqConstraintIdxQP_(0u),
      ineqConstraintIdxQP_(0u),
      deviationEpsilonStatesCounter_(0u),
      zmpEpsilonStatesCounter_(0u) {
  nEqualityConstraints_ = 0;
  nInequalityConstraints_ = 0;
}

bool LinearZmpFunctionConstraints::initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) {
  if (!ZmpTaskWrapper::initialize(zmpInfo, motionPlan)) {
    return false;
  }
  const double infinity = std::numeric_limits<double>::max();

  // Copy motion plan.
  nEqualityConstraints_ = zmpInfo.numOfEqualityConstraintsQP_;
  nInequalityConstraints_ = zmpInfo.numOfInequalityConstraintsQP_;

  // Initialize indexes and counters.
  eqConstraintIdxQP_ = 0u;
  ineqConstraintIdxQP_ = 0u;
  zmpEpsilonStatesCounter_ = 0u;
  deviationEpsilonStatesCounter_ = 0u;

  // Initialize Constraints.
  eqMatQP_.setZero(nEqualityConstraints_, numOfUnknowns_);
  eqConTargetValues_.setZero(nEqualityConstraints_);

  ineqMatQP_.setZero(nInequalityConstraints_, numOfUnknowns_);
  ineqConMinValues_.setConstant(nInequalityConstraints_, -infinity);
  ineqConMaxValues_.setConstant(nInequalityConstraints_, infinity);

  // Initialize bounded vectors (assume infinite bounds by default).
  globalMinBounds_.setConstant(numOfUnknowns_, -infinity);
  globalMaxBounds_.setConstant(numOfUnknowns_, infinity);

  //! Compute QP matrices.
  if (!computeContinuousTimeQPMatrices()) {
    return false;
  }
  if (!computeDiscreteTimeQPMatrices()) {
    return false;
  }
  if (!computeGlobalBoundConstraintMaxValues()) {
    return false;
  }

  // Check if constraints are set proper.
  if (nEqualityConstraints_ != static_cast<int>(eqConstraintIdxQP_)) {
    MELO_FATAL_STREAM("[LinearZmpFunctionConstraints::initializeParameters] Wrong number of equality constraints.")
    return false;
  }
  if (nInequalityConstraints_ != static_cast<int>(ineqConstraintIdxQP_)) {
    MELO_FATAL_STREAM("[LinearZmpFunctionConstraints::initializeParameters] Wrong number of inequality constraints.")
    return false;
  }

  // Set constraints Jacobian and convert them to sparse view.
  globalEqConJacobian_ = eqMatQP_.sparseView();
  globalIneqConJacobian_ = ineqMatQP_.sparseView();

  return true;
}

bool LinearZmpFunctionConstraints::addHardInitialAndFinalConstraintsQP(unsigned int splineId, unsigned int coeffId, Derivative derivative,
                                                                       CogDim dim) {
  bool success = true;

  // Hard initial conditions.
  if (splineId == 0u && !(derivative == Derivative::Second && skipInitialAccelConditionsHard_)) {
    if (!isFirstSupportFlightPhase_) {
      objectiveHandler_.getInitialStateEqualityConstraints(eqMatQP_.block<1u, splineCoeffs>(eqConstraintIdxQP_, coeffId), derivative);
      eqConTargetValues_(eqConstraintIdxQP_) = initialRobotState_->operator[](derivative)[dim];
    }

    // Special case for flight phases
    else {
      success &= objectiveHandler_.getInitialFlightPhaseStateEqualityConstraints(
          eqMatQP_.block<1u, splineCoeffs>(eqConstraintIdxQP_, coeffId), eqConTargetValues_(eqConstraintIdxQP_),
          supportPolygons_->front().getDuration(), dim, derivative, initialRobotState_->operator[](Derivative::Zero)[dim],
          initialRobotState_->operator[](Derivative::First)[dim]);
    }
    ++eqConstraintIdxQP_;
  }

  /*
   * Hard final Conditions.
   * Note: we do not set final conditions for acceleration. Since final position (and velocity) are
   * specified, a hard final constraint on the acceleration would define the final zmp location.
   * This location may not lay within the final support polygon.
   */
  if (splineId == splineInfoSequence_->size() - 1 && setHardFinalConstraints_ && derivative != Derivative::Second) {
    if (!isLastSupportFlightPhase_) {
      objectiveHandler_.getFinalStateEqualityConstraints(eqMatQP_.block<1u, splineCoeffs>(eqConstraintIdxQP_, coeffId), derivative);
      eqConTargetValues_(eqConstraintIdxQP_) = finalRobotState_->operator[](derivative)[dim];
    }

    // Special case for flight phases.
    else {
      success &= objectiveHandler_.getFinalFlightPhaseStateEqualityConstraints(
          eqMatQP_.block<1u, splineCoeffs>(eqConstraintIdxQP_, coeffId), eqConTargetValues_(eqConstraintIdxQP_),
          supportPolygons_->back().getDuration(), derivative, dim, finalRobotState_->operator[](Derivative::Zero)[dim],
          finalRobotState_->operator[](Derivative::First)[dim]);
    }
    ++eqConstraintIdxQP_;
  }

  return success;
}

bool LinearZmpFunctionConstraints::addJunctionConstraintsQP(const SplineInfo& splineInfo, double /*timeAtSplineEnd*/, unsigned int coeffId,
                                                            unsigned int nextCoeffId, Derivative derivative, CogDim dim) {
  /*
   * Interconnect two splines s.t. the transition is smooth (w.r.t. position, velocity,
   * and acceleration). Two splines might be interconnected through a full flight phase
   * in between.
   */

  bool success = true;

  if (splineInfoSequence_->size() > 1u && splineInfo.id_ < splineInfoSequence_->size() - 1) {
    // Skip acceleration conditions if required.
    if (!(derivative == Derivative::Second && splineInfo.skipJunctionAccelAtNextPhase_)) {
      // Transition between two splines that are connected with each other through a flight phase.
      if (splineInfo.flightDurationOfNextPhase_ > 0.0) {
        success &= objectiveHandler_.getJunctionConstraintsFlightPhase(
            eqMatQP_.block<1, splineCoeffs>(eqConstraintIdxQP_, coeffId), eqMatQP_.block<1, splineCoeffs>(eqConstraintIdxQP_, nextCoeffId),
            eqConTargetValues_(eqConstraintIdxQP_), splineInfo.flightDurationOfNextPhase_, derivative, dim);
        ++eqConstraintIdxQP_;

        // Also constrain lift-off acceleration to be equal to gravity acceleration.
        if (derivative == Derivative::Second) {
          eqMatQP_.block<1, splineCoeffs>(eqConstraintIdxQP_, coeffId) = objectiveHandler_.getTimeVectorF(derivative);
          eqConTargetValues_(eqConstraintIdxQP_) = objectiveHandler_.getGravityInPlaneFrame(dim);
          ++eqConstraintIdxQP_;
        }

      }
      // Enforce smooth transition between two adjacent splines.
      else {
        success &= objectiveHandler_.getJunctionConstraints(eqMatQP_.block<1, splineCoeffs>(eqConstraintIdxQP_, coeffId),
                                                            eqMatQP_.block<1, splineCoeffs>(eqConstraintIdxQP_, nextCoeffId),
                                                            eqConTargetValues_(eqConstraintIdxQP_), derivative);
        ++eqConstraintIdxQP_;
      }
    }
  }  // end if junction constraint

  return success;
}

bool LinearZmpFunctionConstraints::addFinalBoxConstraintsQP(unsigned int coeffId, unsigned int splineId, Derivative derivative,
                                                            CogDim dim) {
  /*
   * Final position is constraint to a box, centered at the desired final position.
   * This guarantees that the final point does not drift away as hard final equality
   * constraints are not specifed.
   */

  if (splineId != splineInfoSequence_->size() - 1) {
    return true;
  }

  bool success = true;

  // Hard Final State inequality constraints (Box constraint) for the position
  if (!setHardFinalConstraints_ && enableInequalityConstraints_[Ineq::FinalCogBox] && derivative == Derivative::Zero) {
    if (!isLastSupportFlightPhase_) {
      success &= objectiveHandler_.getFinalMaxStateInequalityConstraints(
          ineqMatQP_.block<2, splineCoeffs>(ineqConstraintIdxQP_, coeffId), ineqConMinValues_.segment<2>(ineqConstraintIdxQP_), derivative,
          finalRobotState_->operator[](derivative)[dim], finalMaxState_[dim]);
    }

    // Special case for flight phases
    else {
      success &= objectiveHandler_.getFinalFlightPhaseMaxStateInequalityConstraints(
          ineqMatQP_.block<2, splineCoeffs>(ineqConstraintIdxQP_, coeffId), ineqConMinValues_.segment<2>(ineqConstraintIdxQP_),
          finalRobotState_->operator[](derivative)[dim], supportPolygons_->back().getDuration(), derivative, dim,
          finalRobotState_->operator[](Derivative::Zero)[dim], finalRobotState_->operator[](Derivative::First)[dim], finalMaxState_[dim]);
    }
    ineqConstraintIdxQP_ += 2u;  // upper and lower bound
  }

  return success;
}

bool LinearZmpFunctionConstraints::addMinDeviationConstraintsQP(unsigned int coeffId, unsigned int splineId, double splineTime,
                                                                CogDim dim) {
  /*
   * Implementation of the constraints
   *  x-x_des <=  epsilon
   *  x-x_des >= -epsilon
   * The constraints can be written as
   *  -x+epsilon >= -x_des  (1)
   *   x+epsilon >= x_des   (2)
   * Coefficients (A: active, D: desired):
   *   xA, xD, yA, yD, zA, zD
   */

  // Reset.
  if (dim == objectiveHandler_.getOptimizationDofs().front()) {
    deviationEpsilonStatesCounter_ = 0u;
  }

  unsigned int coeffIdEpsilon;

  // Add epsilon constraints only for active gait.
  if (zmpParams_->getAddEpsilonStateMinDeviationActiveGait(dim) && !zmpParams_->getAddEpsilonStateMinDeviationDesiredGait(dim) &&
      zmpParams_->isSplineIdInActiveGait(splineId)) {
    coeffIdEpsilon = getFirstMinDeviationEpsilonIndex() + deviationEpsilonStatesCounter_;
    ++deviationEpsilonStatesCounter_;
  }

  // Add epsilon constraints only for desired gait.
  else if (!zmpParams_->getAddEpsilonStateMinDeviationActiveGait(dim) && zmpParams_->getAddEpsilonStateMinDeviationDesiredGait(dim) &&
           !zmpParams_->isSplineIdInActiveGait(splineId)) {
    coeffIdEpsilon = getFirstMinDeviationEpsilonIndex() + deviationEpsilonStatesCounter_;
    ++deviationEpsilonStatesCounter_;
  }

  // Add epsilon states for active and desired gait.
  else if (zmpParams_->getAddEpsilonStateMinDeviationActiveGait(dim) && zmpParams_->getAddEpsilonStateMinDeviationDesiredGait(dim)) {
    if (zmpParams_->isSplineIdInActiveGait(splineId)) {
      coeffIdEpsilon = getFirstMinDeviationEpsilonIndex() + deviationEpsilonStatesCounter_;
    } else {
      coeffIdEpsilon = getFirstMinDeviationEpsilonIndex() + deviationEpsilonStatesCounter_ + 1u;
    }
    deviationEpsilonStatesCounter_ += 2u;
  }
  // No epsilon constraints involved.
  else {
    return true;
  }

  // Safety check.
  if (deviationEpsilonStatesCounter_ > numDeviationEpsilonStates_) {
    MELO_WARN_STREAM("[LinearZmpFunctionConstraints::addMinDeviationConstraintsQP] Wrong number of epsilon states.")
    return false;
  }

  // Get desired position.
  const double desiredState = pathRegularizer_->getComStateInPlaneFrameAtTime(splineTime, dim, Derivative::Zero);

  // Upper bound (1).
  ineqMatQP_.block<1, splineCoeffs>(ineqConstraintIdxQP_, coeffId) = -objectiveHandler_.getTimeVector(Derivative::Zero);
  ineqMatQP_(ineqConstraintIdxQP_, coeffIdEpsilon) = 1.0;
  ineqConMinValues_(ineqConstraintIdxQP_) = -desiredState;
  ++ineqConstraintIdxQP_;

  // Lower bound (2).
  ineqMatQP_.block<1, splineCoeffs>(ineqConstraintIdxQP_, coeffId) = objectiveHandler_.getTimeVector(Derivative::Zero);
  ineqMatQP_(ineqConstraintIdxQP_, coeffIdEpsilon) = 1.0;
  ineqConMinValues_(ineqConstraintIdxQP_) = desiredState;
  ++ineqConstraintIdxQP_;

  return true;
}

bool LinearZmpFunctionConstraints::addFrictionConstraintsQP(unsigned int coeffStartId, unsigned int splineId) {
  bool success = true;
  /*
   * Friction constraints:
   *  (1) Contact wrench must be of type push (legs cannot pull on the ground)
   *  (2) Contact wrench must lay within friction cone
   *  (3) Contact wrench must be upper bounded
   */

  if (std_utils::containsEnums(objectiveHandler_.getOptimizationDofs(), optimizationTranslationalDofs)) {
    // (1) Push contact inequality constraints.
    // Note: This constraint is required only if we optimize for z! Otherwise, the inequality
    // n'p_ddot >= n'g will always be satisfied
    success &= objectiveHandler_.getPushContactConstraints(ineqMatQP_.block<1, coeffsTransl>(ineqConstraintIdxQP_, coeffStartId),
                                                           ineqConMinValues_(ineqConstraintIdxQP_));
    ++ineqConstraintIdxQP_;

    // (2) Friction pyramide.
    if (enableInequalityConstraints_[Ineq::ForceModel]) {
      const Eigen::Vector3d& contactNormal = objectiveHandler_.getPlaneNormalInPlaneFrame();
      Eigen::Matrix<double, 4, coeffsTransl> frictionPyramideInEqualityMat;
      Eigen::Matrix<double, 4, 1> frictionPyramideInequalityVecMin;

      success &= objectiveHandler_.getFrictionPyramideConstraints(ineqMatQP_.block<4, coeffsTransl>(ineqConstraintIdxQP_, coeffStartId),
                                                                  ineqConMinValues_.segment<4>(ineqConstraintIdxQP_), contactNormal);
      ineqConstraintIdxQP_ += 4u;

      // (3) Max normal contact force.
      success &= objectiveHandler_.getMaxNormalContactForceConstraints(
          ineqMatQP_.block<1, coeffsTransl>(ineqConstraintIdxQP_, coeffStartId), ineqConMinValues_(ineqConstraintIdxQP_), contactNormal,
          static_cast<unsigned int>(splineInfoSequence_->operator[](splineId).supportPolygon.getPolygonType()));
      ++ineqConstraintIdxQP_;
    }
  }

  return success;
}

bool LinearZmpFunctionConstraints::addZmpConstraintsQP(unsigned int coeffStartId, unsigned int /*splineId*/, double splineTime,
                                                       const robot_utils::geometry::Polygon::LineCoefficientList& lineCoefficients) {
  /*
   * Zmp constraints: Zmp sample must lay within a polygon that is spanned by all the
   * grounded legs at that time instance. If we do not optimize for the z-component,
   * these constraints are linear in the spline coefficients.
   */
  bool success = true;

  if (std_utils::consistsOfEnums(objectiveHandler_.getOptimizationDofs(), optimizationXYDofs)) {
    const unsigned int coeffIdEpsilon = getFirstZmpEpsilonIndex() + zmpEpsilonStatesCounter_;

    for (const auto& lineCoefficient : lineCoefficients) {
      // c(x) >= 0.
      success &= objectiveHandler_.getPlanerZmpInequalityConstraints(
          ineqMatQP_.block<1, 2 * splineCoeffs>(ineqConstraintIdxQP_, coeffStartId), ineqConMinValues_(ineqConstraintIdxQP_),
          pathRegularizer_->getComStateInPlaneFrameAtTime(splineTime, CogDim::z, Derivative::Zero), lineCoefficient);

      // c(x) >= -epsilon.
      if (zmpEpsilonStatesCounter_ < numZmpEspilonStates_) {
        ineqMatQP_(ineqConstraintIdxQP_, coeffIdEpsilon) = 1.0;
      }
      ++ineqConstraintIdxQP_;
    }

    if (zmpEpsilonStatesCounter_ < numZmpEspilonStates_) {
      ++zmpEpsilonStatesCounter_;
    }
  }

  return success;
}

bool LinearZmpFunctionConstraints::computeContinuousTimeQPMatrices() {
  if (splineInfoSequence_->empty()) {
    return false;
  }

  double timeAtSplineEnd = 0.0;
  objectiveHandler_.setTime(0.0);

  // Skip flight phase at start
  if (isFirstSupportFlightPhase_) {
    timeAtSplineEnd += supportPolygons_->front().getDuration();
  }

  for (const auto& splineInfo : *splineInfoSequence_) {
    timeAtSplineEnd += splineInfo.duration_;
    if (!objectiveHandler_.setActiveSplineId(splineInfo.id_, splineInfo.duration_)) {
      return false;
    }

    for (const auto& dim : objectiveHandler_.getOptimizationDofs()) {
      const unsigned int coeffId = objectiveHandler_.getCoeffStartIndex(splineInfo.id_, dim);
      const unsigned int nextCoeffId = objectiveHandler_.getCoeffStartIndex(splineInfo.id_ + 1u, dim);

      for (const auto derivative : std_utils::enum_iterator<zmp::Derivative>()) {
        if (!addHardInitialAndFinalConstraintsQP(splineInfo.id_, coeffId, derivative, dim)) {
          MELO_WARN_STREAM("[LinearZmpFunctionConstraints::computeContinuousTimeQPMatrices] Failed to add initial or final constraints!")
          return false;
        }

        if (!addJunctionConstraintsQP(splineInfo, timeAtSplineEnd, coeffId, nextCoeffId, derivative, dim)) {
          MELO_WARN_STREAM("[LinearZmpFunctionConstraints::computeContinuousTimeQPMatrices] Failed to add junction constraints!")
          return false;
        }

        if (!addFinalBoxConstraintsQP(coeffId, splineInfo.id_, derivative, dim)) {
          MELO_WARN_STREAM("[LinearZmpFunctionConstraints::computeContinuousTimeQPMatrices] Failed to add final inequality constraints!")
          return false;
        }

      }  // end for derivative
    }    // end for dim

    // skip flight phases
    if (splineInfo.flightDurationOfNextPhase_ > 0.0) {
      timeAtSplineEnd += splineInfo.flightDurationOfNextPhase_;
    }
  }  // end for all splines

  return true;
}

bool LinearZmpFunctionConstraints::computeDiscreteTimeQPMatrices() {
  if (splineInfoSequence_->empty()) {
    return false;
  }
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

    // Discretization (tk start at 0).
    for (const auto& tk : timeInstantsPerSplineId_->operator[](splineInfo.id_)) {
      const unsigned int coeffStartId = objectiveHandler_.getCoeffStartIndex(splineInfo.id_);

      // Sample Interval.
      objectiveHandler_.setTime(tk);
      const double splineTime = timeAtSplineStart + tk;

      for (const auto& dim : objectiveHandler_.getOptimizationDofs()) {
        const unsigned int coeffId = objectiveHandler_.getCoeffStartIndex(splineInfo.id_, dim);

        if (!addMinDeviationConstraintsQP(coeffId, splineInfo.id_, splineTime, dim)) {
          MELO_WARN_STREAM("[LinearZmpFunctionConstraints::computeDiscreteTimeQPMatrices] Failed to add min deviation constraints!")
          return false;
        }

      }  // end for dim

      getLineCoefficients(lineCoefficients, splineInfo);

      if (!addZmpConstraintsQP(coeffStartId, splineInfo.id_, splineTime, lineCoefficients)) {
        MELO_WARN_STREAM(
            "[LinearZmpFunctionConstraints::computeDiscreteTimeQPMatrices] Failed to add discrete-time zmp inequality constraints!")
        return false;
      }

      if (!addFrictionConstraintsQP(coeffStartId, splineInfo.id_)) {
        MELO_WARN_STREAM("[LinearZmpFunctionConstraints::computeDiscreteTimeQPMatrices] Failed to add discrete-time friction constraints!")
        return false;
      }
    }  // end for all sample points

    // Skip flight phases.
    timeAtSplineStart += splineInfo.duration_;
    if (splineInfo.flightDurationOfNextPhase_ > 0.0) {
      timeAtSplineStart += splineInfo.flightDurationOfNextPhase_;
    }

  }  // end for all splines

  return true;
}

bool LinearZmpFunctionConstraints::computeGlobalBoundConstraintMaxValues() {
  const unsigned int numTotalEpsilonStates = numOfEpsilonStates();

  // epsilon >= 0.0 (for avoiding numerical issues we implement epsilon>0).
  if (numTotalEpsilonStates > 0u) {
    globalMinBounds_.tail(numTotalEpsilonStates).setConstant(std::numeric_limits<double>::min());
  }
  return true;
}

}  // namespace zmp
