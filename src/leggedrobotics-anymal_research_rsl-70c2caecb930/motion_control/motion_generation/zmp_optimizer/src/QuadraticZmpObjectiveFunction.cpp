/*
 * QuadraticZmpObjectiveFunction.cpp
 *
 *  Created on: 03.26, 2017
 *      Author: Fabian Jenelten
 */

// anymal_description
#include <anymal_description/LegEnum.hpp>

// zmp optimizer
#include <zmp_optimizer/QuadraticZmpObjectiveFunction.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace zmp {

QuadraticZmpObjectiveFunction::QuadraticZmpObjectiveFunction(ZmpOptimizerObjectiveHandler& objectiveHandler, double hessianRegularizer)
    : QuadraticObjectiveFunction(),
      ZmpTaskWrapper(objectiveHandler),
      previousMotionPlanInPreviousPlaneFrame_(nullptr),
      vectorsTargetToLimbThighInBaseFrame_(nullptr),
      hessianRegularizer_(hessianRegularizer),
      hessian_(),
      posePreviousPlaneToWorld_(nullptr),
      posePlaneToWorld_(nullptr),
      footholdTrackingOffsetIndicator_(nullptr),
      positionPlaneToEndEffectorInPlaneFrame_(nullptr),
      vectorFootholdToLimbThighInPlaneFrame_(Eigen::Vector3d::Zero()) {}

bool QuadraticZmpObjectiveFunction::initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) {
  if (!ZmpTaskWrapper::initialize(zmpInfo, motionPlan)) {
    return false;
  }

  // Copy motion plan.
  previousMotionPlanInPreviousPlaneFrame_ = &motionPlan->getComStateInPlaneFrame();
  posePreviousPlaneToWorld_ = &motionPlan->getPosePreviousPlaneToWorld();
  posePlaneToWorld_ = &motionPlan->getVirtualPlaneFrame().getPosePlaneToWorld();
  vectorsTargetToLimbThighInBaseFrame_ = &motionPlan->getVectorsTargetToLimbThighInBaseFrame();
  footholdTrackingOffsetIndicator_ = &motionPlan->getFootholdTrackingOffsetIndicator();
  positionPlaneToEndEffectorInPlaneFrame_ = &motionPlan->getPositionPlaneToEndEffectorInPlaneFrame();

  // Default limb position.
  const Eigen::Vector3d planeNormalInPlaneFrame = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d worldZNormalInPlaneFrame = posePlaneToWorld_->getRotation().inverseRotate(planeNormalInPlaneFrame);
  vectorFootholdToLimbThighInPlaneFrame_ =
      (motionPlan->getGravityFactor() * worldZNormalInPlaneFrame + (1.0 - motionPlan->getGravityFactor()) * planeNormalInPlaneFrame) *
      motionPlan->getNominalLegExtension();

  // Initialize objective.
  hessian_.setZero(numOfUnknowns_, numOfUnknowns_);
  linearTerm_.setZero(numOfUnknowns_);

  // Compute QP matrices.
  if (!computeContinuousTimeQPMatrices()) {
    return false;
  }
  if (!computeDiscreteTimeQPMatrices()) {
    return false;
  }

  /*
   * Note: So far, we have only stored the lower part (because of numerical issues rather than
   * efficiency reasoning). Hence, copy lower part to upper triangular part.
   */
  const Eigen::MatrixXd fullHessianQP = hessian_.selfadjointView<Eigen::Lower>();
  hessian_ = fullHessianQP;

  // Convert to sparse view.
  globalHessian_ = hessian_.sparseView();

  return true;
}

bool QuadraticZmpObjectiveFunction::addSoftInitialAndFinalConstraintsQP(unsigned int splineId, unsigned int coeffId, Derivative derivative,
                                                                        CogDim dim) {
  bool success = true;

  // Soft Initial constraints: Approach acceleration as close as possible to previous acceleration.
  if (splineId == 0u && derivative == Derivative::Second && skipInitialAccelConditionsHard_ &&
      zmpParams_->getParamsBySpline(static_cast<int>(splineId)).weightsInitialState_[Derivative::Second] > 0.0) {
    if (!isFirstSupportFlightPhase_) {
      success &= objectiveHandler_.getInitialStateObjective(
          hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId), linearTerm_.segment<splineCoeffs>(coeffId),
          initialRobotState_->operator[](derivative)[dim], derivative,
          zmpParams_->getParamsBySpline(static_cast<int>(splineId)).weightsInitialState_[derivative]);
    }

    else {
      success &= objectiveHandler_.getInitialFlightPhaseStateObjective(
          hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId), linearTerm_.segment<splineCoeffs>(coeffId),
          zmpParams_->getParamsBySpline(static_cast<int>(splineId)).weightsInitialState_[derivative],
          supportPolygons_->front().getDuration(), derivative, dim, initialRobotState_->operator[](Derivative::Zero)[dim],
          initialRobotState_->operator[](Derivative::First)[dim]);
    }
  }

  // Soft final constraints: Approach end point as close as possible to the desired final point
  if (splineId == (splineInfoSequence_->size() - 1) && !setHardFinalConstraints_ &&
      zmpParams_->getParamsBySpline(static_cast<int>(splineId)).weightsFinalState_[derivative] > 0.0) {
    // Last polygon is not a flight phase.
    if (!isLastSupportFlightPhase_) {
      success &= objectiveHandler_.getFinalStateObjective(
          hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId), linearTerm_.segment<splineCoeffs>(coeffId),
          finalRobotState_->operator[](derivative)[dim], derivative,
          zmpParams_->getParamsBySpline(static_cast<int>(splineId)).weightsFinalState_[derivative]);
    }

    // Special case for flight phases.
    else {
      success &= objectiveHandler_.getFinalFlightPhaseStateObjective(
          hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId), linearTerm_.segment<splineCoeffs>(coeffId),
          zmpParams_->getParamsBySpline(static_cast<int>(splineId)).weightsFinalState_[derivative], supportPolygons_->back().getDuration(),
          derivative, dim, finalRobotState_->operator[](Derivative::Zero)[dim], finalRobotState_->operator[](Derivative::First)[dim]);
    }
  }

  return success;
}

bool QuadraticZmpObjectiveFunction::addAccelerationObjectiveQP(unsigned int splineId, unsigned int coeffId, CogDim dim) {
  /*
   * Minimize acceleration over entire spline. Since the desired state is known a-priori
   * (zero), we can find an analytical expression of the Hessian for the entire spline
   * (i.e., we do not need to discretize the spline).
   */
  bool success = true;

  if (zmpParams_->getParamsBySpline(static_cast<int>(splineId)).weightsPathRegularizer_[dim][Derivative::Second] > 0.0) {
    success &= objectiveHandler_.getAccelerationObjective(
        hessian_.block<splineCoeffs - 2, splineCoeffs - 2>(coeffId, coeffId),
        zmpParams_->getParamsBySpline(static_cast<int>(splineId)).weightsPathRegularizer_[dim][Derivative::Second]);

    // Regularization.
    if (hessianRegularizer_ > 0.0) {
      hessian_.block<2, 2>(coeffId + 4, coeffId + 4) += Eigen::Matrix2d::Identity() * hessianRegularizer_;
    }
  }

  // Pure Regularization.
  else if (hessianRegularizer_ > 0.0) {
    hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId) +=
        Eigen::Matrix<double, splineCoeffs, splineCoeffs>::Identity() * hessianRegularizer_;
  }

  return success;
}

bool QuadraticZmpObjectiveFunction::addLegExtensionObjectivesQP(const SplineInfo& splineInfo, unsigned int coeffId, double sampleTime,
                                                                double splineTime, CogDim dim) {
  if (zmp::isTranslation(dim)) {
    const auto& zmpParams = zmpParams_->getParamsBySpline(static_cast<int>(splineInfo.id_));
    const auto orientationBaseToPlane = pathRegularizer_->getAnglesZyxBaseToPlaneAtTime(splineTime);
    const auto dimId = zmp::toIndex(dim);

    for (const auto& legEnum : anymal_description::LegEnumIterator()) {
      const Eigen::Vector3d vectorTargetToLimbThighInPlaneFrame =
          orientationBaseToPlane.rotate(vectorsTargetToLimbThighInBaseFrame_->operator[](legEnum));

      // Enforce default leg configuration.
      if (zmpParams.weightsDefaultLegConfig_[dim] > 0.0) {
        if (splineInfo.supportPolygon.isStanceLeg(legEnum)) {
          if (!objectiveHandler_.getStateObjective(
                  hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId), linearTerm_.segment<splineCoeffs>(coeffId),
                  splineInfo.supportPolygon.getPositionPlaneToFootInPlaneFrame(legEnum)[dimId] +
                      vectorFootholdToLimbThighInPlaneFrame_[dimId] - vectorTargetToLimbThighInPlaneFrame[dimId],
                  zmp::Derivative::Zero, zmpParams.weightsDefaultLegConfig_[dim] * sampleTime)) {
            return false;
          }
        }
      }

      // Minimize leg over extension.
      if (zmpParams.weightsMinLegExtension_[dim] > 0.0) {
        if (splineInfo.supportPolygon.isStanceLeg(legEnum)) {
          if (!objectiveHandler_.getStateObjective(
                  hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId), linearTerm_.segment<splineCoeffs>(coeffId),
                  splineInfo.supportPolygon.getPositionPlaneToFootInPlaneFrame(legEnum)[dimId] - vectorTargetToLimbThighInPlaneFrame[dimId],
                  zmp::Derivative::Zero, zmpParams.weightsMinLegExtension_[dim] * sampleTime)) {
            return false;
          }
        }
      }

      // Avoid torso to tip-over (approach limb-thigh towards legs that are stuck, i.e., that have a bad tracking).
      if (zmpParams.weightsTipOverAvoidance_[dim] > 0.0) {
        if (!objectiveHandler_.getStateObjective(
                hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId), linearTerm_.segment<splineCoeffs>(coeffId),
                positionPlaneToEndEffectorInPlaneFrame_->operator[](legEnum)[dimId] + vectorFootholdToLimbThighInPlaneFrame_[dimId] -
                    vectorTargetToLimbThighInPlaneFrame[dimId],
                zmp::Derivative::Zero,
                zmpParams.weightsTipOverAvoidance_[dim] * footholdTrackingOffsetIndicator_->operator[](legEnum) * sampleTime)) {
          return false;
        }
      }
    }
  }

  return true;
}

bool QuadraticZmpObjectiveFunction::addTrackingObjectivesQP(unsigned int splineId, unsigned int coeffId, double sampleTime,
                                                            double splineTime, double splineTimePreviousSolution, Derivative derivative,
                                                            CogDim dim) {
  /*
   * Minimize the area between the solution and the path regularizer, aswell as between
   * the solution and the previous solution.
   */

  bool success = true;
  const auto& zmpParams = zmpParams_->getParamsBySpline(static_cast<int>(splineId));

  // Approach solution to path regularizer (acceleration is minimized to zero, hence we skip acceleration here)
  if (zmpParams.weightsPathRegularizer_[dim][derivative] > 0.0 && derivative != Derivative::Second) {
    success &= objectiveHandler_.getStateObjective(hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId),
                                                   linearTerm_.segment<splineCoeffs>(coeffId),
                                                   pathRegularizer_->getComStateInPlaneFrameAtTime(splineTime, dim, derivative), derivative,
                                                   zmpParams.weightsPathRegularizer_[dim][derivative] * sampleTime);
  }

  // Approach solution to previous solution
  if (splineTimePreviousSolution < previousMotionPlanInPreviousPlaneFrame_->getMotionPlan()[dim].getContainerDuration() &&
      zmpParams.weightsPreviousSolution_[derivative] > 0.0) {
    // Translational part.
    Eigen::Vector3d desiredState;
    if (zmp::isTranslation(dim)) {
      switch (derivative) {
        case Derivative::Zero: {
          const auto positionWorldToTargetInWorldFrame = posePreviousPlaneToWorld_->transform(
              previousMotionPlanInPreviousPlaneFrame_->getPositionPlaneToComInPlaneFrameAtTime(splineTimePreviousSolution));
          desiredState = posePlaneToWorld_->inverseTransform(positionWorldToTargetInWorldFrame).toImplementation();
        } break;

        case Derivative::First: {
          const auto velocityTargetInWorldFrame = posePreviousPlaneToWorld_->getRotation().rotate(
              previousMotionPlanInPreviousPlaneFrame_->getLinearVelocityComInPlaneFrameAtTime(splineTimePreviousSolution));
          desiredState = posePlaneToWorld_->getRotation().inverseRotate(velocityTargetInWorldFrame).toImplementation();
        } break;

        default: { return false; }
      }
    }

    // Rotational part.
    else if (zmp::isRotation(dim)) {
      const auto orientationBaseToPlane =
          (motion_generation::EulerAnglesZyx(posePlaneToWorld_->getRotation().inverted()) *
           motion_generation::EulerAnglesZyx(posePreviousPlaneToWorld_->getRotation()) *
           previousMotionPlanInPreviousPlaneFrame_->getAnglesZyxBaseToPlaneAtTime(splineTimePreviousSolution))
              .getUnique();

      switch (derivative) {
        case Derivative::Zero: {
          desiredState = orientationBaseToPlane.toImplementation();
        } break;

        case Derivative::First: {
          const auto angularVelocityInWorldFrame = posePreviousPlaneToWorld_->getRotation().rotate(
              previousMotionPlanInPreviousPlaneFrame_->getAngularVelocityBaseInPlaneFrameAtTime(splineTimePreviousSolution));
          const auto angularVelocityInPlaneFrame =
              posePlaneToWorld_->getRotation().inverseRotate(angularVelocityInWorldFrame).toImplementation();
          desiredState =
              zmp::getMatrixAngularVelocityInInertialFrameToAngularRatesZyx(orientationBaseToPlane) * angularVelocityInPlaneFrame;
        } break;

        default: { return false; }
      }
    }

    else {
      MELO_WARN_STREAM("[QuadraticZmpObjectiveFunction::addTrackingObjectivesQP] Unknown dim.")
      return false;
    }

    success &= objectiveHandler_.getStateObjective(hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId),
                                                   linearTerm_.segment<splineCoeffs>(coeffId), desiredState(toIndex(dim)), derivative,
                                                   zmpParams.weightsPreviousSolution_[derivative] * sampleTime);
  }

  return success;
}

bool QuadraticZmpObjectiveFunction::addJunctionObjectivesQP(const SplineInfo& splineInfo, double timeAtSplineEnd, unsigned int coeffId,
                                                            unsigned int nextCoeffId, Derivative derivative, CogDim dim) {
  /*
   * Special treatment for flight phases. For position and velocity we track the path regularizer
   * at touch down while the acceleration is minimized to zero. The overshoot of the height
   * trajectory can be minimized by minimizing the z-velocity at lift off.
   */
  bool success = true;

  if (splineInfo.flightDurationOfNextPhase_ > 0.0) {
    const auto& zmpParams = zmpParams_->getParamsBySpline(static_cast<int>(splineInfo.id_));

    // Track path regularizer at lift-off and touch-down events.
    if (zmpParams.weightsTouchDownFlightTrajectory_[dim][derivative] > 0.0) {
      // At lift-off state.
      if (timeAtSplineEnd < pathRegularizer_->getContainerDuration()) {
        success &= objectiveHandler_.getFinalStateObjective(
            hessian_.block<splineCoeffs, splineCoeffs>(coeffId, coeffId), linearTerm_.segment<splineCoeffs>(coeffId),
            pathRegularizer_->getComStateInPlaneFrameAtTime(timeAtSplineEnd, dim, derivative), derivative,
            zmpParams.weightsTouchDownFlightTrajectory_[dim][derivative] * splineInfo.flightDurationOfNextPhase_);
      }

      // At touch-down state.
      if (splineInfo.id_ < splineInfoSequence_->size() - 1) {
        const double splineTime = timeAtSplineEnd + splineInfo.flightDurationOfNextPhase_;

        if (splineTime < pathRegularizer_->getContainerDuration()) {
          success &= objectiveHandler_.getInitialStateObjective(
              hessian_.block<splineCoeffs, splineCoeffs>(nextCoeffId, nextCoeffId), linearTerm_.segment<splineCoeffs>(nextCoeffId),
              pathRegularizer_->getComStateInPlaneFrameAtTime(splineTime, dim, derivative), derivative,
              zmpParams.weightsTouchDownFlightTrajectory_[dim][derivative] * splineInfo.flightDurationOfNextPhase_);
        }
      }
    }
  }

  return success;
}

bool QuadraticZmpObjectiveFunction::addMinDeviationObjective() {
  /*
   * Epsilon are associated to the largest overshoot of the trajectory. Objective:
   *    min {lambda*epsilon * kappa*epsilon^2}
   * The linear term penalizes violations of the constraints and the
   * quadratic term is added to render the cost smooth in
   */
  if (numDeviationEpsilonStates_ == 0u) {
    return true;
  }

  unsigned int coeffIdEpsilon = getFirstMinDeviationEpsilonIndex();

  for (const auto& dim : objectiveHandler_.getOptimizationDofs()) {
    // Epsilon states for active gait.
    if (zmpParams_->getAddEpsilonStateMinDeviationActiveGait(dim)) {
      const auto& weightMinDeviation = zmpParams_->getActiveParams().weightsMinDeviation_[dim];
      linearTerm_(coeffIdEpsilon) = std::fmax(weightMinDeviation[Objective::Lin], 0.0);
      hessian_(coeffIdEpsilon, coeffIdEpsilon) = std::fmax(2.0 * weightMinDeviation[Objective::Quad], hessianRegularizer_);
      ++coeffIdEpsilon;
    }

    // Epsilon states for desired gait.
    if (zmpParams_->getAddEpsilonStateMinDeviationDesiredGait(dim)) {
      const auto& weightMinDeviation = zmpParams_->getDesiredParams().weightsMinDeviation_[dim];
      linearTerm_(coeffIdEpsilon) = std::fmax(weightMinDeviation[Objective::Lin], 0.0);
      hessian_(coeffIdEpsilon, coeffIdEpsilon) = std::fmax(2.0 * weightMinDeviation[Objective::Quad], hessianRegularizer_);
      ++coeffIdEpsilon;
    }
  }

  if (coeffIdEpsilon != (getFirstMinDeviationEpsilonIndex() + numDeviationEpsilonStates_)) {
    MELO_WARN_STREAM("[QuadraticZmpObjectiveFunction::addMinDeviationObjective] Wrong number of deviation epsilon states!")
    return false;
  }

  return true;
}

bool QuadraticZmpObjectiveFunction::addMinZmpRelaxationBoundsObjective() {
  /*
   * Soft zmp inequality constraints
   *    c(x) <= epsilon
   * with
   *    min {lambda*epsilon + kappa*epsilon^2}
   * The linear term penalizes violations of the constraints and the
   * quadratic term is added to render the cost smooth in epsilon.
   * Note:
   *  > We use the parameters of the active gait only (this is tractable since
   *    since this parameter does not have a large impact on the solution).
   *  > Each spline has its dedicated epsilon state.
   */

  const auto& weightZmpRelaxation = zmpParams_->getActiveParams().weightZmpRelaxation_;
  const double linTerm = std::fmax(weightZmpRelaxation[Objective::Lin], 0.0);
  const double quadTerm = std::fmax(2.0 * weightZmpRelaxation[Objective::Quad], hessianRegularizer_);

  for (unsigned int epsilonId = 0u; epsilonId < numZmpEspilonStates_; ++epsilonId) {
    const unsigned int coeffIdEpsilon = getFirstZmpEpsilonIndex() + epsilonId;
    linearTerm_(coeffIdEpsilon) = linTerm;
    hessian_(coeffIdEpsilon, coeffIdEpsilon) = quadTerm;
  }

  return true;
}

bool QuadraticZmpObjectiveFunction::computeContinuousTimeQPMatrices() {
  if (splineInfoSequence_->empty()) {
    return false;
  }
  double timeAtSplineEnd = 0.0;

  // Skip flight phase at start.
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
      const unsigned int nextCoeffId = objectiveHandler_.getCoeffStartIndex(splineInfo.id_ + 1, dim);

      for (const auto& derivative : std_utils::enum_iterator<zmp::Derivative>()) {
        if (!addSoftInitialAndFinalConstraintsQP(splineInfo.id_, coeffId, derivative, dim)) {
          MELO_WARN_STREAM("[QuadraticZmpObjectiveFunction::computeContinuousTimeQPMatrices] Failed to add initial or final constraints!")
          return false;
        }

        if (!addJunctionObjectivesQP(splineInfo, timeAtSplineEnd, coeffId, nextCoeffId, derivative, dim)) {
          MELO_WARN_STREAM("[QuadraticZmpObjectiveFunction::computeContinuousTimeQPMatrices] Failed to add junction objectives!")
          return false;
        }

      }  // end for derivative

      if (!addAccelerationObjectiveQP(splineInfo.id_, coeffId, dim)) {
        MELO_WARN_STREAM("[QuadraticZmpObjectiveFunction::computeContinuousTimeQPMatrices] Failed to add acceleration objective!")
        return false;
      }
    }  // end for dim

    // Skip next flight phases.
    if (splineInfo.flightDurationOfNextPhase_ > 0.0) {
      timeAtSplineEnd += splineInfo.flightDurationOfNextPhase_;
    }

  }  // end for all splines

  if (!addMinDeviationObjective()) {
    MELO_WARN_STREAM("[QuadraticZmpObjectiveFunction::computeContinuousTimeQPMatrices] Failed to add min deviation objective!")
    return false;
  }

  if (!addMinZmpRelaxationBoundsObjective()) {
    MELO_WARN_STREAM("[QuadraticZmpObjectiveFunction::computeContinuousTimeQPMatrices] Failed to add min zmp relaxation objective!")
    return false;
  }

  return true;
}

bool QuadraticZmpObjectiveFunction::computeDiscreteTimeQPMatrices() {
  if (splineInfoSequence_->empty()) {
    return false;
  }

  double timeAtSplineStart = 0.0;
  double previousSplineTime = 0.0;

  // Skip flight phase at start.
  if (isFirstSupportFlightPhase_) {
    timeAtSplineStart += supportPolygons_->front().getDuration();
  }

  for (const auto& splineInfo : *splineInfoSequence_) {
    // Skip splines that were not accessed a sample time.
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

    // Discretization (tk starts at 0).
    for (auto tk : timeInstantsPerSplineId_->operator[](splineInfo.id_)) {
      // Sample Interval.
      objectiveHandler_.setTime(tk);
      const double splineTime = timeAtSplineStart + tk;
      const double sampleTime = splineTime - previousSplineTime;
      previousSplineTime = splineTime;
      if (sampleTime < 0.0) {
        return false;
      }

      for (const auto& dim : objectiveHandler_.getOptimizationDofs()) {
        const double splineTimePreviousSolution =
            splineTime + previousMotionPlanInPreviousPlaneFrame_->getMotionPlan()[dim].getContainerTime();
        const unsigned int coeffId = objectiveHandler_.getCoeffStartIndex(splineInfo.id_, dim);

        for (const auto& derivative : std_utils::enum_iterator<zmp::Derivative>()) {
          if (!addTrackingObjectivesQP(splineInfo.id_, coeffId, sampleTime, splineTime, splineTimePreviousSolution, derivative, dim)) {
            std::cout << "QuadraticZmpObjectiveFunction::computeDiscreteTimeQPMatrices: Failed to add discrete-time tracking objectives!\n";
            return false;
          }
        }  // end for derivative

        if (!addLegExtensionObjectivesQP(splineInfo, coeffId, sampleTime, splineTime, dim)) {
          return false;
        }

      }  // end for dim
    }    // end for all sample points

    // Skip flight phases.
    timeAtSplineStart += splineInfo.duration_;
    if (splineInfo.flightDurationOfNextPhase_ > 0.0) {
      timeAtSplineStart += splineInfo.flightDurationOfNextPhase_;
    }

  }  // end for all splines

  return true;
}

}  // namespace zmp
