/*
 * QuadraticZmpObjectiveFunction.hpp
 *
 *  Created on: 09.05, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// zmp optimizer
#include "zmp_optimizer/ZmpTaskWrapper.hpp"

// numerical optimization
#include "numopt_common/QuadraticObjectiveFunction.hpp"

namespace zmp {

class QuadraticZmpObjectiveFunction : public numopt_common::QuadraticObjectiveFunction, public ZmpTaskWrapper {
  /*
   * Implementation of the objective function
   *  min_{x} f(x)
   */

 public:
  explicit QuadraticZmpObjectiveFunction(ZmpOptimizerObjectiveHandler& objectiveHandler, double hessianRegularizer);

  ~QuadraticZmpObjectiveFunction() override = default;

  bool initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) override;

 protected:
  bool addSoftInitialAndFinalConstraintsQP(unsigned int splineId, unsigned int coeffId, Derivative derivative, CogDim dim);

  bool addAccelerationObjectiveQP(unsigned int splineId, unsigned int coeffId, CogDim dim);

  bool addTrackingObjectivesQP(unsigned int splineId, unsigned int coeffId, double sampleTime, double splineTime,
                               double splineTimePreviousSolution, Derivative derivative, CogDim dim);

  bool addLegExtensionObjectivesQP(const SplineInfo& splineInfo, unsigned int coeffId, double sampleTime, double splineTime, CogDim dim);

  bool addJunctionObjectivesQP(const SplineInfo& splineInfo, double timeAtSplineEnd, unsigned int coeffId, unsigned int nextCoeffId,
                               Derivative derivative, CogDim dim);

  bool addMinDeviationObjective();

  bool addMinZmpRelaxationBoundsObjective();

  bool computeContinuousTimeQPMatrices();

  bool computeDiscreteTimeQPMatrices();

  //! Previous solution in spline representation.
  const ComStateHandler* previousMotionPlanInPreviousPlaneFrame_;

  //! Offset target point to limb thigh.
  const motion_generation::anymalLegsVector* vectorsTargetToLimbThighInBaseFrame_;

  //! A regularizer used in the cost function.
  double hessianRegularizer_;

  //! Hessian of a quadratic program.
  Eigen::MatrixXd hessian_;

  //! Pose of the previous virtual plane w.r.t. the earth frame.
  const motion_generation::Pose* posePreviousPlaneToWorld_;

  //! Pose of the virtual plane w.r.t. the earth frame.
  const motion_generation::Pose* posePlaneToWorld_;

  //! Position error norm between grounded end-effector and desired foothold.
  const motion_generation::anymalLegsDouble* footholdTrackingOffsetIndicator_;

  //! Current footprint pattern.
  const motion_generation::anymalLegsVector* positionPlaneToEndEffectorInPlaneFrame_;

  //! Desired limb thigh over foothold.
  Eigen::Vector3d vectorFootholdToLimbThighInPlaneFrame_;
};

}  // namespace zmp
