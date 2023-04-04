/*
 * LinearZmpFunctionConstraints.hpp
 *
 *  Created on: 09.05, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// zmp optimizer
#include "zmp_optimizer/ZmpTaskWrapper.hpp"

// numerical optimization
#include "numopt_common/LinearFunctionConstraints.hpp"

namespace zmp {

class LinearZmpFunctionConstraints : public numopt_common::LinearFunctionConstraints, public ZmpTaskWrapper {
  /*
   * Implementation of the constraints
   *  c_in(x) >= 0
   *  c_eq(x) =  0
   */

 public:
  LinearZmpFunctionConstraints(ZmpOptimizerObjectiveHandler& objectiveHandler,
                               const std_utils::EnumArray<zmp::CogDim, double>& finalMaxState,
                               const std_utils::EnumArray<Ineq, bool>& enableInequalityConstraints, bool useConstraintHessian);

  ~LinearZmpFunctionConstraints() override = default;

  bool initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) override;

 protected:
  bool addHardInitialAndFinalConstraintsQP(unsigned int splineId, unsigned int coeffId, Derivative derivative, CogDim dim);

  bool addJunctionConstraintsQP(const SplineInfo& splineInfo, double timeAtSplineEnd, unsigned int coeffId, unsigned int nextCoeffId,
                                Derivative derivative, CogDim dim);

  bool addFinalBoxConstraintsQP(unsigned int coeffId, unsigned int splineId, Derivative derivative, CogDim dim);

  bool addMinDeviationConstraintsQP(unsigned int coeffId, unsigned int splineId, double splineTime, CogDim dim);

  bool addZmpConstraintsQP(unsigned int coeffStartId, unsigned int splineId, double splineTime,
                           const robot_utils::geometry::Polygon::LineCoefficientList& lineCoefficients);

  bool addFrictionConstraintsQP(unsigned int coeffStartId, unsigned int splineId);

  bool computeContinuousTimeQPMatrices();

  bool computeDiscreteTimeQPMatrices();

  bool computeGlobalBoundConstraintMaxValues();

  //! final state boundaries relative to desired final point.
  const std_utils::EnumArray<zmp::CogDim, double>& finalMaxState_;

  //! If ith element is true, then inequality constraints of type "i" are set.
  const std_utils::EnumArray<Ineq, bool>& enableInequalityConstraints_;

  //! Equality matrix of a QP/LP
  Eigen::MatrixXd eqMatQP_;

  //! Equality vector (target values) of a QP/LP.
  Eigen::MatrixXd ineqMatQP_;

  //! counter for linear equality constraints.
  unsigned int eqConstraintIdxQP_;

  //! counter for linear inequality constraints.
  unsigned int ineqConstraintIdxQP_;

  //! counter for epsilon states associated to minimize max overshoot.
  unsigned int deviationEpsilonStatesCounter_;

  //! counter for zmp constraints that are softened.
  unsigned int zmpEpsilonStatesCounter_;
};

}  // namespace zmp
