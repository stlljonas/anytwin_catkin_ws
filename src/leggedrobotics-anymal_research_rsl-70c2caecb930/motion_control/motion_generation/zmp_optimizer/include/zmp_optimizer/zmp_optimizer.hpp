/*
 * zmp_optimizer.hpp
 *
 *  Created on: Jan 26, 2017
 *      Author: Dario Bellicoso, Fabian Jenelten
 */

#pragma once

// motion generation utils
#include <motion_generation_utils/TrajectoryStateHandlerLineSearchLinearAngular.hpp>
#include <motion_generation_utils/motion_generation.hpp>

// zmp optimizer
#include <zmp_optimizer/SupportPolygon.hpp>

// std utils
#include "std_utils/std_utils.hpp"

namespace zmp {

using ComStateHandler = motion_generation::TrajectoryStateHandlerLineSearchLinearAngular;
using PathRegularizer = motion_generation::TrajectoryStateHandlerLinearAngular;

// Linear or quadratic objective of a QP.
CONSECUTIVE_ENUM(Objective, Lin, Quad)

// Enumeration of inequality constraints.
CONSECUTIVE_ENUM(Ineq, FinalCogBox, ForceModel)

struct SplineInfo {
  unsigned int id_;
  double duration_;
  double flightDurationOfNextPhase_;
  bool skipJunctionAccelAtNextPhase_;
  SupportPolygon supportPolygon;

  SplineInfo() : id_(0u), duration_(0.0), flightDurationOfNextPhase_(-1), skipJunctionAccelAtNextPhase_(false), supportPolygon() {}
};

//! Struct containing objects used for the zmp-optimization.
struct ZmpInfo {
  //! Vector containing disrete-time instances of the sampled time domain.
  std::vector<std::vector<double>> timeInstantsPerSplineId_;

  //! Helper vector for storing spline information.
  std::vector<zmp::SplineInfo> splineInfoSequence_;

  //! If true, initial conditions for acceleration are replaced by soft constraints.
  bool skipInitialAccelConditionsHard_;

  //! The total number of unknowns.
  unsigned int numOfUnknowns_;

  //! Number of linear equality constraints.
  unsigned int numOfEqualityConstraintsQP_;

  //! Number of linear inequality constraints.
  unsigned int numOfInequalityConstraintsQP_;

  //! Number of nonlinear equality constraints.
  unsigned int numOfEqualityConstraintsSQP_;

  //! Number of nonlinear inequality constraints.
  unsigned int numOfInequalityConstraintsSQP_;

  //! Number of states associated to minimize max overshoot.
  unsigned int numDeviationEpsilonStates_;

  //! Number of epsilon states for the zmp constraints c(x)<=epsilon.
  unsigned int numZmpEspilonStates_;

  ZmpInfo()
      : timeInstantsPerSplineId_(0),
        splineInfoSequence_(0),
        skipInitialAccelConditionsHard_(false),
        numOfUnknowns_(0u),
        numOfEqualityConstraintsQP_(0u),
        numOfInequalityConstraintsQP_(0u),
        numOfEqualityConstraintsSQP_(0u),
        numOfInequalityConstraintsSQP_(0u),
        numDeviationEpsilonStates_(0u),
        numZmpEspilonStates_(0u) {}

  void reset() {
    // Linear constraints.
    numOfEqualityConstraintsQP_ = 0u;
    numOfInequalityConstraintsQP_ = 0u;

    // Nonlinear constraints.
    numOfEqualityConstraintsSQP_ = 0u;
    numOfInequalityConstraintsSQP_ = 0u;

    // Tota states.
    numOfUnknowns_ = 0u;

    // Epsilon states.
    numZmpEspilonStates_ = 0u;
    numDeviationEpsilonStates_ = 0u;
  }
};

struct Box {
  //! Final state boundaries relative to desired final point.
  std_utils::EnumArray<CogDim, double> finalMaxStateBox_;

  //! Center of the box.
  motion_generation::Position positionPlaneToCenterInPlaneFrame_;

  Box(const std_utils::EnumArray<CogDim, double>& finalMaxStateBox, const motion_generation::Position& positionPlaneToCenterInPlaneFrame)
      : finalMaxStateBox_(finalMaxStateBox), positionPlaneToCenterInPlaneFrame_(positionPlaneToCenterInPlaneFrame) {}

  Box() : finalMaxStateBox_(), positionPlaneToCenterInPlaneFrame_() {}
};

} /* namespace zmp */
