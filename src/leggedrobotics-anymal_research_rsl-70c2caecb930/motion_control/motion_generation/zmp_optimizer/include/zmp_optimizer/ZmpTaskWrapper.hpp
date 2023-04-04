/*
 * ZmpTaskWrapper.hpp
 *
 *  Created on: 04.07, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// zmp optimizer
#include "zmp_optimizer/MotionPlan.hpp"
#include "zmp_optimizer/ZmpOptimizerObjectiveHandler.hpp"
#include "zmp_optimizer/ZmpParameterHandler.hpp"
#include "zmp_optimizer/zmp_optimizer.hpp"

namespace zmp {

class ZmpTaskWrapper {
 public:
  ZmpTaskWrapper(ZmpOptimizerObjectiveHandler& objectiveHandler)
      : objectiveHandler_(objectiveHandler),
        pathRegularizer_(nullptr),
        supportPolygons_(nullptr),
        initialRobotState_(nullptr),
        finalRobotState_(nullptr),
        timeInstantsPerSplineId_(nullptr),
        setHardFinalConstraints_(false),
        splineInfoSequence_(nullptr),
        skipInitialAccelConditionsHard_(false),
        numOfUnknowns_(0u),
        isFirstSupportFlightPhase_(false),
        isLastSupportFlightPhase_(false),
        numDeviationEpsilonStates_(0u),
        numZmpEspilonStates_(0u),
        zmpParams_(nullptr) {}

  virtual ~ZmpTaskWrapper() = default;

  virtual bool initialize(const ZmpInfo& zmpInfo, const MotionPlan* const motionPlan) {
    pathRegularizer_ = &motionPlan->getPathRegularizerInPlaneFrame();
    supportPolygons_ = &motionPlan->getSupportPolygonsInPlaneFrame();
    initialRobotState_ = &motionPlan->getInitialRobotStateInPlaneFrame();
    finalRobotState_ = &motionPlan->getFinalRobotStateInPlaneFrame();
    timeInstantsPerSplineId_ = &zmpInfo.timeInstantsPerSplineId_;
    splineInfoSequence_ = &zmpInfo.splineInfoSequence_;
    setHardFinalConstraints_ = motionPlan->getEnforceHardFinalConstraints();
    skipInitialAccelConditionsHard_ = zmpInfo.skipInitialAccelConditionsHard_;
    numOfUnknowns_ = zmpInfo.numOfUnknowns_;
    isFirstSupportFlightPhase_ = (supportPolygons_->front().getPolygonType() == PolygonType::Empty);
    isLastSupportFlightPhase_ = (supportPolygons_->back().getPolygonType() == PolygonType::Empty);
    numDeviationEpsilonStates_ = zmpInfo.numDeviationEpsilonStates_;
    numZmpEspilonStates_ = zmpInfo.numZmpEspilonStates_;
    zmpParams_ = &motionPlan->getZmpParams();
    return true;
  }

  //! Get first index corresponding to min deviation states.
  unsigned int getFirstMinDeviationEpsilonIndex() {
    /*
     * Vector of unknowns consists of three parts:
     * 1) spline coefficients,
     * 2) epsilon states for minimizing deviation and
     * 3) epsilon states for minimizing zmp constraint violation.
     */
    return (numOfUnknowns_ - numZmpEspilonStates_ - numDeviationEpsilonStates_);
  }

  //! Get first index corresponding to zmp epsilon states.
  unsigned int getFirstZmpEpsilonIndex() {
    /*
     * Vector of unknowns consists of three parts:
     * 1) spline coefficients,
     * 2) epsilon states for minimizing deviation and
     * 3) epsilon states for minimizing zmp constraint violation.
     */
    return (numOfUnknowns_ - numZmpEspilonStates_);
  }

  //! Returns number of total epsilon states.
  unsigned int numOfEpsilonStates() { return (numZmpEspilonStates_ + numDeviationEpsilonStates_); }

  virtual void getLineCoefficients(robot_utils::geometry::Polygon::LineCoefficientList& lineCoefficients, const SplineInfo& splineInfo) {
    lineCoefficients = splineInfo.supportPolygon.getPolygon().getLineCoefficients();
  }

 protected:
  //! Object that computes objective and constraint matrices.
  ZmpOptimizerObjectiveHandler& objectiveHandler_;

  //! path regularizer containing desired trajectory for Com.
  const motion_generation::TrajectoryStateHandlerLinearAngular* pathRegularizer_;

  //! A list of support polygons defined by the gait pattern.
  const std::vector<SupportPolygon>* supportPolygons_;

  //! Vector that contains initial conditions.
  const ZmpOptStateVector* initialRobotState_;

  //! Vector that contains final conditions.
  const ZmpOptStateVector* finalRobotState_;

  //! Vector containing discrete-time instances of the sampled time domain.
  const std::vector<std::vector<double>>* timeInstantsPerSplineId_;

  //! If true, final state constraints will be set as hard equality constraints.
  bool setHardFinalConstraints_;

  //! helper vector for storing spline information
  const std::vector<SplineInfo>* splineInfoSequence_;

  //! If true, initial conditions for acceleration are replaced by soft constraints.
  bool skipInitialAccelConditionsHard_;

  //! The total number of unknowns.
  unsigned int numOfUnknowns_;

  //! If true, first phase corresponds to a full flight phase.
  bool isFirstSupportFlightPhase_;

  //! If true, last phase corresponds to a full flight phase.
  bool isLastSupportFlightPhase_;

  //! number of state epsilon (for minimizing max deviations).
  unsigned int numDeviationEpsilonStates_;

  //! number of epsilon states for the zmp constraints c(x)<=epsilon.
  unsigned int numZmpEspilonStates_;

  //! Container for all gait-varying parameters.
  const loco::ZmpParameterHandler* zmpParams_;
};

}  // namespace zmp
