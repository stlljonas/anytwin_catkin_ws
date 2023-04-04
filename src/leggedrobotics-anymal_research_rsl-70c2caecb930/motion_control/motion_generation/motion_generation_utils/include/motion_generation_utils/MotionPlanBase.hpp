/*
 * MotionPlanBase.hpp
 *
 *  Created on: 03.26, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// motion generation utils
#include "motion_generation_utils/typedefs.hpp"
#include "motion_generation_utils/motion_generation.hpp"
#include "motion_generation_utils/VirtualPlaneFrameBase.hpp"

namespace motion_generation {

class MotionPlanBase {
public:
  MotionPlanBase();
  virtual ~MotionPlanBase() = default;

  virtual bool initialize();

  const zmp::ZmpOptStateVector& getInitialRobotStateInPlaneFrame() const;
  const zmp::ZmpOptStateVector& getFinalRobotStateInPlaneFrame() const;

  void setOptimizationDofs(const std::vector<zmp::CogDim>& optimizationDofs);
  const std::vector<zmp::CogDim>& getOptimizationDofs() const;

  zmp::TerminationState getTerminationState() const;
  bool didOptimizationSucceeded() const;
  double getOptimizationHorizon() const;

  const VirtualPlaneFrameBase& getVirtualPlaneFrame() const;

  void setTerminationState(zmp::TerminationState terminationState);

  const Pose& getPosePreviousPlaneToWorld() const;

  bool getEnforceHardFinalConstraints() const;
  bool getSkipHardInitialAccelConstraints() const;
  void setEqualityConstraints(bool enforceHardFinalConstraints, bool skipHardInitialAccelConstraints);

protected:

  //! Vector that contains initial conditions.
  zmp::ZmpOptStateVector initialRobotStateInPlaneFrame_;

  //! Vector that contains final conditions (a std::vector of Eigen::Vectors).
  zmp::ZmpOptStateVector finalRobotStateInPlaneFrame_;

  //! Termination state of the optimization.
  zmp::TerminationState terminationState_;

  //! Optimization horizon in seconds.
  double optimizationHorizon_;

  //! The optimization is done in virtual plane frame.
  VirtualPlaneFrameBase virtualPlaneFrame_;

  //! Pose of the previous virtual plane w.r.t. the earth frame.
  Pose posePreviousPlaneToWorld_;

  //! True if, we require hard final constraints. Otherwise, final constraints will be soft.
  bool enforceHardFinalConstraints_;

  //! If true, acceleration initial constraints are soft. Otherwise, they may be hard
  // (They can be still soft to relax the optimization problem).
  bool skipHardInitialAccelConstraints_;

  //! Degrees of freedom which are optimized.
  std::vector<zmp::CogDim> optimizationDofs_;

  //! True if motion plan has been initialized.
  bool isInitialized_;
};

} /* namespace motion_generation */
