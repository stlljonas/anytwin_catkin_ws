/*
 * TrajectoryStateHandlerLineSearchLinear.hpp
 *
 *  Created on: 07.09, 2018
 *      Author: Fabian Jenelten
 *
 *                   TrajectoryStateHandlerBase
 *                      /                 \
 *                     /                   \
 *  TrajectoryStateHandlerLineSearch      TrajectoryStateHandlerLinear
 *                   |                      |
 *                   |                      |
 *             TrajectoryStateHandlerLineSearchLinear
 */

#pragma once

// motion generation utils
#include <motion_generation_utils/TrajectoryStateHandlerLineSearch.hpp>
#include <motion_generation_utils/TrajectoryStateHandlerLinear.hpp>

namespace motion_generation {

class TrajectoryStateHandlerLineSearchLinear:
    virtual public TrajectoryStateHandlerLineSearch,
    virtual public TrajectoryStateHandlerLinear {
public:
  using Weight = Eigen::DiagonalMatrix<double, 3>;

  TrajectoryStateHandlerLineSearchLinear();
  ~TrajectoryStateHandlerLineSearchLinear() override = default;

  void computeGradientAndHessian(double& gradient, double& hessian, double dt) const override;
  double computeObjective(double dt) const override;

  double lineSearch(
      double containerTimeGuess,
      const Position& measuredPositionInPlaneFrame,
      double minTime = -1.0,
      double maxTime = -1.0);

protected:

  //! Weighting matrix for line search.
  Weight LinearWeight_;

  // Measured position in plane frame.
  Position measuredPositionInPlaneFrame_;

};

} /* namespace motion_generation */
