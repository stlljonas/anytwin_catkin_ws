/*
 * TrajectoryStateHandlerLineSearchLinearAngular.hpp
 *
 *  Created on: 07.09, 2018
 *      Author: Fabian Jenelten
 *
 *                               ___ TrajectoryStateHandlerBase ___
 *                              /                 |                \
 *                             /                  |                 \
 *                            /  TrajectoryStateHandlerLinear       TrajectoryStateHandlerAngular
 *                           /                     \                       /
 *                          /                       \                     /
 *  TrajectoryStateHandlerLineSearch      TrajectoryStateHandlerLinearAngular
 *                   |                      |
 *                   |                      |
 *             TrajectoryStateHandlerLineSearchLinearAngular
 */

#pragma once

// motion generation utils
#include <motion_generation_utils/TrajectoryStateHandlerLineSearch.hpp>
#include <motion_generation_utils/TrajectoryStateHandlerLinearAngular.hpp>

namespace motion_generation {

class TrajectoryStateHandlerLineSearchLinearAngular:
    virtual public TrajectoryStateHandlerLineSearch,
    virtual public TrajectoryStateHandlerLinearAngular {
public:
  using Weight = Eigen::DiagonalMatrix<double, 3>;

  TrajectoryStateHandlerLineSearchLinearAngular();
  ~TrajectoryStateHandlerLineSearchLinearAngular() override = default;

  void computeGradientAndHessian(double& gradient, double& hessian, double dt) const override;
  double computeObjective(double dt) const override;

  double lineSearch(
      double containerTimeGuess,
      const Position& measuredPositionInPlaneFrame,
      const EulerAnglesZyx& measuredOrientationPlaneToBase,
      double minTime = -1.0,
      double maxTime = -1.0);

protected:

  //! Weighting matrix for line search.
  Weight LinearWeight_;
  Weight AngularWeight_;

  // Measured position in plane frame.
  Position measuredPositionInPlaneFrame_;

  // Measured orientation plane to base.
  EulerAnglesZyx measuredOrientationBaseToPlane_;

};

}
