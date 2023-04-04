/*
 * TrajectoryStateHandlerLineSearchLinear.hpp
 *
 *  Created on: 05.08, 2017
 *      Author: Fabian Jenelten
 */

// motion generation
#include <motion_generation_utils/TrajectoryStateHandlerLineSearchLinear.hpp>


namespace motion_generation {

TrajectoryStateHandlerLineSearchLinear::TrajectoryStateHandlerLineSearchLinear():
    TrajectoryStateHandlerLineSearch(),
    TrajectoryStateHandlerLinear(),
    measuredPositionInPlaneFrame_()
{
  LinearWeight_.diagonal() = Eigen::Vector3d(1.0, 1.0, 1.0);
}

void TrajectoryStateHandlerLineSearchLinear::computeGradientAndHessian(double& gradient, double& hessian, double dt) const {
  /*
   * Minimize the objective
   *  Q||p(t)-p_meas||_2^2 = (p(t)-p_meas)'Q(p(t)-p_meas)
   * Using Newton method combined with backtracing line search algorithm. Q indicates a weighting matrix.
   * Differentiating and double differentiating w.r.t time of the objective yields Gradient and Hessian:
   *  Gradient: 2*(p-p_meas)'*Q*v
   *  Hessian:  2*v'Q*v + 2*(p-p_meas)*Q*a
   */

  const auto splinePosition     = getPositionPlaneToComInPlaneFrameAtTime(dt).toImplementation();
  const auto splineVelocity     = getLinearVelocityComInPlaneFrameAtTime(dt).toImplementation();
  const auto splineAcceleration = getLinearAccelerationComInPlaneFrameAtTime(dt).toImplementation();

  Eigen::Vector3d positionDifference = splinePosition-measuredPositionInPlaneFrame_.toImplementation();

  gradient = 2.0 * positionDifference.dot(LinearWeight_*splineVelocity);
  hessian  = 2.0 * (
      splineVelocity.dot(LinearWeight_*splineVelocity) +
      positionDifference.dot(LinearWeight_*splineAcceleration)
  );
}

double TrajectoryStateHandlerLineSearchLinear::computeObjective(double dt) const {
  const auto splinePosition = getPositionPlaneToComInPlaneFrameAtTime(dt);
  const auto positionDifference = (splinePosition-measuredPositionInPlaneFrame_).toImplementation();
  return positionDifference.dot(LinearWeight_*positionDifference);
}

double TrajectoryStateHandlerLineSearchLinear::lineSearch(
    double containerTimeGuess,
    const Position& measuredPositionInPlaneFrame,
    double minTime,
    double maxTime) {
  measuredPositionInPlaneFrame_ = measuredPositionInPlaneFrame;
  return TrajectoryStateHandlerLineSearch::lineSearch(containerTimeGuess, minTime, maxTime);
}

} /* namespace motion_generation */
