/*
 * TrajectoryStateHandlerLineSearchLinearAngular.hpp
 *
 *  Created on: 05.08, 2017
 *      Author: Fabian Jenelten
 */

// motion generation
#include <motion_generation_utils/TrajectoryStateHandlerLineSearchLinearAngular.hpp>


namespace motion_generation {

TrajectoryStateHandlerLineSearchLinearAngular::TrajectoryStateHandlerLineSearchLinearAngular():
    TrajectoryStateHandlerLineSearch(),
    TrajectoryStateHandlerLinearAngular(),
    measuredPositionInPlaneFrame_(),
    measuredOrientationBaseToPlane_()
{
  LinearWeight_.diagonal() = Eigen::Vector3d(1.0, 1.0, 1.0);
  AngularWeight_.diagonal() = Eigen::Vector3d(0.9, 0.9, 0.9);
}

void TrajectoryStateHandlerLineSearchLinearAngular::computeGradientAndHessian(double& gradient, double& hessian, double dt) const {
  /*
   * Minimize the objective
   *  Qlin||p(t)-p_meas||_2^2 + Qrot||q(t)-q_meas||_2^2
   *      = (p(t)-p_meas)'Qlin(p(t)-p_meas) + (q(t)-q_meas)'Qrot(q(t)-q_meas)
   * Using Newton method combined with backtracing line search algorithm. Q indicates a weighting matrix.
   * Differentiating and double differentiating w.r.t time of the objective yields Gradient and Hessian:
   *  Gradient: 2*(p-p_meas)'*Qlin*v + 2*(q-q_meas)'*Qrot*qdot
   *  Hessian:  2*v'Qlin*v + 2*(p-p_meas)*Qlin*a + 2*qdot'Qrot*qdot + 2*(q-q_meas)*Qrot*qddot
   */

  const auto splinePosition          = getPositionPlaneToComInPlaneFrameAtTime(dt).toImplementation();
  const auto splineVelocity          = getLinearVelocityComInPlaneFrameAtTime(dt).toImplementation();
  const auto splineAcceleration      = getLinearAccelerationComInPlaneFrameAtTime(dt).toImplementation();
  const auto splineOrientation       = getAnglesZyxBaseToPlaneAtTime(dt).toImplementation();
  const auto splineEulerRates        = getEulerRatesZyxBaseInPlaneFrameAtTime(dt).toImplementation();
  const auto splineEulerAcceleration = getEulerAccelerationZyxBaseInPlaneFrameAtTime(dt).toImplementation();


  Eigen::Vector3d positionDifference = (splinePosition-measuredPositionInPlaneFrame_.toImplementation());
  Eigen::Vector3d orientationDifference = (splineOrientation-measuredOrientationBaseToPlane_.toImplementation());

  gradient =
      2.0 * positionDifference.dot(LinearWeight_*splineVelocity) +
      2.0 * orientationDifference.dot(AngularWeight_*splineEulerRates);

  hessian  =
      2.0 * ( splineVelocity.dot(LinearWeight_*splineVelocity) +
              positionDifference.dot(LinearWeight_*splineAcceleration) ) +
      2.0 * ( splineEulerRates.dot(AngularWeight_*splineEulerRates) +
              orientationDifference.dot(AngularWeight_*splineEulerAcceleration) );
}

double TrajectoryStateHandlerLineSearchLinearAngular::computeObjective(double dt) const {
  const auto splinePosition        = getPositionPlaneToComInPlaneFrameAtTime(dt).toImplementation();
  const auto splineOrientation     = getAnglesZyxBaseToPlaneAtTime(dt).toImplementation();
  const auto positionDifference    = (splinePosition-measuredPositionInPlaneFrame_.toImplementation());
  const auto orientationDifference = (splineOrientation-measuredOrientationBaseToPlane_.toImplementation());

  return positionDifference.dot(LinearWeight_*positionDifference) +
         orientationDifference.dot(AngularWeight_*orientationDifference);
}

double TrajectoryStateHandlerLineSearchLinearAngular::lineSearch(
    double containerTimeGuess,
    const Position& measuredPositionInPlaneFrame,
    const EulerAnglesZyx& measuredOrientationBaseToPlane,
    double minTime,
    double maxTime) {
  measuredPositionInPlaneFrame_   = measuredPositionInPlaneFrame;
  measuredOrientationBaseToPlane_ = measuredOrientationBaseToPlane;
  return TrajectoryStateHandlerLineSearch::lineSearch(containerTimeGuess, minTime, maxTime);
}

} /* namespace motion_generation */
