/**
 * @authors     Dario Bellicoso, Francisco Giraldez Gamez
 * @affiliation ETH Zurich, ANYbotics
 * @brief       Class to compute the analytical kinematics of a 3DOF leg.
 */

#include <message_logger/message_logger.hpp>
#include <robot_utils/math/math.hpp>

#include "analytical_inverse_kinematics/AnalyticalInverseKinematics.hpp"

namespace analytical_inverse_kinematics {

void AnalyticalInverseKinematics::initialize(double minHipToFootLength, double maxHipToFootLength, double maxThighToFootLength) {
  minHipToFootLength_ = minHipToFootLength;
  maxHipToFootLength_ = maxHipToFootLength;
  maxThighToFootLength_ = maxThighToFootLength;
}

bool AnalyticalInverseKinematics::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
    Eigen::Vector3d& legJoints, const Eigen::Vector3d& positionBaseToFootInBaseFrame, const LegKinematicParameters& parameters,
    bool positiveHFESolution, bool positiveKFESolution) const {
  Position positionHipToFootInBaseFrame = Position(positionBaseToFootInBaseFrame) - parameters.getPositionBaseToHipInBaseFrame();

  /********
   * qHAA *
   ********/
  const double d = parameters.getDefaultPositionHipToFootInBaseFrame().y();
  double positionYzSquared = positionHipToFootInBaseFrame.y() * positionHipToFootInBaseFrame.y() +
                             positionHipToFootInBaseFrame.z() * positionHipToFootInBaseFrame.z();

  // limit the desired hip to foot position if necessary
  const double dSquared = d * d;

  // Check for minimum distance from hip.
  if (positionYzSquared < dSquared) {
    MELO_WARN_STREAM("Desired position closer than " << dSquared << " [m] to the hip.");
    Eigen::Vector2d vec(positionHipToFootInBaseFrame.toImplementation().segment<2>(1));
    if (!robot_utils::areNear(static_cast<double>(vec.norm()), 0.0)) {
      vec = vec / vec.norm() * d * 1.0001;
      positionHipToFootInBaseFrame.toImplementation().segment<2>(1) = vec;
    } else {
      positionHipToFootInBaseFrame.y() = d;
      positionHipToFootInBaseFrame.z() = 0.0;
    }

    positionYzSquared = positionHipToFootInBaseFrame.y() * positionHipToFootInBaseFrame.y() +
                        positionHipToFootInBaseFrame.z() * positionHipToFootInBaseFrame.z();
  }

  // Check for minimum distance from hip.
  double rDelta = 0.0;
  if (positionYzSquared < dSquared) {
    MELO_WARN("Desired position is still too close to the hip. IK computation failed!");
    return false;
  } else {
    rDelta = positionYzSquared - dSquared;
  }

  const double r = std::sqrt(rDelta);
  const double delta = std::atan2(positionHipToFootInBaseFrame.y(), -positionHipToFootInBaseFrame.z());
  const double beta = std::atan2(r, d);
  const double qHAA = beta + delta - M_PI_2;
  legJoints[0] = qHAA;

  MELO_DEBUG_STREAM("qHAA = " << qHAA << ", with r = " << r << ", beta = " << beta << ", delta = " << delta << ".")

  /********
   * qKFE *
   ********/
  const RotationMatrix orientationHipToBase(EulerAnglesZyx(0.0, 0.0, qHAA).setUnique());
  const Position positionBaseToThighInBaseFrame =
      parameters.getPositionBaseToHipInBaseFrame() + orientationHipToBase.rotate(parameters.getPositionHipToThighInHipFrame());
  const Position positionThighToFootInBaseFrame = Position(positionBaseToFootInBaseFrame) - positionBaseToThighInBaseFrame;

  const double sqrt_dKFE = positionThighToFootInBaseFrame.norm();
  const double dKFE = sqrt_dKFE * sqrt_dKFE;

  double firstSol = 0.0;
  double secondSol = 0.0;
  double aKFE = parameters.getTrigonometricEquationCoefficientKfeA();
  double bKFE = parameters.getTrigonometricEquationCoefficientKfeB();
  double cKFE = parameters.getTrigonometricEquationCoefficientKfeC();
  solveLinearTrigonometricEquation(firstSol, secondSol, aKFE, bKFE, dKFE - cKFE);

  double qKFE = (positiveKFESolution) ? secondSol : firstSol;
  legJoints[2] = qKFE;

  MELO_DEBUG_STREAM("qKFE = " << qKFE << ", with aKFE = " << aKFE << ", bKFE = " << bKFE << ", cKFE = " << cKFE << ", and dKFE = " << dKFE
                              << ".")

  /********
   * qHFE *
   ********/
  const double sqrt_dHFE = positionHipToFootInBaseFrame.norm();
  const double dHFE = sqrt_dHFE * sqrt_dHFE;
  const double aHFE = parameters.getTrigonometricEquationCoefficientHfeA(qKFE);
  const double bHFE = parameters.getTrigonometricEquationCoefficientHfeB(qKFE);
  const double cHFE = parameters.getTrigonometricEquationCoefficientHfeC(qKFE);

  solveLinearTrigonometricEquation(firstSol, secondSol, aHFE, bHFE, dHFE - cHFE);

  double qHFE = (positiveHFESolution) ? secondSol : firstSol;
  legJoints[1] = qHFE;

  MELO_DEBUG_STREAM("qHFE = " << qHFE << ", with aHFE = " << aHFE << ", bHFE = " << bHFE << ", cHFE = " << cHFE << ", and dHFE = " << dHFE
                              << ".")

  return true;
}

bool AnalyticalInverseKinematics::solveLinearTrigonometricEquation(double& firstSolution, double& secondSolution, double a, double b,
                                                                   double c) {
  /* We want to solve the equation:
   *       a*cos(x) + b*sin(x) = c
   */
  try {
    double delta = (a * a + b * b - c * c);
    if (delta < 0.0) {
      MELO_WARN_STREAM("[InverseKinematics::solveLinearTrigonometricEquation] Delta = a^2 + b^2 - c^2  = "
                       << delta << " < 0.0, for a = " << a << ", b = " << b << ", c =" << c);
      delta = 0.0;
      if (fabs(c) < 1e-10) {
        c = 0.0;
      }
    }
    const double firstTerm = std::atan2(a, b);
    const double secondTerm = std::atan2(std::sqrt(delta), c);

    firstSolution = firstTerm - secondTerm;
    secondSolution = firstTerm + secondTerm;

  } catch (...) {
    MELO_ERROR(
        "[InverseKinematics::solveLinearTrigonometricEquation] Failed to solve linear trigonometric equation while solving"
        " inverse kinematics!");
    return false;
  }

  return true;
}

}  // namespace analytical_inverse_kinematics
