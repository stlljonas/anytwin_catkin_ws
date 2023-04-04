/**
 * @authors     Dario Bellicoso, Francisco Giraldez Gamez
 * @affiliation ETH Zurich, ANYbotics
 * @brief       Class to compute the analytical kinematics of a 3DOF leg.
 */

#pragma once

#include "analytical_inverse_kinematics/LegKinematicParameters.hpp"

namespace analytical_inverse_kinematics {

/** @class AnalyticalInverseKinematics
 *  @brief Computes the analytical inverse kinematics for a 3DOF leg.
 *
 *  It solves the following equations:
 *      *   q = atan(y_foot, x_foot) to compute HAA angle.
 *      *   a*cos(q) + b*sin(q) = c to compute the HFE and KFE angles.
 *
 * The equation coefficients are given by the leg parameters, which are passed upon computation.
 */
class AnalyticalInverseKinematics {
 public:
  AnalyticalInverseKinematics() = default;
  virtual ~AnalyticalInverseKinematics() = default;

  /** @brief Sets workspace limits for the inverse kinematics, to prevent singular configurations.
   *
   * @param minHipToFootLength Minimum distance allowed between FOOT and HIP frames.
   * @param maxHipToFootLength Minimum distance allowed between FOOT and HIP frames.
   * @param maxThighToFootLength Minimum distance allowed between FOOT and HIP frame.
   */
  virtual void initialize(double minHipToFootLength, double maxHipToFootLength, double maxThighToFootLength);

  /** @brief Computes the analytical IK for a 3DOF leg.
   *
   * @param[out] legJoints Joint positions to reach desired foot position.
   * @param[in] positionBaseToFootInBaseFrame Desired foot position in the base frame.
   * @param[in] parameters Kinematic parameters of the leg.
   * @param positiveHFESolution Flag to return the positive solution for the HFE trig. equation. If false, the negative one is returned.
   * @param positiveKFESolution Flag to return the positive solution for the KFE trig. equation. If false, the negative one is returned.
   * @return
   */
  bool getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                              const Eigen::Vector3d& positionBaseToFootInBaseFrame,
                                                              const LegKinematicParameters& parameters, bool positiveHFESolution,
                                                              bool positiveKFESolution) const;

 protected:
  //! Solves a*cos(x) + b*sin(x) = c.
  static bool solveLinearTrigonometricEquation(double& firstSolution, double& secondSolution, double a, double b, double c);

  //! Workspace limits.
  double minHipToFootLength_ = 0.0;
  double maxHipToFootLength_ = 0.0;
  double maxThighToFootLength_ = 0.0;
};

}  // namespace analytical_inverse_kinematics
