/*!
 * @file     NonlinearConstraintsGradientTest.cpp
 * @author   Dario Bellicoso
 * @date     Mar. 29, 2019
 * @brief
 */

// gtest
#include <gtest/gtest.h>

// zmp_optimizer
#include <zmp_optimizer/ZmpOptimizerObjectiveHandler.hpp>

// kindr
#include "kindr/common/gtest_eigen.hpp"

TEST(NonlinearConstraints, LinearGradientTest) {
  zmp::ZmpOptimizerObjectiveHandler handler;

  constexpr double mass = 10.0;
  const Eigen::Matrix3d inertiaTensor = Eigen::Matrix3d::Identity();
  handler.initialize(mass, inertiaTensor);

  std::vector<zmp::CogDim> optimizationDofs;
  optimizationDofs.push_back(zmp::CogDim::x);
  optimizationDofs.push_back(zmp::CogDim::y);
  optimizationDofs.push_back(zmp::CogDim::z);

  constexpr double optimizationTime = 0.3;

  handler.setTime(optimizationTime);
  handler.setOptimizationDofs(optimizationDofs);

  constexpr unsigned int numSplineCoefficients = zmp::coeffsTransl;

  Eigen::VectorXd gradientAnalytical = Eigen::VectorXd::Zero(numSplineCoefficients);
  Eigen::VectorXd gradientFiniteDifferences = Eigen::VectorXd::Zero(numSplineCoefficients);
  double evaluatedInequality;

  Eigen::VectorXd splineCoefficientsAtIterationK = Eigen::VectorXd::Random(numSplineCoefficients);
  std::vector<double> lineCoeffs;
  lineCoeffs.push_back(1.0);
  lineCoeffs.push_back(1.0);
  lineCoeffs.push_back(1.0);
  motion_generation::Position pathPosition = motion_generation::Position::Random();
  motion_generation::LinearAcceleration pathAccleration = motion_generation::LinearAcceleration::Random();

  // Evaluate the analytical gradient.
  handler.getTranslationalZmpInequalityConstraintObjective(gradientAnalytical, evaluatedInequality, splineCoefficientsAtIterationK,
                                                           lineCoeffs, pathPosition, pathAccleration);

  // Evaluate the finite differences gradient.
  constexpr double deltaH = 1.0e-6;
  for (auto k = 0u; k < numSplineCoefficients; ++k) {
    Eigen::VectorXd dummyVec;
    Eigen::VectorXd splineCoefficientsAtIterationKPerturbed = splineCoefficientsAtIterationK;
    splineCoefficientsAtIterationKPerturbed(k) = splineCoefficientsAtIterationK(k) + deltaH;
    double perturbationPos = 0.0;
    handler.getTranslationalZmpInequalityConstraintObjective(dummyVec, perturbationPos, splineCoefficientsAtIterationKPerturbed, lineCoeffs,
                                                             pathPosition, pathAccleration);

    splineCoefficientsAtIterationKPerturbed(k) = splineCoefficientsAtIterationK(k) - deltaH;
    double perturbationNeg = 0.0;
    handler.getTranslationalZmpInequalityConstraintObjective(dummyVec, perturbationNeg, splineCoefficientsAtIterationKPerturbed, lineCoeffs,
                                                             pathPosition, pathAccleration);
    gradientFiniteDifferences(k) = (perturbationPos - perturbationNeg) / (2.0 * deltaH);
  }

  std::string msg = "";
  KINDR_ASSERT_DOUBLE_MX_EQ_ZT(gradientAnalytical, gradientFiniteDifferences, 1.0, msg, 1.0e-7);
}

TEST(NonlinearConstraints, LinearAndAngularGradientTest) {
  zmp::ZmpOptimizerObjectiveHandler handler;

  constexpr double mass = 10.0;
  const Eigen::Matrix3d inertiaTensor = Eigen::Matrix3d::Identity();
  handler.initialize(mass, inertiaTensor);

  std::vector<zmp::CogDim> optimizationDofs;
  optimizationDofs.push_back(zmp::CogDim::x);
  optimizationDofs.push_back(zmp::CogDim::y);
  optimizationDofs.push_back(zmp::CogDim::z);
  optimizationDofs.push_back(zmp::CogDim::yaw);
  optimizationDofs.push_back(zmp::CogDim::pitch);
  optimizationDofs.push_back(zmp::CogDim::roll);

  constexpr double optimizationTime = 0.3;

  handler.setTime(optimizationTime);
  handler.setOptimizationDofs(optimizationDofs);

  constexpr unsigned int numSplineCoefficients = zmp::coeffsFullState;
  constexpr unsigned int numSplineCoefficientsLinear = zmp::coeffsTransl;
  constexpr unsigned int numSplineCoefficientsAngular = zmp::coeffsRot;

  Eigen::VectorXd gradientAnalytical = Eigen::VectorXd::Zero(numSplineCoefficients);
  Eigen::VectorXd gradientFiniteDifferences = Eigen::VectorXd::Zero(numSplineCoefficients);
  double evaluatedInequality;

  Eigen::VectorXd splineCoefficientsAtIterationK = Eigen::VectorXd::Random(numSplineCoefficients);

  std::vector<double> lineCoeffs;
  lineCoeffs.push_back(1.0);
  lineCoeffs.push_back(1.0);
  lineCoeffs.push_back(1.0);
  motion_generation::Position pathPosition = motion_generation::Position::Random();
  motion_generation::LinearAcceleration pathAccleration = motion_generation::LinearAcceleration::Random();
  motion_generation::EulerAnglesZyx eulerAnglesPathZyx = motion_generation::EulerAnglesZyx(Eigen::Vector3d::Random());
  motion_generation::EulerAnglesZyxDiff eulerAnglesPathZyx_dot = motion_generation::EulerAnglesZyxDiff(Eigen::Vector3d::Random());
  motion_generation::EulerAnglesZyxDiff eulerAnglesPathZyx_ddot = motion_generation::EulerAnglesZyxDiff(Eigen::Vector3d::Random());

  // Evaluate the analytical gradient.
  handler.getRotationalTranslationalZmpInequalityConstraintObjective(
      gradientAnalytical, evaluatedInequality, splineCoefficientsAtIterationK, lineCoeffs, pathPosition, pathAccleration,
      eulerAnglesPathZyx, eulerAnglesPathZyx_dot, eulerAnglesPathZyx_ddot);

  // Evaluate the finite differences gradient.
  constexpr double deltaH = 1.0e-6;
  for (auto k = 0u; k < numSplineCoefficients; ++k) {
    Eigen::VectorXd dummyVec = Eigen::VectorXd::Zero(numSplineCoefficients);
    Eigen::VectorXd splineCoefficientsAtIterationKPerturbed = splineCoefficientsAtIterationK;
    splineCoefficientsAtIterationKPerturbed(k) = splineCoefficientsAtIterationK(k) + deltaH;
    double perturbationPos = 0.0;
    handler.getRotationalTranslationalZmpInequalityConstraintObjective(dummyVec, perturbationPos, splineCoefficientsAtIterationKPerturbed,
                                                                       lineCoeffs, pathPosition, pathAccleration, eulerAnglesPathZyx,
                                                                       eulerAnglesPathZyx_dot, eulerAnglesPathZyx_ddot);

    splineCoefficientsAtIterationKPerturbed(k) = splineCoefficientsAtIterationK(k) - deltaH;
    double perturbationNeg = 0.0;
    handler.getRotationalTranslationalZmpInequalityConstraintObjective(dummyVec, perturbationNeg, splineCoefficientsAtIterationKPerturbed,
                                                                       lineCoeffs, pathPosition, pathAccleration, eulerAnglesPathZyx,
                                                                       eulerAnglesPathZyx_dot, eulerAnglesPathZyx_ddot);

    gradientFiniteDifferences(k) = (perturbationPos - perturbationNeg) / (2.0 * deltaH);
  }

  std::string msg = "";
  KINDR_ASSERT_DOUBLE_MX_EQ_ZT(gradientAnalytical.topRows<numSplineCoefficientsLinear>(),
                               gradientFiniteDifferences.topRows<numSplineCoefficientsLinear>(), 1.0, msg, 1.0e-10);
  KINDR_ASSERT_DOUBLE_MX_EQ_ZT(gradientAnalytical.bottomRows<numSplineCoefficientsAngular>(),
                               gradientFiniteDifferences.bottomRows<numSplineCoefficientsAngular>(), 1.0, msg, 1.0e-10);
}
