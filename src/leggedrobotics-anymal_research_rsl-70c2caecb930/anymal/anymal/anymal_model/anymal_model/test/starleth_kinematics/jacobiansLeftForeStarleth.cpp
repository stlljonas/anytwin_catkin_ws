/*
 * dJ_lf.hpp
 *
 *  Created on: Oct 16, 2015
 *      Author: dbellicoso
 */

#include "jacobiansLeftForeStarleth.hpp"

constexpr double baseToHipX = 0.505 / 2.0;
constexpr double baseToHipY = 0.37 / 2.0;

constexpr double lengthHip = -0.0685;
constexpr double lengthThigh = -0.2000;
constexpr double lengthShank = -0.2760;

namespace starleth_kinematics {

Eigen::MatrixXd getJacobianRotationWorldToBaseInWorldFrame(const Eigen::VectorXd& generalizedPositionEulerXyz) {
  Eigen::MatrixXd jacobian(3, 18);
  jacobian.setZero();

  double qAL = generalizedPositionEulerXyz[3];
  double qBE = generalizedPositionEulerXyz[4];
  // double qGA = generalizedPositionEulerXyz[5];

  const double t68 = sin(qAL);
  const double t69 = cos(qAL);
  const double t70 = cos(qBE);
  jacobian(0, 3) = 1.0;
  jacobian(0, 5) = sin(qBE);
  jacobian(1, 4) = t69;
  jacobian(1, 5) = -t68 * t70;
  jacobian(2, 4) = t68;
  jacobian(2, 5) = t69 * t70;

  return jacobian;
}

Eigen::MatrixXd getJacobianRotationDerivativeWorldToBaseInWorldFrame(const Eigen::VectorXd& generalizedPositionEulerXyz,
                                                                     const Eigen::VectorXd& generalizedVelocitiesEulerXyz) {
  Eigen::MatrixXd jacobianDerivative(3, 18);
  jacobianDerivative.setZero();

  double qAL = generalizedPositionEulerXyz[3];
  double qBE = generalizedPositionEulerXyz[4];
  // double qGA = generalizedPositionEulerXyz[5];

  double DqAL = generalizedVelocitiesEulerXyz[3];
  double DqBE = generalizedVelocitiesEulerXyz[4];
  // double DqGA = generalizedVelocitiesEulerXyz[5];

  const double t72 = cos(qBE);
  const double t73 = sin(qAL);
  const double t74 = cos(qAL);
  const double t75 = sin(qBE);

  jacobianDerivative(0, 5) = DqBE * t72;
  jacobianDerivative(1, 4) = -DqAL * t73;
  jacobianDerivative(1, 5) = -DqAL * t72 * t74 + DqBE * t73 * t75;
  jacobianDerivative(2, 4) = DqAL * t74;
  jacobianDerivative(2, 5) = -DqAL * t72 * t73 - DqBE * t74 * t75;

  return jacobianDerivative;
}

Eigen::MatrixXd getJacobianWorldToLFFootInWorldFrameFromRobotModel(const Eigen::VectorXd& generalizedPositionsEulerXyz) {
  // double t0;

  double qAL = generalizedPositionsEulerXyz[3];
  double qBE = generalizedPositionsEulerXyz[4];
  double qGA = generalizedPositionsEulerXyz[5];
  double qLF_HAA = generalizedPositionsEulerXyz[6];
  double qLF_HFE = generalizedPositionsEulerXyz[7];
  double qLF_KFE = generalizedPositionsEulerXyz[8];

  Eigen::MatrixXd jacobian(3, 18);
  jacobian.setZero();

  const double t2768 = sin(qBE);
  const double t2769 = cos(qBE);
  const double t2770 = cos(qLF_HAA);
  const double t2771 = cos(qGA);
  const double t2772 = sin(qGA);
  const double t2773 = cos(qLF_HFE);
  const double t2774 = cos(qLF_KFE);
  const double t2775 = sin(qLF_HFE);
  const double t2776 = sin(qLF_KFE);
  const double t2777 = sin(qLF_HAA);
  const double t2778 = lengthShank * t2771 * t2773 * t2774 * t2769;
  const double t2779 = sin(qAL);
  const double t2780 = cos(qAL);
  const double t2781 = lengthShank * t2780 * t2772 * t2773 * t2774;
  const double t2782 = lengthShank * t2771 * t2773 * t2774 * t2768 * t2779;
  const double t2783 = lengthShank * t2770 * t2773 * t2776 * t2769 * t2779;
  const double t2784 = lengthShank * t2770 * t2774 * t2775 * t2769 * t2779;
  const double t2785 = lengthShank * t2771 * t2780 * t2773 * t2776 * t2777;
  const double t2786 = lengthShank * t2771 * t2780 * t2774 * t2775 * t2777;
  const double t2787 = lengthShank * t2772 * t2773 * t2774 * t2779;
  const double t2788 = lengthShank * t2771 * t2780 * t2775 * t2776 * t2768;
  const double t2789 = lengthShank * t2771 * t2773 * t2776 * t2777 * t2779;
  const double t2790 = lengthShank * t2771 * t2774 * t2775 * t2777 * t2779;
  const double t2791 = lengthShank * t2780 * t2772 * t2773 * t2776 * t2768 * t2777;
  const double t2792 = lengthShank * t2780 * t2772 * t2774 * t2775 * t2768 * t2777;
  // t0 = jacobian(0, 0) = 1.0;
  jacobian(0, 4) = -baseToHipX * t2771 * t2768 + baseToHipY * t2772 * t2768 + lengthHip * t2770 * t2769 -
                   lengthHip * t2772 * t2768 * t2777 + lengthThigh * t2770 * t2773 * t2769 - lengthThigh * t2771 * t2775 * t2768 +
                   lengthShank * t2770 * t2773 * t2774 * t2769 - lengthShank * t2771 * t2773 * t2776 * t2768 -
                   lengthShank * t2771 * t2774 * t2775 * t2768 - lengthShank * t2770 * t2775 * t2776 * t2769 -
                   lengthThigh * t2772 * t2773 * t2768 * t2777 - lengthShank * t2772 * t2773 * t2774 * t2768 * t2777 +
                   lengthShank * t2772 * t2775 * t2776 * t2768 * t2777;
  jacobian(0, 5) = -baseToHipX * t2772 * t2769 - baseToHipY * t2771 * t2769 + lengthHip * t2771 * t2777 * t2769 -
                   lengthThigh * t2772 * t2775 * t2769 - lengthShank * t2772 * t2773 * t2776 * t2769 -
                   lengthShank * t2772 * t2774 * t2775 * t2769 + lengthThigh * t2771 * t2773 * t2777 * t2769 +
                   lengthShank * t2771 * t2773 * t2774 * t2777 * t2769 - lengthShank * t2771 * t2775 * t2776 * t2777 * t2769;
  jacobian(0, 6) = -lengthHip * t2768 * t2777 + lengthHip * t2770 * t2772 * t2769 - lengthThigh * t2773 * t2768 * t2777 -
                   lengthShank * t2773 * t2774 * t2768 * t2777 + lengthShank * t2775 * t2776 * t2768 * t2777 +
                   lengthThigh * t2770 * t2772 * t2773 * t2769 + lengthShank * t2770 * t2772 * t2773 * t2774 * t2769 -
                   lengthShank * t2770 * t2772 * t2775 * t2776 * t2769;
  jacobian(0, 7) = t2778 - lengthThigh * t2770 * t2775 * t2768 + lengthThigh * t2771 * t2773 * t2769 -
                   lengthShank * t2770 * t2773 * t2776 * t2768 - lengthShank * t2770 * t2774 * t2775 * t2768 -
                   lengthShank * t2771 * t2775 * t2776 * t2769 - lengthThigh * t2772 * t2775 * t2777 * t2769 -
                   lengthShank * t2772 * t2773 * t2776 * t2777 * t2769 - lengthShank * t2772 * t2774 * t2775 * t2777 * t2769;
  jacobian(0, 8) = t2778 - lengthShank * t2770 * t2773 * t2776 * t2768 - lengthShank * t2770 * t2774 * t2775 * t2768 -
                   lengthShank * t2771 * t2775 * t2776 * t2769 - lengthShank * t2772 * t2773 * t2776 * t2777 * t2769 -
                   lengthShank * t2772 * t2774 * t2775 * t2777 * t2769;
  jacobian(1, 1) = 1.0;
  jacobian(1, 3) = -baseToHipX * t2772 * t2779 - baseToHipY * t2771 * t2779 + baseToHipX * t2771 * t2780 * t2768 -
                   baseToHipY * t2780 * t2772 * t2768 - lengthHip * t2770 * t2780 * t2769 + lengthHip * t2771 * t2777 * t2779 -
                   lengthThigh * t2772 * t2775 * t2779 + lengthHip * t2780 * t2772 * t2768 * t2777 -
                   lengthShank * t2772 * t2773 * t2776 * t2779 - lengthShank * t2772 * t2774 * t2775 * t2779 -
                   lengthThigh * t2770 * t2780 * t2773 * t2769 + lengthThigh * t2771 * t2780 * t2775 * t2768 +
                   lengthThigh * t2771 * t2773 * t2777 * t2779 - lengthShank * t2770 * t2780 * t2773 * t2774 * t2769 +
                   lengthShank * t2771 * t2780 * t2773 * t2776 * t2768 + lengthShank * t2771 * t2780 * t2774 * t2775 * t2768 +
                   lengthShank * t2770 * t2780 * t2775 * t2776 * t2769 + lengthShank * t2771 * t2773 * t2774 * t2777 * t2779 -
                   lengthShank * t2771 * t2775 * t2776 * t2777 * t2779 + lengthThigh * t2780 * t2772 * t2773 * t2768 * t2777 +
                   lengthShank * t2780 * t2772 * t2773 * t2774 * t2768 * t2777 -
                   lengthShank * t2780 * t2772 * t2775 * t2776 * t2768 * t2777;
  jacobian(1, 4) = baseToHipX * t2771 * t2769 * t2779 - baseToHipY * t2772 * t2769 * t2779 + lengthHip * t2770 * t2768 * t2779 +
                   lengthHip * t2772 * t2777 * t2769 * t2779 + lengthThigh * t2770 * t2773 * t2768 * t2779 +
                   lengthThigh * t2771 * t2775 * t2769 * t2779 + lengthShank * t2770 * t2773 * t2774 * t2768 * t2779 -
                   lengthShank * t2770 * t2775 * t2776 * t2768 * t2779 + lengthShank * t2771 * t2773 * t2776 * t2769 * t2779 +
                   lengthShank * t2771 * t2774 * t2775 * t2769 * t2779 + lengthThigh * t2772 * t2773 * t2777 * t2769 * t2779 +
                   lengthShank * t2772 * t2773 * t2774 * t2777 * t2769 * t2779 -
                   lengthShank * t2772 * t2775 * t2776 * t2777 * t2769 * t2779;
  jacobian(1, 5) = baseToHipX * t2771 * t2780 - baseToHipY * t2780 * t2772 - baseToHipX * t2772 * t2768 * t2779 -
                   baseToHipY * t2771 * t2768 * t2779 + lengthHip * t2780 * t2772 * t2777 + lengthThigh * t2771 * t2780 * t2775 +
                   lengthHip * t2771 * t2768 * t2777 * t2779 + lengthShank * t2771 * t2780 * t2773 * t2776 +
                   lengthShank * t2771 * t2780 * t2774 * t2775 + lengthThigh * t2780 * t2772 * t2773 * t2777 -
                   lengthThigh * t2772 * t2775 * t2768 * t2779 + lengthShank * t2780 * t2772 * t2773 * t2774 * t2777 -
                   lengthShank * t2780 * t2772 * t2775 * t2776 * t2777 - lengthShank * t2772 * t2773 * t2776 * t2768 * t2779 -
                   lengthShank * t2772 * t2774 * t2775 * t2768 * t2779 + lengthThigh * t2771 * t2773 * t2768 * t2777 * t2779 +
                   lengthShank * t2771 * t2773 * t2774 * t2768 * t2777 * t2779 -
                   lengthShank * t2771 * t2775 * t2776 * t2768 * t2777 * t2779;
  jacobian(1, 6) = -lengthHip * t2770 * t2771 * t2780 + lengthHip * t2777 * t2769 * t2779 + lengthHip * t2770 * t2772 * t2768 * t2779 -
                   lengthThigh * t2770 * t2771 * t2780 * t2773 + lengthThigh * t2773 * t2777 * t2769 * t2779 -
                   lengthShank * t2770 * t2771 * t2780 * t2773 * t2774 + lengthShank * t2770 * t2771 * t2780 * t2775 * t2776 +
                   lengthShank * t2773 * t2774 * t2777 * t2769 * t2779 - lengthShank * t2775 * t2776 * t2777 * t2769 * t2779 +
                   lengthThigh * t2770 * t2772 * t2773 * t2768 * t2779 + lengthShank * t2770 * t2772 * t2773 * t2774 * t2768 * t2779 -
                   lengthShank * t2770 * t2772 * t2775 * t2776 * t2768 * t2779;
  jacobian(1, 7) = t2781 + t2782 + t2783 + t2784 + t2785 + t2786 + lengthThigh * t2780 * t2772 * t2773 -
                   lengthShank * t2780 * t2772 * t2775 * t2776 + lengthThigh * t2771 * t2780 * t2775 * t2777 +
                   lengthThigh * t2771 * t2773 * t2768 * t2779 + lengthThigh * t2770 * t2775 * t2769 * t2779 -
                   lengthShank * t2771 * t2775 * t2776 * t2768 * t2779 - lengthThigh * t2772 * t2775 * t2768 * t2777 * t2779 -
                   lengthShank * t2772 * t2773 * t2776 * t2768 * t2777 * t2779 -
                   lengthShank * t2772 * t2774 * t2775 * t2768 * t2777 * t2779;
  jacobian(1, 8) = t2781 + t2782 + t2783 + t2784 + t2785 + t2786 - lengthShank * t2780 * t2772 * t2775 * t2776 -
                   lengthShank * t2771 * t2775 * t2776 * t2768 * t2779 - lengthShank * t2772 * t2773 * t2776 * t2768 * t2777 * t2779 -
                   lengthShank * t2772 * t2774 * t2775 * t2768 * t2777 * t2779;
  jacobian(2, 2) = 1.0;
  jacobian(2, 3) = baseToHipX * t2780 * t2772 + baseToHipY * t2771 * t2780 + baseToHipX * t2771 * t2768 * t2779 -
                   baseToHipY * t2772 * t2768 * t2779 - lengthHip * t2771 * t2780 * t2777 - lengthHip * t2770 * t2769 * t2779 +
                   lengthThigh * t2780 * t2772 * t2775 + lengthHip * t2772 * t2768 * t2777 * t2779 +
                   lengthShank * t2780 * t2772 * t2773 * t2776 + lengthShank * t2780 * t2772 * t2774 * t2775 -
                   lengthThigh * t2771 * t2780 * t2773 * t2777 - lengthThigh * t2770 * t2773 * t2769 * t2779 +
                   lengthThigh * t2771 * t2775 * t2768 * t2779 - lengthShank * t2771 * t2780 * t2773 * t2774 * t2777 +
                   lengthShank * t2771 * t2780 * t2775 * t2776 * t2777 - lengthShank * t2770 * t2773 * t2774 * t2769 * t2779 +
                   lengthShank * t2771 * t2773 * t2776 * t2768 * t2779 + lengthShank * t2771 * t2774 * t2775 * t2768 * t2779 +
                   lengthShank * t2770 * t2775 * t2776 * t2769 * t2779 + lengthThigh * t2772 * t2773 * t2768 * t2777 * t2779 +
                   lengthShank * t2772 * t2773 * t2774 * t2768 * t2777 * t2779 -
                   lengthShank * t2772 * t2775 * t2776 * t2768 * t2777 * t2779;
  jacobian(2, 4) = -baseToHipX * t2771 * t2780 * t2769 + baseToHipY * t2780 * t2772 * t2769 - lengthHip * t2770 * t2780 * t2768 -
                   lengthHip * t2780 * t2772 * t2777 * t2769 - lengthThigh * t2770 * t2780 * t2773 * t2768 -
                   lengthThigh * t2771 * t2780 * t2775 * t2769 - lengthShank * t2770 * t2780 * t2773 * t2774 * t2768 +
                   lengthShank * t2770 * t2780 * t2775 * t2776 * t2768 - lengthShank * t2771 * t2780 * t2773 * t2776 * t2769 -
                   lengthShank * t2771 * t2780 * t2774 * t2775 * t2769 - lengthThigh * t2780 * t2772 * t2773 * t2777 * t2769 -
                   lengthShank * t2780 * t2772 * t2773 * t2774 * t2777 * t2769 +
                   lengthShank * t2780 * t2772 * t2775 * t2776 * t2777 * t2769;
  jacobian(2, 5) = baseToHipX * t2771 * t2779 - baseToHipY * t2772 * t2779 + baseToHipX * t2780 * t2772 * t2768 +
                   baseToHipY * t2771 * t2780 * t2768 + lengthHip * t2772 * t2777 * t2779 + lengthThigh * t2771 * t2775 * t2779 -
                   lengthHip * t2771 * t2780 * t2768 * t2777 + lengthShank * t2771 * t2773 * t2776 * t2779 +
                   lengthShank * t2771 * t2774 * t2775 * t2779 + lengthThigh * t2780 * t2772 * t2775 * t2768 +
                   lengthThigh * t2772 * t2773 * t2777 * t2779 + lengthShank * t2780 * t2772 * t2773 * t2776 * t2768 +
                   lengthShank * t2780 * t2772 * t2774 * t2775 * t2768 + lengthShank * t2772 * t2773 * t2774 * t2777 * t2779 -
                   lengthShank * t2772 * t2775 * t2776 * t2777 * t2779 - lengthThigh * t2771 * t2780 * t2773 * t2768 * t2777 -
                   lengthShank * t2771 * t2780 * t2773 * t2774 * t2768 * t2777 +
                   lengthShank * t2771 * t2780 * t2775 * t2776 * t2768 * t2777;
  jacobian(2, 6) = -lengthHip * t2770 * t2771 * t2779 - lengthHip * t2780 * t2777 * t2769 - lengthHip * t2770 * t2780 * t2772 * t2768 -
                   lengthThigh * t2770 * t2771 * t2773 * t2779 - lengthThigh * t2780 * t2773 * t2777 * t2769 -
                   lengthShank * t2770 * t2771 * t2773 * t2774 * t2779 + lengthShank * t2770 * t2771 * t2775 * t2776 * t2779 -
                   lengthShank * t2780 * t2773 * t2774 * t2777 * t2769 + lengthShank * t2780 * t2775 * t2776 * t2777 * t2769 -
                   lengthThigh * t2770 * t2780 * t2772 * t2773 * t2768 - lengthShank * t2770 * t2780 * t2772 * t2773 * t2774 * t2768 +
                   lengthShank * t2770 * t2780 * t2772 * t2775 * t2776 * t2768;
  jacobian(2, 7) = t2790 + t2791 + t2792 + t2787 + t2788 + t2789 + lengthThigh * t2772 * t2773 * t2779 -
                   lengthShank * t2772 * t2775 * t2776 * t2779 - lengthThigh * t2771 * t2780 * t2773 * t2768 -
                   lengthThigh * t2770 * t2780 * t2775 * t2769 + lengthThigh * t2771 * t2775 * t2777 * t2779 -
                   lengthShank * t2771 * t2780 * t2773 * t2774 * t2768 - lengthShank * t2770 * t2780 * t2773 * t2776 * t2769 -
                   lengthShank * t2770 * t2780 * t2774 * t2775 * t2769 + lengthThigh * t2780 * t2772 * t2775 * t2768 * t2777;
  jacobian(2, 8) = t2790 + t2791 + t2792 + t2787 + t2788 + t2789 - lengthShank * t2772 * t2775 * t2776 * t2779 -
                   lengthShank * t2771 * t2780 * t2773 * t2774 * t2768 - lengthShank * t2770 * t2780 * t2773 * t2776 * t2769 -
                   lengthShank * t2770 * t2780 * t2774 * t2775 * t2769;
  return jacobian;
}

Eigen::MatrixXd getJacobianDerivativeWorldToLFFootInWorldFrameFromRobotModel(const Eigen::VectorXd& generalizedPositionsEulerXyz,
                                                                             const Eigen::VectorXd& generalizedVelocitiesEulerXyz) {
  Eigen::MatrixXd jacobianDerivative(3, 18);
  jacobianDerivative.setZero();

  double qAL = generalizedPositionsEulerXyz[3];
  double qBE = generalizedPositionsEulerXyz[4];
  double qGA = generalizedPositionsEulerXyz[5];
  double qLF_HAA = generalizedPositionsEulerXyz[6];
  double qLF_HFE = generalizedPositionsEulerXyz[7];
  double qLF_KFE = generalizedPositionsEulerXyz[8];

  double DqAL = generalizedVelocitiesEulerXyz[3];
  double DqBE = generalizedVelocitiesEulerXyz[4];
  double DqGA = generalizedVelocitiesEulerXyz[5];
  double DqLF_HAA = generalizedVelocitiesEulerXyz[6];
  double DqLF_HFE = generalizedVelocitiesEulerXyz[7];
  double DqLF_KFE = generalizedVelocitiesEulerXyz[8];

  // double t0;

  const double t1424 = sin(qBE);
  const double t1425 = cos(qGA);
  const double t1426 = sin(qGA);
  const double t1427 = sin(qLF_HAA);
  const double t1428 = cos(qLF_HFE);
  const double t1429 = sin(qLF_HFE);
  const double t1430 = cos(qLF_KFE);
  const double t1431 = sin(qLF_KFE);
  const double t1432 = cos(qBE);
  const double t1433 = cos(qLF_HAA);
  const double t1434 = lengthShank * t1430 * t1424 * t1425 * t1428;
  const double t1435 = lengthShank * t1431 * t1432 * t1433 * t1428;
  const double t1436 = lengthShank * t1430 * t1432 * t1433 * t1429;
  const double t1437 = baseToHipY * t1424 * t1425;
  const double t1438 = baseToHipX * t1424 * t1426;
  const double t1439 = lengthThigh * t1424 * t1426 * t1429;
  const double t1440 = lengthShank * t1431 * t1424 * t1426 * t1428;
  const double t1441 = lengthShank * t1430 * t1424 * t1426 * t1429;
  const double t1442 = lengthShank * t1431 * t1424 * t1425 * t1427 * t1429;
  const double t1443 = t1440 + t1441 + t1442 + t1437 + t1438 + t1439 - lengthHip * t1424 * t1425 * t1427 -
                       lengthThigh * t1424 * t1425 * t1427 * t1428 - lengthShank * t1430 * t1424 * t1425 * t1427 * t1428;
  const double t1444 = baseToHipX * t1432 * t1425;
  const double t1445 = lengthThigh * t1432 * t1425 * t1429;
  const double t1446 = lengthHip * t1432 * t1426 * t1427;
  const double t1447 = lengthShank * t1431 * t1432 * t1425 * t1428;
  const double t1448 = lengthShank * t1430 * t1432 * t1425 * t1429;
  const double t1449 = lengthThigh * t1432 * t1426 * t1427 * t1428;
  const double t1450 = lengthShank * t1430 * t1432 * t1426 * t1427 * t1428;
  const double t1451 = lengthShank * t1430 * t1432 * t1426 * t1428;
  const double t1452 = lengthShank * t1431 * t1432 * t1425 * t1427 * t1428;
  const double t1453 = lengthShank * t1430 * t1432 * t1425 * t1427 * t1429;
  const double t1454 = lengthHip * t1432 * t1427;
  const double t1455 = lengthThigh * t1432 * t1427 * t1428;
  const double t1456 = lengthHip * t1424 * t1433 * t1426;
  const double t1457 = lengthShank * t1430 * t1432 * t1427 * t1428;
  const double t1458 = lengthThigh * t1424 * t1433 * t1426 * t1428;
  const double t1459 = lengthShank * t1430 * t1424 * t1433 * t1426 * t1428;
  const double t1460 = t1454 + t1455 + t1456 + t1457 + t1458 + t1459 - lengthShank * t1431 * t1432 * t1427 * t1429 -
                       lengthShank * t1431 * t1424 * t1433 * t1426 * t1429;
  const double t1461 = lengthHip * t1424 * t1433;
  const double t1462 = lengthThigh * t1424 * t1433 * t1428;
  const double t1463 = lengthShank * t1430 * t1424 * t1433 * t1428;
  const double t1464 = lengthHip * t1432 * t1433 * t1425;
  const double t1465 = lengthThigh * t1432 * t1433 * t1425 * t1428;
  const double t1466 = lengthShank * t1430 * t1432 * t1433 * t1425 * t1428;
  const double t1467 = t1464 + t1465 + t1466 - lengthShank * t1431 * t1432 * t1433 * t1425 * t1429;
  const double t1468 = lengthShank * t1431 * t1424 * t1427 * t1428;
  const double t1469 = lengthShank * t1430 * t1424 * t1427 * t1429;
  const double t1470 = lengthThigh * t1432 * t1426 * t1428;
  const double t1471 = lengthThigh * t1432 * t1425 * t1427 * t1429;
  const double t1484 = lengthShank * t1431 * t1432 * t1426 * t1429;
  const double t1472 = t1451 + t1452 + t1470 + t1453 + t1471 - t1484;
  const double t1473 = lengthThigh * t1424 * t1425 * t1428;
  const double t1474 = lengthThigh * t1432 * t1433 * t1429;
  const double t1481 = lengthShank * t1431 * t1424 * t1425 * t1429;
  const double t1482 = lengthShank * t1431 * t1424 * t1426 * t1427 * t1428;
  const double t1483 = lengthShank * t1430 * t1424 * t1426 * t1427 * t1429;
  const double t1475 = t1434 + t1435 + t1436 - t1481 + t1473 - t1482 + t1474 - t1483 - lengthThigh * t1424 * t1426 * t1427 * t1429;
  const double t1476 = lengthThigh * t1424 * t1427 * t1429;
  const double t1485 = lengthShank * t1431 * t1432 * t1433 * t1426 * t1428;
  const double t1486 = lengthShank * t1430 * t1432 * t1433 * t1426 * t1429;
  const double t1477 = t1476 - t1485 + t1468 - t1486 + t1469 - lengthThigh * t1432 * t1433 * t1426 * t1429;
  const double t1479 = lengthShank * t1431 * t1424 * t1433 * t1429;
  const double t1480 = lengthShank * t1431 * t1432 * t1426 * t1427 * t1429;
  const double t1478 = t1450 - t1480 + t1463 + t1447 + t1448 - t1479;
  const double t1487 = sin(qAL);
  const double t1488 = cos(qAL);
  const double t1489 = lengthShank * t1430 * t1426 * t1428 * t1487;
  const double t1490 = lengthShank * t1431 * t1424 * t1425 * t1429 * t1488;
  const double t1491 = lengthShank * t1431 * t1425 * t1427 * t1428 * t1487;
  const double t1492 = lengthShank * t1430 * t1425 * t1427 * t1429 * t1487;
  const double t1493 = lengthShank * t1431 * t1424 * t1426 * t1427 * t1428 * t1488;
  const double t1494 = lengthShank * t1430 * t1424 * t1426 * t1427 * t1429 * t1488;
  const double t1495 = lengthShank * t1431 * t1432 * t1425 * t1429 * t1487;
  const double t1496 = lengthShank * t1431 * t1424 * t1433 * t1428 * t1487;
  const double t1497 = lengthShank * t1430 * t1424 * t1433 * t1429 * t1487;
  const double t1498 = lengthShank * t1431 * t1432 * t1426 * t1427 * t1428 * t1487;
  const double t1499 = lengthShank * t1430 * t1432 * t1426 * t1427 * t1429 * t1487;
  const double t1500 = baseToHipX * t1432 * t1425 * t1488;
  const double t1501 = lengthHip * t1424 * t1433 * t1488;
  const double t1502 = lengthThigh * t1432 * t1425 * t1429 * t1488;
  const double t1503 = lengthThigh * t1424 * t1433 * t1428 * t1488;
  const double t1504 = lengthHip * t1432 * t1426 * t1427 * t1488;
  const double t1505 = lengthShank * t1431 * t1432 * t1425 * t1428 * t1488;
  const double t1506 = lengthShank * t1430 * t1432 * t1425 * t1429 * t1488;
  const double t1507 = lengthShank * t1430 * t1424 * t1433 * t1428 * t1488;
  const double t1508 = lengthThigh * t1432 * t1426 * t1427 * t1428 * t1488;
  const double t1509 = lengthShank * t1430 * t1432 * t1426 * t1427 * t1428 * t1488;
  const double t1510 = t1500 + t1501 + t1502 + t1503 + t1504 + t1505 + t1506 + t1507 + t1508 + t1509 - baseToHipY * t1432 * t1426 * t1488 -
                       lengthShank * t1431 * t1424 * t1433 * t1429 * t1488 - lengthShank * t1431 * t1432 * t1426 * t1427 * t1429 * t1488;
  const double t1511 = baseToHipX * t1424 * t1425 * t1487;
  const double t1512 = lengthThigh * t1424 * t1425 * t1429 * t1487;
  const double t1513 = lengthHip * t1424 * t1426 * t1427 * t1487;
  const double t1514 = lengthShank * t1431 * t1424 * t1425 * t1428 * t1487;
  const double t1515 = lengthShank * t1430 * t1424 * t1425 * t1429 * t1487;
  const double t1516 = lengthShank * t1431 * t1432 * t1433 * t1429 * t1487;
  const double t1517 = lengthThigh * t1424 * t1426 * t1427 * t1428 * t1487;
  const double t1518 = lengthShank * t1430 * t1424 * t1426 * t1427 * t1428 * t1487;
  const double t1519 = lengthShank * t1431 * t1425 * t1429 * t1488;
  const double t1520 = lengthShank * t1430 * t1424 * t1426 * t1428 * t1487;
  const double t1521 = lengthShank * t1431 * t1426 * t1427 * t1428 * t1488;
  const double t1522 = lengthShank * t1430 * t1426 * t1427 * t1429 * t1488;
  const double t1523 = lengthShank * t1431 * t1424 * t1425 * t1427 * t1428 * t1487;
  const double t1524 = lengthShank * t1430 * t1424 * t1425 * t1427 * t1429 * t1487;
  const double t1525 = baseToHipY * t1432 * t1425 * t1487;
  const double t1526 = baseToHipX * t1432 * t1426 * t1487;
  const double t1527 = lengthThigh * t1432 * t1426 * t1429 * t1487;
  const double t1528 = lengthShank * t1431 * t1432 * t1426 * t1428 * t1487;
  const double t1529 = lengthShank * t1430 * t1432 * t1426 * t1429 * t1487;
  const double t1530 = lengthShank * t1431 * t1432 * t1425 * t1427 * t1429 * t1487;
  const double t1531 = t1530 + t1525 + t1526 + t1527 + t1528 + t1529 - lengthHip * t1432 * t1425 * t1427 * t1487 -
                       lengthThigh * t1432 * t1425 * t1427 * t1428 * t1487 - lengthShank * t1430 * t1432 * t1425 * t1427 * t1428 * t1487;
  const double t1532 = baseToHipX * t1425 * t1487;
  const double t1533 = baseToHipY * t1424 * t1425 * t1488;
  const double t1534 = baseToHipX * t1424 * t1426 * t1488;
  const double t1535 = lengthThigh * t1425 * t1429 * t1487;
  const double t1536 = lengthHip * t1426 * t1427 * t1487;
  const double t1537 = lengthShank * t1431 * t1425 * t1428 * t1487;
  const double t1538 = lengthShank * t1430 * t1425 * t1429 * t1487;
  const double t1539 = lengthThigh * t1424 * t1426 * t1429 * t1488;
  const double t1540 = lengthThigh * t1426 * t1427 * t1428 * t1487;
  const double t1541 = lengthShank * t1431 * t1424 * t1426 * t1428 * t1488;
  const double t1542 = lengthShank * t1430 * t1424 * t1426 * t1429 * t1488;
  const double t1543 = lengthShank * t1430 * t1426 * t1427 * t1428 * t1487;
  const double t1544 = lengthShank * t1431 * t1424 * t1425 * t1427 * t1429 * t1488;
  const double t1545 = t1540 + t1532 + t1541 + t1533 + t1542 + t1534 + t1543 + t1535 + t1544 + t1536 + t1537 + t1538 + t1539 -
                       baseToHipY * t1426 * t1487 - lengthHip * t1424 * t1425 * t1427 * t1488 -
                       lengthShank * t1431 * t1426 * t1427 * t1429 * t1487 - lengthThigh * t1424 * t1425 * t1427 * t1428 * t1488 -
                       lengthShank * t1430 * t1424 * t1425 * t1427 * t1428 * t1488;
  const double t1546 = baseToHipY * t1425 * t1488;
  const double t1547 = baseToHipX * t1426 * t1488;
  const double t1548 = lengthThigh * t1426 * t1429 * t1488;
  const double t1549 = lengthShank * t1431 * t1426 * t1428 * t1488;
  const double t1550 = lengthShank * t1430 * t1426 * t1429 * t1488;
  const double t1551 = lengthShank * t1431 * t1425 * t1427 * t1429 * t1488;
  const double t1552 = lengthShank * t1431 * t1432 * t1427 * t1428 * t1487;
  const double t1553 = lengthShank * t1430 * t1432 * t1427 * t1429 * t1487;
  const double t1554 = lengthShank * t1431 * t1424 * t1433 * t1426 * t1428 * t1487;
  const double t1555 = lengthShank * t1430 * t1424 * t1433 * t1426 * t1429 * t1487;
  const double t1556 = lengthHip * t1424 * t1427 * t1487;
  const double t1557 = lengthThigh * t1424 * t1427 * t1428 * t1487;
  const double t1558 = lengthShank * t1430 * t1424 * t1427 * t1428 * t1487;
  const double t1559 = lengthShank * t1431 * t1432 * t1433 * t1426 * t1429 * t1487;
  const double t1560 = t1556 + t1557 + t1558 + t1559 - lengthHip * t1432 * t1433 * t1426 * t1487 -
                       lengthShank * t1431 * t1424 * t1427 * t1429 * t1487 - lengthThigh * t1432 * t1433 * t1426 * t1428 * t1487 -
                       lengthShank * t1430 * t1432 * t1433 * t1426 * t1428 * t1487;
  const double t1561 = lengthHip * t1432 * t1427 * t1488;
  const double t1562 = lengthHip * t1433 * t1425 * t1487;
  const double t1563 = lengthThigh * t1432 * t1427 * t1428 * t1488;
  const double t1564 = lengthThigh * t1433 * t1425 * t1428 * t1487;
  const double t1565 = lengthHip * t1424 * t1433 * t1426 * t1488;
  const double t1566 = lengthShank * t1430 * t1432 * t1427 * t1428 * t1488;
  const double t1567 = lengthShank * t1430 * t1433 * t1425 * t1428 * t1487;
  const double t1568 = lengthThigh * t1424 * t1433 * t1426 * t1428 * t1488;
  const double t1569 = lengthShank * t1430 * t1424 * t1433 * t1426 * t1428 * t1488;
  const double t1570 = t1561 + t1562 + t1563 + t1564 + t1565 + t1566 + t1567 + t1568 + t1569 -
                       lengthShank * t1431 * t1433 * t1425 * t1429 * t1487 - lengthShank * t1431 * t1432 * t1427 * t1429 * t1488 -
                       lengthShank * t1431 * t1424 * t1433 * t1426 * t1429 * t1488;
  const double t1571 = lengthHip * t1433 * t1426 * t1488;
  const double t1572 = lengthThigh * t1433 * t1426 * t1428 * t1488;
  const double t1573 = lengthHip * t1424 * t1433 * t1425 * t1487;
  const double t1574 = lengthShank * t1430 * t1433 * t1426 * t1428 * t1488;
  const double t1575 = lengthThigh * t1424 * t1433 * t1425 * t1428 * t1487;
  const double t1576 = lengthShank * t1430 * t1424 * t1433 * t1425 * t1428 * t1487;
  const double t1577 = t1571 + t1572 + t1573 + t1574 + t1575 + t1576 - lengthShank * t1431 * t1433 * t1426 * t1429 * t1488 -
                       lengthShank * t1431 * t1424 * t1433 * t1425 * t1429 * t1487;
  const double t1578 = lengthThigh * t1432 * t1427 * t1429 * t1487;
  const double t1579 = lengthThigh * t1424 * t1433 * t1426 * t1429 * t1487;
  const double t1604 = lengthShank * t1431 * t1433 * t1425 * t1428 * t1488;
  const double t1605 = lengthShank * t1430 * t1433 * t1425 * t1429 * t1488;
  const double t1580 = t1552 + t1553 + t1554 + t1555 + t1578 + t1579 - t1604 - t1605 - lengthThigh * t1433 * t1425 * t1429 * t1488;
  const double t1581 = lengthThigh * t1424 * t1433 * t1429 * t1487;
  const double t1582 = lengthThigh * t1432 * t1426 * t1427 * t1429 * t1487;
  const double t1606 = lengthShank * t1430 * t1432 * t1425 * t1428 * t1487;
  const double t1583 = t1495 + t1496 + t1497 + t1498 + t1499 + t1581 + t1582 - t1606 - lengthThigh * t1432 * t1425 * t1428 * t1487;
  const double t1584 = lengthThigh * t1424 * t1426 * t1428 * t1487;
  const double t1585 = lengthThigh * t1426 * t1427 * t1429 * t1488;
  const double t1586 = lengthThigh * t1424 * t1425 * t1427 * t1429 * t1487;
  const double t1597 = lengthShank * t1430 * t1425 * t1428 * t1488;
  const double t1598 = lengthShank * t1431 * t1424 * t1426 * t1429 * t1487;
  const double t1587 =
      t1520 + t1521 + t1522 + t1523 + t1524 + t1519 + t1584 + t1585 + t1586 - t1597 - t1598 - lengthThigh * t1425 * t1428 * t1488;
  const double t1588 = lengthShank * t1430 * t1432 * t1433 * t1428 * t1487;
  const double t1589 = lengthShank * t1430 * t1425 * t1427 * t1428 * t1488;
  const double t1590 = lengthShank * t1431 * t1424 * t1426 * t1427 * t1429 * t1487;
  const double t1591 = lengthThigh * t1426 * t1428 * t1487;
  const double t1592 = lengthThigh * t1425 * t1427 * t1429 * t1487;
  const double t1593 = lengthThigh * t1424 * t1426 * t1427 * t1429 * t1488;
  const double t1599 = lengthShank * t1431 * t1426 * t1429 * t1487;
  const double t1600 = lengthShank * t1430 * t1424 * t1425 * t1428 * t1488;
  const double t1601 = lengthShank * t1431 * t1432 * t1433 * t1428 * t1488;
  const double t1602 = lengthShank * t1430 * t1432 * t1433 * t1429 * t1488;
  const double t1594 = t1490 + t1491 + t1492 + t1493 + t1494 + t1489 + t1591 + t1592 + t1593 - t1599 - t1600 - t1601 - t1602 -
                       lengthThigh * t1432 * t1433 * t1429 * t1488 - lengthThigh * t1424 * t1425 * t1428 * t1488;
  const double t1595 = lengthThigh * t1432 * t1433 * t1428 * t1487;
  const double t1596 = lengthThigh * t1425 * t1427 * t1428 * t1488;
  const double t1603 = t1514 + t1550 + t1515 + t1551 + t1516 + t1518 - t1590 + t1549 - t1588 - t1589;
  const double t1607 = lengthShank * t1430 * t1426 * t1428 * t1488;
  const double t1608 = lengthShank * t1430 * t1424 * t1425 * t1428 * t1487;
  const double t1609 = lengthShank * t1431 * t1432 * t1433 * t1428 * t1487;
  const double t1610 = lengthShank * t1430 * t1432 * t1433 * t1429 * t1487;
  const double t1611 = lengthShank * t1431 * t1425 * t1427 * t1428 * t1488;
  const double t1612 = lengthShank * t1430 * t1425 * t1427 * t1429 * t1488;
  const double t1613 = lengthHip * t1432 * t1433 * t1488;
  const double t1614 = baseToHipY * t1424 * t1426 * t1488;
  const double t1615 = lengthThigh * t1432 * t1433 * t1428 * t1488;
  const double t1616 = lengthShank * t1430 * t1432 * t1433 * t1428 * t1488;
  const double t1617 = lengthShank * t1431 * t1424 * t1426 * t1427 * t1429 * t1488;
  const double t1618 = baseToHipX * t1432 * t1425 * t1487;
  const double t1619 = lengthHip * t1424 * t1433 * t1487;
  const double t1620 = lengthThigh * t1432 * t1425 * t1429 * t1487;
  const double t1621 = lengthThigh * t1424 * t1433 * t1428 * t1487;
  const double t1622 = lengthHip * t1432 * t1426 * t1427 * t1487;
  const double t1623 = lengthShank * t1431 * t1432 * t1425 * t1428 * t1487;
  const double t1624 = lengthShank * t1430 * t1432 * t1425 * t1429 * t1487;
  const double t1625 = lengthShank * t1430 * t1424 * t1433 * t1428 * t1487;
  const double t1626 = lengthThigh * t1432 * t1426 * t1427 * t1428 * t1487;
  const double t1627 = lengthShank * t1430 * t1432 * t1426 * t1427 * t1428 * t1487;
  const double t1628 = t1620 + t1621 + t1622 + t1623 + t1624 + t1625 + t1626 + t1618 + t1627 + t1619 - baseToHipY * t1432 * t1426 * t1487 -
                       lengthShank * t1431 * t1424 * t1433 * t1429 * t1487 - lengthShank * t1431 * t1432 * t1426 * t1427 * t1429 * t1487;
  const double t1629 = lengthShank * t1431 * t1432 * t1425 * t1429 * t1488;
  const double t1630 = lengthShank * t1431 * t1424 * t1433 * t1428 * t1488;
  const double t1631 = lengthShank * t1430 * t1424 * t1433 * t1429 * t1488;
  const double t1632 = lengthShank * t1431 * t1432 * t1426 * t1427 * t1428 * t1488;
  const double t1633 = lengthShank * t1430 * t1432 * t1426 * t1427 * t1429 * t1488;
  const double t1634 = lengthShank * t1430 * t1425 * t1428 * t1487;
  const double t1635 = lengthShank * t1430 * t1424 * t1426 * t1428 * t1488;
  const double t1636 = lengthShank * t1431 * t1424 * t1425 * t1427 * t1428 * t1488;
  const double t1637 = lengthShank * t1430 * t1424 * t1425 * t1427 * t1429 * t1488;
  const double t1638 = baseToHipX * t1425 * t1488;
  const double t1639 = lengthThigh * t1425 * t1429 * t1488;
  const double t1640 = lengthHip * t1426 * t1427 * t1488;
  const double t1641 = lengthShank * t1431 * t1425 * t1428 * t1488;
  const double t1642 = lengthShank * t1430 * t1425 * t1429 * t1488;
  const double t1643 = lengthThigh * t1426 * t1427 * t1428 * t1488;
  const double t1644 = lengthHip * t1424 * t1425 * t1427 * t1487;
  const double t1645 = lengthShank * t1430 * t1426 * t1427 * t1428 * t1488;
  const double t1646 = lengthThigh * t1424 * t1425 * t1427 * t1428 * t1487;
  const double t1647 = lengthShank * t1430 * t1424 * t1425 * t1427 * t1428 * t1487;
  const double t1648 = t1640 + t1641 + t1642 + t1643 + t1644 + t1645 + t1646 + t1638 + t1647 + t1639 - baseToHipY * t1426 * t1488 -
                       baseToHipX * t1424 * t1426 * t1487 - baseToHipY * t1424 * t1425 * t1487 -
                       lengthThigh * t1424 * t1426 * t1429 * t1487 - lengthShank * t1430 * t1424 * t1426 * t1429 * t1487 -
                       lengthShank * t1431 * t1424 * t1426 * t1428 * t1487 - lengthShank * t1431 * t1426 * t1427 * t1429 * t1488 -
                       lengthShank * t1431 * t1424 * t1425 * t1427 * t1429 * t1487;
  const double t1649 = baseToHipY * t1425 * t1487;
  const double t1650 = baseToHipX * t1426 * t1487;
  const double t1651 = baseToHipX * t1424 * t1425 * t1488;
  const double t1652 = lengthThigh * t1426 * t1429 * t1487;
  const double t1653 = lengthThigh * t1424 * t1425 * t1429 * t1488;
  const double t1654 = lengthHip * t1424 * t1426 * t1427 * t1488;
  const double t1655 = lengthShank * t1431 * t1426 * t1428 * t1487;
  const double t1656 = lengthShank * t1430 * t1426 * t1429 * t1487;
  const double t1657 = lengthShank * t1431 * t1424 * t1425 * t1428 * t1488;
  const double t1658 = lengthShank * t1430 * t1424 * t1425 * t1429 * t1488;
  const double t1659 = lengthThigh * t1424 * t1426 * t1427 * t1428 * t1488;
  const double t1660 = lengthShank * t1431 * t1425 * t1427 * t1429 * t1487;
  const double t1661 = lengthShank * t1430 * t1424 * t1426 * t1427 * t1428 * t1488;
  const double t1662 = baseToHipY * t1432 * t1425 * t1488;
  const double t1663 = baseToHipX * t1432 * t1426 * t1488;
  const double t1664 = lengthThigh * t1432 * t1426 * t1429 * t1488;
  const double t1665 = lengthShank * t1431 * t1432 * t1426 * t1428 * t1488;
  const double t1666 = lengthShank * t1430 * t1432 * t1426 * t1429 * t1488;
  const double t1667 = lengthShank * t1431 * t1432 * t1425 * t1427 * t1429 * t1488;
  const double t1668 = t1662 + t1663 + t1664 + t1665 + t1666 + t1667 - lengthHip * t1432 * t1425 * t1427 * t1488 -
                       lengthThigh * t1432 * t1425 * t1427 * t1428 * t1488 - lengthShank * t1430 * t1432 * t1425 * t1427 * t1428 * t1488;
  const double t1669 = lengthShank * t1431 * t1432 * t1427 * t1428 * t1488;
  const double t1670 = lengthShank * t1430 * t1432 * t1427 * t1429 * t1488;
  const double t1671 = lengthShank * t1431 * t1433 * t1425 * t1428 * t1487;
  const double t1672 = lengthShank * t1430 * t1433 * t1425 * t1429 * t1487;
  const double t1673 = lengthShank * t1431 * t1424 * t1433 * t1426 * t1428 * t1488;
  const double t1674 = lengthShank * t1430 * t1424 * t1433 * t1426 * t1429 * t1488;
  const double t1675 = lengthHip * t1432 * t1427 * t1487;
  const double t1676 = lengthThigh * t1432 * t1427 * t1428 * t1487;
  const double t1677 = lengthHip * t1424 * t1433 * t1426 * t1487;
  const double t1678 = lengthShank * t1430 * t1432 * t1427 * t1428 * t1487;
  const double t1679 = lengthShank * t1431 * t1433 * t1425 * t1429 * t1488;
  const double t1680 = lengthThigh * t1424 * t1433 * t1426 * t1428 * t1487;
  const double t1681 = lengthShank * t1430 * t1424 * t1433 * t1426 * t1428 * t1487;
  const double t1682 = t1680 + t1681 + t1675 + t1676 + t1677 + t1678 + t1679 - lengthHip * t1433 * t1425 * t1488 -
                       lengthThigh * t1433 * t1425 * t1428 * t1488 - lengthShank * t1430 * t1433 * t1425 * t1428 * t1488 -
                       lengthShank * t1431 * t1432 * t1427 * t1429 * t1487 - lengthShank * t1431 * t1424 * t1433 * t1426 * t1429 * t1487;
  const double t1683 = lengthHip * t1425 * t1427 * t1487;
  const double t1684 = lengthThigh * t1425 * t1427 * t1428 * t1487;
  const double t1685 = lengthShank * t1431 * t1432 * t1433 * t1429 * t1488;
  const double t1686 = lengthShank * t1430 * t1425 * t1427 * t1428 * t1487;
  const double t1687 = lengthHip * t1424 * t1427 * t1488;
  const double t1688 = lengthThigh * t1424 * t1427 * t1428 * t1488;
  const double t1689 = lengthShank * t1430 * t1424 * t1427 * t1428 * t1488;
  const double t1690 = lengthShank * t1431 * t1432 * t1433 * t1426 * t1429 * t1488;
  const double t1691 = t1690 + t1687 + t1688 + t1689 - lengthHip * t1432 * t1433 * t1426 * t1488 -
                       lengthShank * t1431 * t1424 * t1427 * t1429 * t1488 - lengthThigh * t1432 * t1433 * t1426 * t1428 * t1488 -
                       lengthShank * t1430 * t1432 * t1433 * t1426 * t1428 * t1488;
  const double t1692 = lengthHip * t1433 * t1426 * t1487;
  const double t1693 = lengthThigh * t1433 * t1426 * t1428 * t1487;
  const double t1694 = lengthShank * t1430 * t1433 * t1426 * t1428 * t1487;
  const double t1695 = lengthShank * t1431 * t1424 * t1433 * t1425 * t1429 * t1488;
  const double t1696 = t1692 + t1693 + t1694 + t1695 - lengthHip * t1424 * t1433 * t1425 * t1488 -
                       lengthShank * t1431 * t1433 * t1426 * t1429 * t1487 - lengthThigh * t1424 * t1433 * t1425 * t1428 * t1488 -
                       lengthShank * t1430 * t1424 * t1433 * t1425 * t1428 * t1488;
  const double t1697 = lengthThigh * t1425 * t1428 * t1487;
  const double t1698 = lengthThigh * t1424 * t1426 * t1428 * t1488;
  const double t1699 = lengthThigh * t1424 * t1425 * t1427 * t1429 * t1488;
  const double t1713 = lengthShank * t1431 * t1425 * t1429 * t1487;
  const double t1714 = lengthShank * t1431 * t1424 * t1426 * t1429 * t1488;
  const double t1715 = lengthShank * t1431 * t1426 * t1427 * t1428 * t1487;
  const double t1716 = lengthShank * t1430 * t1426 * t1427 * t1429 * t1487;
  const double t1700 =
      t1634 + t1635 + t1636 + t1637 + t1697 + t1698 + t1699 - t1713 - t1714 - t1715 - t1716 - lengthThigh * t1426 * t1427 * t1429 * t1487;
  const double t1701 = lengthThigh * t1426 * t1428 * t1488;
  const double t1702 = lengthThigh * t1424 * t1425 * t1428 * t1487;
  const double t1703 = lengthThigh * t1432 * t1433 * t1429 * t1487;
  const double t1704 = lengthThigh * t1425 * t1427 * t1429 * t1488;
  const double t1720 = lengthShank * t1431 * t1426 * t1429 * t1488;
  const double t1721 = lengthShank * t1431 * t1424 * t1425 * t1429 * t1487;
  const double t1722 = lengthShank * t1431 * t1424 * t1426 * t1427 * t1428 * t1487;
  const double t1723 = lengthShank * t1430 * t1424 * t1426 * t1427 * t1429 * t1487;
  const double t1705 = t1610 + t1611 + t1612 + t1607 + t1608 + t1609 + t1701 + t1702 - t1720 + t1703 - t1721 + t1704 - t1722 - t1723 -
                       lengthThigh * t1424 * t1426 * t1427 * t1429 * t1487;
  const double t1706 = lengthThigh * t1424 * t1433 * t1429 * t1488;
  const double t1707 = lengthThigh * t1432 * t1426 * t1427 * t1429 * t1488;
  const double t1717 = lengthShank * t1430 * t1432 * t1425 * t1428 * t1488;
  const double t1708 = t1630 + t1631 + t1632 + t1633 + t1629 + t1706 + t1707 - t1717 - lengthThigh * t1432 * t1425 * t1428 * t1488;
  const double t1709 = lengthThigh * t1432 * t1427 * t1429 * t1488;
  const double t1710 = lengthThigh * t1433 * t1425 * t1429 * t1487;
  const double t1711 = lengthThigh * t1424 * t1433 * t1426 * t1429 * t1488;
  const double t1712 = t1670 + t1671 + t1672 + t1673 + t1674 + t1669 + t1710 + t1711 + t1709;
  const double t1718 = t1660 + t1616 - t1661 + t1617 + t1655 + t1656 - t1657 - t1658 - t1685 - t1686;
  const double t1719 = t1670 + t1671 + t1672 + t1673 + t1674 + t1669;
  // t0 = jacobianDerivative(0, 4) =
  /* DqGA* t1443 - DqLF_HAA* t1460 - DqLF_HFE* t1475 -
      DqLF_KFE*(t1434 + t1435 + t1436 - lengthShank * t1431 * t1424 * t1425 * t1429 - lengthShank * t1430 * t1424 * t1426 * t1427 * t1429 -
                lengthShank * t1431 * t1424 * t1426 * t1427 * t1428) -
      DqBE*(t1450 + t1461 + t1444 + t1462 + t1445 + t1463 + t1446 + t1447 + t1448 + t1449 - baseToHipY * t1432 * t1426 -
            lengthShank * t1431 * t1424 * t1433 * t1429 - lengthShank * t1431 * t1432 * t1426 * t1427 * t1429); */
  jacobianDerivative(0, 5) = DqBE * t1443 + DqLF_HAA * t1467 - DqLF_HFE * t1472 -
                             DqGA * (t1450 + t1444 + t1445 + t1446 + t1447 + t1448 + t1449 - baseToHipY * t1432 * t1426 -
                                     lengthShank * t1431 * t1432 * t1426 * t1427 * t1429) -
                             DqLF_KFE * (t1451 + t1452 + t1453 - lengthShank * t1431 * t1432 * t1426 * t1429);
  jacobianDerivative(0, 6) = DqLF_KFE * (t1468 + t1469 - lengthShank * t1430 * t1432 * t1433 * t1426 * t1429 -
                                         lengthShank * t1431 * t1432 * t1433 * t1426 * t1428) -
                             DqBE * t1460 + DqGA * t1467 + DqLF_HFE * t1477 -
                             DqLF_HAA * (t1450 + t1461 + t1462 + t1463 + t1446 + t1449 - lengthShank * t1431 * t1424 * t1433 * t1429 -
                                         lengthShank * t1431 * t1432 * t1426 * t1427 * t1429);
  jacobianDerivative(0, 7) = -DqBE * t1475 - DqGA * t1472 + DqLF_HAA * t1477 - DqLF_KFE * t1478 -
                             DqLF_HFE * (t1450 + t1462 + t1445 + t1463 + t1447 + t1448 + t1449 -
                                         lengthShank * t1431 * t1424 * t1433 * t1429 - lengthShank * t1431 * t1432 * t1426 * t1427 * t1429);
  jacobianDerivative(0, 8) = -DqLF_HFE * t1478 - DqLF_KFE * t1478 - DqGA * (t1451 + t1452 + t1453 - t1484) -
                             DqLF_HAA * (t1485 - t1468 + t1486 - t1469) - DqBE * (t1434 + t1435 + t1436 - t1481 - t1482 - t1483);
  jacobianDerivative(1, 3) =
      DqBE * t1510 - DqGA * t1545 + DqLF_HAA * t1570 - DqLF_HFE * t1594 -
      DqLF_KFE * (t1490 + t1491 + t1492 + t1493 + t1494 + t1489 - lengthShank * t1431 * t1426 * t1429 * t1487 -
                  lengthShank * t1430 * t1432 * t1433 * t1429 * t1488 - lengthShank * t1431 * t1432 * t1433 * t1428 * t1488 -
                  lengthShank * t1430 * t1424 * t1425 * t1428 * t1488) -
      DqAL * (t1511 + t1512 + t1513 + t1514 + t1550 + t1515 + t1551 + t1516 + t1517 + t1518 + t1546 + t1547 + t1548 + t1549 -
              baseToHipY * t1424 * t1426 * t1487 - lengthHip * t1432 * t1433 * t1487 - lengthHip * t1425 * t1427 * t1488 -
              lengthThigh * t1432 * t1433 * t1428 * t1487 - lengthThigh * t1425 * t1427 * t1428 * t1488 -
              lengthShank * t1430 * t1432 * t1433 * t1428 * t1487 - lengthShank * t1430 * t1425 * t1427 * t1428 * t1488 -
              lengthShank * t1431 * t1424 * t1426 * t1427 * t1429 * t1487);
  jacobianDerivative(1, 4) =
      DqAL * t1510 - DqGA * t1531 - DqLF_HAA * t1560 - DqLF_HFE * t1583 -
      DqBE * (t1511 + t1512 + t1513 + t1514 + t1515 + t1516 + t1517 + t1518 - baseToHipY * t1424 * t1426 * t1487 -
              lengthHip * t1432 * t1433 * t1487 - lengthThigh * t1432 * t1433 * t1428 * t1487 -
              lengthShank * t1430 * t1432 * t1433 * t1428 * t1487 - lengthShank * t1431 * t1424 * t1426 * t1427 * t1429 * t1487) -
      DqLF_KFE * (t1495 + t1496 + t1497 + t1498 + t1499 - lengthShank * t1430 * t1432 * t1425 * t1428 * t1487);
  jacobianDerivative(1, 5) =
      -DqAL * t1545 - DqBE * t1531 + DqLF_HAA * t1577 - DqLF_HFE * t1587 -
      DqGA * (t1511 + t1512 + t1513 + t1514 + t1550 + t1515 + t1551 + t1517 + t1518 + t1546 + t1547 + t1548 + t1549 -
              baseToHipY * t1424 * t1426 * t1487 - lengthHip * t1425 * t1427 * t1488 - lengthThigh * t1425 * t1427 * t1428 * t1488 -
              lengthShank * t1430 * t1425 * t1427 * t1428 * t1488 - lengthShank * t1431 * t1424 * t1426 * t1427 * t1429 * t1487) -
      DqLF_KFE * (t1520 + t1521 + t1522 + t1523 + t1524 + t1519 - lengthShank * t1430 * t1425 * t1428 * t1488 -
                  lengthShank * t1431 * t1424 * t1426 * t1429 * t1487);
  jacobianDerivative(1, 6) = DqAL * t1570 - DqBE * t1560 + DqGA * t1577 - DqLF_HFE * t1580 +
                             DqLF_HAA * (-t1513 - t1551 - t1516 - t1517 - t1518 + t1590 + t1595 + t1596 + t1588 + t1589 +
                                         lengthHip * t1432 * t1433 * t1487 + lengthHip * t1425 * t1427 * t1488) -
                             DqLF_KFE * (t1552 + t1553 + t1554 + t1555 - lengthShank * t1430 * t1433 * t1425 * t1429 * t1488 -
                                         lengthShank * t1431 * t1433 * t1425 * t1428 * t1488);
  jacobianDerivative(1, 7) =
      -DqAL * t1594 - DqBE * t1583 - DqGA * t1587 - DqLF_HAA * t1580 - DqLF_KFE * t1603 -
      DqLF_HFE * (t1512 + t1514 + t1550 + t1515 + t1551 + t1516 + t1517 + t1518 - t1590 + t1548 + t1549 - t1595 - t1596 - t1588 - t1589);
  jacobianDerivative(1, 8) = -DqAL * (t1490 + t1491 + t1492 + t1493 + t1494 + t1489 - t1599 - t1600 - t1601 - t1602) - DqLF_HFE * t1603 -
                             DqLF_KFE * t1603 - DqBE * (t1495 + t1496 + t1497 + t1498 + t1499 - t1606) -
                             DqGA * (t1520 + t1521 + t1522 + t1523 + t1524 + t1519 - t1597 - t1598) -
                             DqLF_HAA * (t1552 + t1553 + t1554 + t1555 - t1604 - t1605);
  jacobianDerivative(2, 3) =
      DqBE * t1628 + DqGA * t1648 + DqLF_HAA * t1682 + DqLF_HFE * t1705 +
      DqLF_KFE * (t1610 + t1611 + t1612 + t1607 + t1608 + t1609 - lengthShank * t1431 * t1426 * t1429 * t1488 -
                  lengthShank * t1431 * t1424 * t1425 * t1429 * t1487 - lengthShank * t1430 * t1424 * t1426 * t1427 * t1429 * t1487 -
                  lengthShank * t1431 * t1424 * t1426 * t1427 * t1428 * t1487) -
      DqAL * (t1613 + t1614 + t1650 + t1615 + t1660 + t1616 + t1652 + t1617 + t1655 + t1656 + t1649 - baseToHipX * t1424 * t1425 * t1488 -
              lengthHip * t1425 * t1427 * t1487 - lengthHip * t1424 * t1426 * t1427 * t1488 - lengthThigh * t1424 * t1425 * t1429 * t1488 -
              lengthThigh * t1425 * t1427 * t1428 * t1487 - lengthShank * t1431 * t1432 * t1433 * t1429 * t1488 -
              lengthShank * t1430 * t1424 * t1425 * t1429 * t1488 - lengthShank * t1431 * t1424 * t1425 * t1428 * t1488 -
              lengthShank * t1430 * t1425 * t1427 * t1428 * t1487 - lengthThigh * t1424 * t1426 * t1427 * t1428 * t1488 -
              lengthShank * t1430 * t1424 * t1426 * t1427 * t1428 * t1488);
  jacobianDerivative(2, 4) =
      DqAL * t1628 + DqGA * t1668 + DqLF_HAA * t1691 + DqLF_HFE * t1708 +
      DqLF_KFE * (t1630 + t1631 + t1632 + t1633 + t1629 - lengthShank * t1430 * t1432 * t1425 * t1428 * t1488) +
      DqBE * (-t1613 - t1614 - t1615 + t1651 - t1616 + t1661 - t1617 + t1653 + t1654 + t1657 + t1658 + t1685 + t1659);
  jacobianDerivative(2, 5) =
      DqAL * t1648 + DqBE * t1668 + DqLF_HAA * t1696 + DqLF_HFE * t1700 +
      DqGA * (-t1614 - t1650 + t1651 - t1660 - t1652 + t1661 - t1617 + t1653 + t1654 - t1655 - t1656 + t1683 + t1657 + t1684 - t1649 +
              t1658 + t1659 + t1686) +
      DqLF_KFE * (t1634 + t1635 + t1636 + t1637 - lengthShank * t1431 * t1425 * t1429 * t1487 -
                  lengthShank * t1431 * t1424 * t1426 * t1429 * t1488 - lengthShank * t1430 * t1426 * t1427 * t1429 * t1487 -
                  lengthShank * t1431 * t1426 * t1427 * t1428 * t1487);
  jacobianDerivative(2, 6) = DqAL * t1682 + DqBE * t1691 + DqGA * t1696 + DqLF_HFE * t1712 + DqLF_KFE * t1719 +
                             DqLF_HAA * (-t1613 - t1615 - t1660 - t1616 + t1661 - t1617 + t1654 + t1683 + t1684 + t1685 + t1659 + t1686);
  jacobianDerivative(2, 7) =
      DqAL * t1705 + DqBE * t1708 + DqGA * t1700 + DqLF_HAA * t1712 - DqLF_KFE * t1718 +
      DqLF_HFE * (-t1615 - t1660 - t1616 - t1652 + t1661 - t1617 + t1653 - t1655 - t1656 + t1657 + t1684 + t1658 + t1685 + t1659 + t1686);
  jacobianDerivative(2, 8) = DqAL * (t1610 + t1611 + t1612 + t1607 + t1608 + t1609 - t1720 - t1721 - t1722 - t1723) + DqLF_HAA * t1719 -
                             DqLF_HFE * t1718 - DqLF_KFE * t1718 + DqBE * (t1630 + t1631 + t1632 + t1633 + t1629 - t1717) +
                             DqGA * (t1634 + t1635 + t1636 + t1637 - t1713 - t1714 - t1715 - t1716);

  return jacobianDerivative;
}

}  // namespace starleth_kinematics
