/*
 * conversions.cpp
 *
 *  Created on: Apr 2, 2019
 *      Author: Dario Bellicoso
 */

#include <motion_generation_utils/conversions.hpp>

namespace zmp {

Eigen::Matrix3d getMapEulerAnglesZyxToAngularVelocityInInertialFrame(const motion_generation::EulerAnglesZyx& orientationBodyToInertial) {
  const double yaw = orientationBodyToInertial.yaw();
  const double pitch = orientationBodyToInertial.pitch();

  const double t2 = std::cos(yaw);
  const double t3 = std::cos(pitch);
  const double t4 = std::sin(yaw);
  Eigen::Matrix3d matrix;
  matrix << 0.0, -t4, t2*t3,
            0.0,  t2, t3*t4,
            1.0, 0.0, -std::sin(pitch);

  return matrix;
}

Eigen::Matrix3d getTimeDerivativeMapEulerAnglesZyxToAngularVelocityInInertialFrame(
   const motion_generation::EulerAnglesZyx& orientationBodyToInertial,
   const motion_generation::EulerAnglesZyxDiff& orientationDiffBodyToInertial) {
 const double yawDot = orientationDiffBodyToInertial.yaw();
 const double pitchDot = orientationDiffBodyToInertial.pitch();
 const double yaw = orientationBodyToInertial.yaw();
 const double pitch = orientationBodyToInertial.pitch();

 const double t2 = std::cos(yaw);
 const double t3 = std::sin(yaw);
 const double t4 = std::cos(pitch);
 const double t5 = std::sin(pitch);
 Eigen::Matrix3d matrix;
 matrix.col(0) << 0.0,
                  0.0,
                  0.0;
 matrix.col(1) << -yawDot*t2,
                  -yawDot*t3,
                         0.0;
 matrix.col(2) << -yawDot*t3*t4-pitchDot*t2*t5,
                   yawDot*t2*t4-pitchDot*t3*t5,
                                  -pitchDot*t4;

 return matrix;
}

Eigen::Matrix3d getMapEulerAnglesZyxToAngularVelocityInBodyFrame(const motion_generation::EulerAnglesZyx& orientationBodyToWorld) {
  const double cosPitch  = std::cos(orientationBodyToWorld.pitch());
  const double sinPitch  = std::sin(orientationBodyToWorld.pitch());
  const double cosRoll   = std::cos(orientationBodyToWorld.roll());
  const double sinRoll   = std::sin(orientationBodyToWorld.roll());

  Eigen::Matrix3d matrix;
  matrix <<  -sinPitch,          0.0,     1.0,
              cosPitch*sinRoll,  cosRoll, 0.0,
              cosRoll*cosPitch, -sinRoll, 0.0;
  return matrix;
}

Eigen::Matrix3d getMatrixAngularRatesZyxToAngularVelocityInBodyFrameTimeDerivative(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame) {
  const double cosPitch  = std::cos(orientationBodyToWorld.pitch());
  const double sinPitch  = std::sin(orientationBodyToWorld.pitch());
  const double cosRoll   = std::cos(orientationBodyToWorld.roll());
  const double sinRoll   = std::sin(orientationBodyToWorld.roll());

  Eigen::Matrix3d matrix;
  matrix << -cosPitch*eulerRatesZyxBodyInWorldFrame.pitch(),                                                               0.0,                                             0.0,
            -sinPitch*sinRoll*eulerRatesZyxBodyInWorldFrame.pitch() + cosRoll*cosPitch*eulerRatesZyxBodyInWorldFrame.roll(), -sinRoll*eulerRatesZyxBodyInWorldFrame.roll(), 0.0,
            -cosRoll*sinPitch*eulerRatesZyxBodyInWorldFrame.pitch() - sinRoll*cosPitch*eulerRatesZyxBodyInWorldFrame.roll(), -cosRoll*eulerRatesZyxBodyInWorldFrame.roll(), 0.0;
  return matrix;
}


Eigen::Matrix3d getMatrixAngularVelocityInInertialFrameToAngularRatesZyx(const motion_generation::EulerAnglesZyx& orientationBodyToWorld) {
  const double cosYaw    = std::cos(orientationBodyToWorld.yaw());
  const double sinYaw    = std::sin(orientationBodyToWorld.yaw());
  const double cosPitch  = std::cos(orientationBodyToWorld.pitch());
  const double sinPitch  = std::sin(orientationBodyToWorld.pitch());

  /*
   * https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
   * Equation 2.87.
   * See also https://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf.
   */
  Eigen::Matrix3d matrix;
  if (cosPitch != 0.0){

    matrix <<
        cosYaw*sinPitch/cosPitch, sinYaw*sinPitch/cosPitch,  1.0,
        -sinYaw,                  cosYaw,                    0.0,
        cosYaw/cosPitch,          sinYaw/cosPitch,           0.0;
  } else {
    std::cout << "[TrajectoryStateHandlerBase::getMatrixAngularVelocityInInertialFrameToAngularRatesZyx] Cannot invert singular matrix.";
    matrix.setZero();
  }
  return matrix;
}

Eigen::Matrix3d getMatrixAngularVelocityInInertialFrameToAngularRatesZyxTimeDerivative(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame) {
  const double cosPitch  = std::cos(orientationBodyToWorld.pitch());
  const double sinPitch  = std::sin(orientationBodyToWorld.pitch());
  const double cosYaw    = std::cos(orientationBodyToWorld.yaw());
  const double sinYaw    = std::sin(orientationBodyToWorld.yaw());
  const double yaw_dot   = eulerRatesZyxBodyInWorldFrame.yaw();
  const double pitch_dot = eulerRatesZyxBodyInWorldFrame.pitch();
  const double cosPitchSquared = cosPitch*cosPitch;

  Eigen::Matrix3d matrix;
  if (cosPitchSquared != 0.0){
    matrix <<
        (-sinYaw*sinPitch*cosPitch*yaw_dot+cosYaw*pitch_dot)/cosPitchSquared, (cosYaw*sinPitch*cosPitch*yaw_dot+sinYaw*pitch_dot)/cosPitchSquared,  0.0,
        -cosYaw*yaw_dot,                                                       -sinYaw*yaw_dot,                                                     0.0,
        (-sinYaw*cosPitch*yaw_dot+cosYaw*sinPitch*pitch_dot)/cosPitchSquared,  (cosYaw*cosPitch*yaw_dot+sinYaw*sinPitch*pitch_dot)/cosPitchSquared, 0.0;
  } else {
    std::cout << "[TrajectoryStateHandlerBase::getMatrixAngularVelocityInInertialFrameToAngularRatesZyxTimeDerivative] Cannot invert singular matrix." << std::endl;
    matrix.setZero();
  }

  return matrix;

}

Eigen::Matrix3d getMatrixAngularVelocityInBodyFrameToAngularRatesZyx(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld) {
  const double cosPitch  = std::cos(orientationBodyToWorld.pitch());
  const double sinPitch  = std::sin(orientationBodyToWorld.pitch());
  const double cosRoll   = std::cos(orientationBodyToWorld.roll());
  const double sinRoll   = std::sin(orientationBodyToWorld.roll());

  /*
   * https://www.ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
   * Equation 2.87.
   * See also https://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf.
   */
  Eigen::Matrix3d matrix;
  if (cosPitch != 0.0){
    matrix << 0.0,    sinRoll/cosPitch,          cosRoll/cosPitch,
              0.0,    cosRoll,                  -sinRoll,
              1.0,    sinRoll*sinPitch/cosPitch, cosRoll*sinPitch/cosPitch;
  } else {
    std::cout << "[TrajectoryStateHandlerBase::getMatrixAngularVelocityInBodyFrameToAngularRatesZyx] Cannot invert singular matrix." << std::endl;
    matrix.setZero();
  }
  return matrix;
}

Eigen::Matrix3d getMatrixAngularVelocityInBodyFrameToAngularRatesZyxTimeDerivative(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame) {
  const double cosPitch  = std::cos(orientationBodyToWorld.pitch());
  const double sinPitch  = std::sin(orientationBodyToWorld.pitch());
  const double cosRoll   = std::cos(orientationBodyToWorld.roll());
  const double sinRoll   = std::sin(orientationBodyToWorld.roll());
  const double roll_dot  = eulerRatesZyxBodyInWorldFrame.roll();
  const double pitch_dot = eulerRatesZyxBodyInWorldFrame.pitch();
  const double cosPitchSquared = cosPitch*cosPitch;

  Eigen::Matrix3d matrix;
  if (cosPitchSquared != 0.0){
    matrix <<
          0.0, (cosRoll*cosPitch*roll_dot+sinRoll*sinPitch*pitch_dot)/cosPitchSquared, (-sinRoll*cosPitch*roll_dot+cosRoll*sinPitch*pitch_dot)/cosPitchSquared,
          0.0, -sinRoll*roll_dot,                                                       -cosRoll*roll_dot,
          0.0, (cosRoll*sinPitch*cosPitch*roll_dot+sinRoll*pitch_dot)/cosPitchSquared, (-sinRoll*sinPitch*cosPitch*roll_dot+cosRoll*pitch_dot)/cosPitchSquared;
  } else {
    std::cout << "[TrajectoryStateHandlerBase::getMatrixAngularVelocityInBodyFrameToAngularRatesZyxTimeDerivative] Cannot invert singular matrix." << std::endl;
    matrix.setZero();
  }
  return matrix;
}

// Jacobian of angular velocity in body frame w.r.t to euler angles.
// Equation 3.19 in Thomas report.
Eigen::Matrix3d getDerivativeAngularVelocityWrtEulerAngles(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame) {
  const double cosPitch  = std::cos(orientationBodyToWorld.pitch());
  const double sinPitch  = std::sin(orientationBodyToWorld.pitch());
  const double cosRoll   = std::cos(orientationBodyToWorld.roll());
  const double sinRoll   = std::sin(orientationBodyToWorld.roll());

  Eigen::Matrix3d matrix;
  matrix <<  0.0, -cosPitch*eulerRatesZyxBodyInWorldFrame.yaw()       ,   0.0,
             0.0, -sinPitch*sinRoll*eulerRatesZyxBodyInWorldFrame.yaw(),  cosRoll*cosPitch*eulerRatesZyxBodyInWorldFrame.yaw()-sinRoll*eulerRatesZyxBodyInWorldFrame.pitch(),
             0.0, -sinPitch*cosRoll*eulerRatesZyxBodyInWorldFrame.yaw(), -sinRoll*cosPitch*eulerRatesZyxBodyInWorldFrame.yaw()-cosRoll*eulerRatesZyxBodyInWorldFrame.pitch();
  return matrix;
}

Eigen::Matrix3d getDerivativeLdotWrtEulerAngles(
  const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
  const motion_generation::EulerAnglesZyxDiff& eulerDiffZyxBodyInWorldFrame,
  const motion_generation::EulerAnglesZyxDiff& eulerDDiffZyxBodyInWorldFrame,
  const Eigen::Matrix3d& inertiaTensorInBodyFrame) {

  Eigen::Matrix3d mat;
  const double rollDDot = eulerDDiffZyxBodyInWorldFrame.roll();
  const double yawDDot = eulerDDiffZyxBodyInWorldFrame.yaw();
  const double pitchDDot = eulerDDiffZyxBodyInWorldFrame.pitch();
  const double rollDot = eulerDiffZyxBodyInWorldFrame.roll();
  const double yawDot = eulerDiffZyxBodyInWorldFrame.yaw();
  const double pitchDot = eulerDiffZyxBodyInWorldFrame.pitch();
  const double roll = orientationBodyToWorld.roll();
  const double yaw = orientationBodyToWorld.yaw();
  const double pitch = orientationBodyToWorld.pitch();
  const double B_i11 = inertiaTensorInBodyFrame(0,0);
  const double B_i22 = inertiaTensorInBodyFrame(1,1);
  const double B_i33 = inertiaTensorInBodyFrame(2,2);
  const double B_i12 = inertiaTensorInBodyFrame(0,1);
  const double B_i13 = inertiaTensorInBodyFrame(0,2);
  const double B_i23 = inertiaTensorInBodyFrame(1,2);
  const double t2 = cos(yaw);
  const double t3 = roll*2.0;
  const double t4 = cos(t3);
  const double t5 = rollDot*rollDot;
  const double t6 = pitch*2.0;
  const double t7 = sin(t6);
  const double t8 = sin(yaw);
  const double t9 = yawDot*yawDot;
  const double t10 = cos(roll);
  const double t11 = sin(roll);
  const double t12 = sin(pitch);
  const double t13 = sin(t3);
  const double t14 = pitchDot*pitchDot;
  const double t15 = cos(pitch);
  const double t16 = cos(t6);
  const double t17 = t10*t10;
  const double t18 = t15*t15;

  mat.col(0) << B_i22*pitchDDot*t2*(-1.0/2.0)-(B_i33*pitchDDot*t2)/2.0+(B_i22*yawDot*pitchDot*t8)/2.0+(B_i33*yawDot*pitchDot*t8)/2.0
              -B_i12*rollDDot*t2*t10+B_i13*rollDDot*t2*t11-B_i11*rollDDot*t8*t15+(B_i11*yawDDot*t7*t8)/2.0-(B_i22*yawDDot*t7*t8)/4.0
              -(B_i33*yawDDot*t7*t8)/4.0-(B_i22*pitchDDot*t2*t4)/2.0+B_i23*pitchDDot*t2*t13+(B_i33*pitchDDot*t2*t4)/2.0+(B_i11*t2*t7*t9)/2.0+B_i12*t2*t5*t11
              +B_i13*t2*t5*t10-(B_i22*t2*t7*t9)/4.0-(B_i33*t2*t7*t9)/4.0-B_i11*rollDot*yawDot*t2*t15+B_i12*rollDot*yawDot*t8*t10-B_i13*rollDot*yawDot*t8*t11
              +B_i23*rollDot*pitchDot*t2*t4*2.0+B_i11*rollDot*pitchDot*t8*t12+B_i22*rollDot*pitchDot*t2*t13-B_i33*rollDot*pitchDot*t2*t13
              +(B_i22*yawDot*pitchDot*t4*t8)/2.0+B_i11*yawDot*pitchDot*t8*t16-B_i23*yawDot*pitchDot*t8*t13-(B_i33*yawDot*pitchDot*t4*t8)/2.0
              -(B_i22*yawDot*pitchDot*t8*t16)/2.0-(B_i33*yawDot*pitchDot*t8*t16)/2.0-B_i12*rollDDot*t8*t11*t12-B_i13*rollDDot*t8*t10*t12+B_i12*yawDDot*t2*t10*t12-B_i13*yawDDot*t2*t11*t12
              +(B_i22*yawDDot*t4*t7*t8)/4.0-B_i23*yawDDot*t2*t4*t15-B_i12*yawDDot*t8*t11*t16-B_i13*yawDDot*t8*t10*t16-(B_i23*yawDDot*t7*t8*t13)/2.0
              -(B_i22*yawDDot*t2*t13*t15)/2.0-(B_i33*yawDDot*t4*t7*t8)/4.0+(B_i33*yawDDot*t2*t13*t15)/2.0-B_i12*pitchDDot*t8*t10*t15
              +B_i13*pitchDDot*t8*t11*t15-B_i23*pitchDDot*t4*t8*t12-(B_i22*pitchDDot*t8*t12*t13)/2.0+(B_i33*pitchDDot*t8*t12*t13)/2.0+(B_i22*t2*t4*t7*t9)/4.0
              -B_i12*t5*t8*t10*t12+B_i13*t5*t8*t11*t12-B_i12*t2*t9*t11*t16-B_i13*t2*t9*t10*t16-B_i12*t8*t9*t10*t12+B_i13*t8*t9*t11*t12-(B_i23*t2*t7*t9*t13)/2.0
              -(B_i33*t2*t4*t7*t9)/4.0+B_i12*t8*t10*t12*t14-B_i13*t8*t11*t12*t14+B_i23*t4*t8*t9*t15-B_i23*t4*t8*t14*t15+(B_i22*t8*t9*t13*t15)/2.0
              -(B_i22*t8*t13*t14*t15)/2.0-(B_i33*t8*t9*t13*t15)/2.0+(B_i33*t8*t13*t14*t15)/2.0-B_i12*rollDot*yawDot*t2*t11*t12*2.0-B_i13*rollDot*yawDot*t2*t10*t12*2.0
              -B_i23*rollDot*yawDot*t4*t7*t8-B_i22*rollDot*yawDot*t2*t4*t15-B_i12*rollDot*yawDot*t8*t10*t16+B_i13*rollDot*yawDot*t8*t11*t16
              -(B_i22*rollDot*yawDot*t7*t8*t13)/2.0+B_i23*rollDot*yawDot*t2*t13*t15*2.0+B_i33*rollDot*yawDot*t2*t4*t15+(B_i33*rollDot*yawDot*t7*t8*t13)/2.0
              -B_i22*rollDot*pitchDot*t4*t8*t12+B_i23*rollDot*pitchDot*t8*t12*t13*2.0+B_i33*rollDot*pitchDot*t4*t8*t12+B_i12*yawDot*pitchDot*t7*t8*t11*2.0
              +B_i13*yawDot*pitchDot*t7*t8*t10*2.0+(B_i22*yawDot*pitchDot*t4*t8*t16)/2.0-B_i23*yawDot*pitchDot*t8*t13*t16-(B_i33*yawDot*pitchDot*t4*t8*t16)/2.0,

              B_i22*pitchDDot*t8*(-1.0/2.0)-(B_i33*pitchDDot*t8)/2.0-(B_i22*yawDot*pitchDot*t2)/2.0-(B_i33*yawDot*pitchDot*t2)/2.0+B_i11*rollDDot*t2*t15-B_i12*rollDDot*t8*t10-(B_i11*yawDDot*t2*t7)/2.0+B_i13*rollDDot*t8*t11+(B_i22*yawDDot*t2*t7)/4.0+(B_i33*yawDDot*t2*t7)/4.0-(B_i22*pitchDDot*t4*t8)/2.0+B_i23*pitchDDot*t8*t13+(B_i33*pitchDDot*t4*t8)/2.0+(B_i11*t7*t8*t9)/2.0+B_i12*t5*t8*t11+B_i13*t5*t8*t10-(B_i22*t7*t8*t9)/4.0-(B_i33*t7*t8*t9)/4.0-B_i12*rollDot*yawDot*t2*t10+B_i13*rollDot*yawDot*t2*t11-B_i11*rollDot*yawDot*t8*t15-B_i11*rollDot*pitchDot*t2*t12+B_i23*rollDot*pitchDot*t4*t8*2.0+B_i22*rollDot*pitchDot*t8*t13-B_i33*rollDot*pitchDot*t8*t13-(B_i22*yawDot*pitchDot*t2*t4)/2.0-B_i11*yawDot*pitchDot*t2*t16+B_i23*yawDot*pitchDot*t2*t13+(B_i33*yawDot*pitchDot*t2*t4)/2.0+(B_i22*yawDot*pitchDot*t2*t16)/2.0+(B_i33*yawDot*pitchDot*t2*t16)/2.0+B_i12*rollDDot*t2*t11*t12+B_i13*rollDDot*t2*t10*t12-(B_i22*yawDDot*t2*t4*t7)/4.0+B_i12*yawDDot*t2*t11*t16+B_i13*yawDDot*t2*t10*t16+B_i12*yawDDot*t8*t10*t12-B_i13*yawDDot*t8*t11*t12+(B_i23*yawDDot*t2*t7*t13)/2.0+(B_i33*yawDDot*t2*t4*t7)/4.0-B_i23*yawDDot*t4*t8*t15-(B_i22*yawDDot*t8*t13*t15)/2.0+(B_i33*yawDDot*t8*t13*t15)/2.0+B_i12*pitchDDot*t2*t10*t15-B_i13*pitchDDot*t2*t11*t15+B_i23*pitchDDot*t2*t4*t12+(B_i22*pitchDDot*t2*t12*t13)/2.0-(B_i33*pitchDDot*t2*t12*t13)/2.0+B_i12*t2*t5*t10*t12-B_i13*t2*t5*t11*t12+B_i12*t2*t9*t10*t12-B_i13*t2*t9*t11*t12-B_i12*t2*t10*t12*t14+(B_i22*t4*t7*t8*t9)/4.0+B_i13*t2*t11*t12*t14-B_i23*t2*t4*t9*t15-B_i12*t8*t9*t11*t16-B_i13*t8*t9*t10*t16+B_i23*t2*t4*t14*t15-(B_i23*t7*t8*t9*t13)/2.0-(B_i22*t2*t9*t13*t15)/2.0-(B_i33*t4*t7*t8*t9)/4.0+(B_i22*t2*t13*t14*t15)/2.0+(B_i33*t2*t9*t13*t15)/2.0-(B_i33*t2*t13*t14*t15)/2.0+B_i23*rollDot*yawDot*t2*t4*t7+B_i12*rollDot*yawDot*t2*t10*t16-B_i13*rollDot*yawDot*t2*t11*t16-B_i12*rollDot*yawDot*t8*t11*t12*2.0-B_i13*rollDot*yawDot*t8*t10*t12*2.0+(B_i22*rollDot*yawDot*t2*t7*t13)/2.0-B_i22*rollDot*yawDot*t4*t8*t15-(B_i33*rollDot*yawDot*t2*t7*t13)/2.0+B_i23*rollDot*yawDot*t8*t13*t15*2.0+B_i33*rollDot*yawDot*t4*t8*t15+B_i22*rollDot*pitchDot*t2*t4*t12-B_i23*rollDot*pitchDot*t2*t12*t13*2.0-B_i33*rollDot*pitchDot*t2*t4*t12-B_i12*yawDot*pitchDot*t2*t7*t11*2.0-B_i13*yawDot*pitchDot*t2*t7*t10*2.0-(B_i22*yawDot*pitchDot*t2*t4*t16)/2.0+B_i23*yawDot*pitchDot*t2*t13*t16+(B_i33*yawDot*pitchDot*t2*t4*t16)/2.0,
              0.0;

  mat.col(1) << -B_i11*rollDDot*t2*t12-B_i11*yawDDot*t2*t16+(B_i22*yawDDot*t2*t16)/2.0+(B_i33*yawDDot*t2*t16)/2.0+B_i11*t8*t9*t16-(B_i22*t8*t9*t16)/2.0-(B_i33*t8*t9*t16)/2.0+B_i11*rollDot*yawDot*t8*t12-B_i11*rollDot*pitchDot*t2*t15+B_i11*yawDot*pitchDot*t2*t7*2.0-B_i22*yawDot*pitchDot*t2*t7-B_i33*yawDot*pitchDot*t2*t7+B_i12*rollDDot*t2*t11*t15+B_i13*rollDDot*t2*t10*t15-B_i12*yawDDot*t2*t7*t11*2.0-B_i13*yawDDot*t2*t7*t10*2.0-(B_i22*yawDDot*t2*t4*t16)/2.0+B_i12*yawDDot*t8*t10*t15-B_i13*yawDDot*t8*t11*t15+B_i23*yawDDot*t4*t8*t12+B_i23*yawDDot*t2*t13*t16+(B_i22*yawDDot*t8*t12*t13)/2.0+(B_i33*yawDDot*t2*t4*t16)/2.0-(B_i33*yawDDot*t8*t12*t13)/2.0-B_i12*pitchDDot*t2*t10*t12+B_i13*pitchDDot*t2*t11*t12+B_i23*pitchDDot*t2*t4*t15+(B_i22*pitchDDot*t2*t13*t15)/2.0-(B_i33*pitchDDot*t2*t13*t15)/2.0+B_i12*t2*t5*t10*t15-B_i13*t2*t5*t11*t15+B_i12*t7*t8*t9*t11*2.0+B_i13*t7*t8*t9*t10*2.0+B_i12*t2*t9*t10*t15-B_i13*t2*t9*t11*t15+B_i23*t2*t4*t9*t12-B_i12*t2*t10*t14*t15+B_i13*t2*t11*t14*t15-B_i23*t2*t4*t12*t14+(B_i22*t2*t9*t12*t13)/2.0+(B_i22*t4*t8*t9*t16)/2.0-(B_i22*t2*t12*t13*t14)/2.0-B_i23*t8*t9*t13*t16-(B_i33*t2*t9*t12*t13)/2.0-(B_i33*t4*t8*t9*t16)/2.0+(B_i33*t2*t12*t13*t14)/2.0-B_i12*rollDot*yawDot*t2*t7*t10*2.0+B_i13*rollDot*yawDot*t2*t7*t11*2.0+B_i23*rollDot*yawDot*t2*t4*t16*2.0-B_i12*rollDot*yawDot*t8*t11*t15*2.0-B_i13*rollDot*yawDot*t8*t10*t15*2.0+B_i22*rollDot*yawDot*t4*t8*t12+B_i22*rollDot*yawDot*t2*t13*t16-B_i23*rollDot*yawDot*t8*t12*t13*2.0-B_i33*rollDot*yawDot*t4*t8*t12-B_i33*rollDot*yawDot*t2*t13*t16+B_i22*rollDot*pitchDot*t2*t4*t15-B_i23*rollDot*pitchDot*t2*t13*t15*2.0-B_i33*rollDot*pitchDot*t2*t4*t15+B_i22*yawDot*pitchDot*t2*t4*t7-B_i12*yawDot*pitchDot*t2*t11*t16*4.0-B_i13*yawDot*pitchDot*t2*t10*t16*4.0-B_i23*yawDot*pitchDot*t2*t7*t13*2.0-B_i33*yawDot*pitchDot*t2*t4*t7,
                -B_i11*rollDDot*t8*t12-B_i11*yawDDot*t8*t16+(B_i22*yawDDot*t8*t16)/2.0+(B_i33*yawDDot*t8*t16)/2.0-B_i11*t2*t9*t16+(B_i22*t2*t9*t16)/2.0+(B_i33*t2*t9*t16)/2.0-B_i11*rollDot*yawDot*t2*t12-B_i11*rollDot*pitchDot*t8*t15+B_i11*yawDot*pitchDot*t7*t8*2.0-B_i22*yawDot*pitchDot*t7*t8-B_i33*yawDot*pitchDot*t7*t8+B_i12*rollDDot*t8*t11*t15+B_i13*rollDDot*t8*t10*t15-B_i12*yawDDot*t7*t8*t11*2.0-B_i13*yawDDot*t7*t8*t10*2.0-B_i12*yawDDot*t2*t10*t15+B_i13*yawDDot*t2*t11*t15-B_i23*yawDDot*t2*t4*t12-(B_i22*yawDDot*t2*t12*t13)/2.0-(B_i22*yawDDot*t4*t8*t16)/2.0+B_i23*yawDDot*t8*t13*t16+(B_i33*yawDDot*t2*t12*t13)/2.0+(B_i33*yawDDot*t4*t8*t16)/2.0-B_i12*pitchDDot*t8*t10*t12+B_i13*pitchDDot*t8*t11*t12+B_i23*pitchDDot*t4*t8*t15+(B_i22*pitchDDot*t8*t13*t15)/2.0-(B_i33*pitchDDot*t8*t13*t15)/2.0-B_i12*t2*t7*t9*t11*2.0-B_i13*t2*t7*t9*t10*2.0+B_i12*t5*t8*t10*t15-B_i13*t5*t8*t11*t15-(B_i22*t2*t4*t9*t16)/2.0+B_i12*t8*t9*t10*t15-B_i13*t8*t9*t11*t15+B_i23*t4*t8*t9*t12-B_i12*t8*t10*t14*t15+B_i13*t8*t11*t14*t15-B_i23*t4*t8*t12*t14+B_i23*t2*t9*t13*t16+(B_i22*t8*t9*t12*t13)/2.0+(B_i33*t2*t4*t9*t16)/2.0-(B_i22*t8*t12*t13*t14)/2.0-(B_i33*t8*t9*t12*t13)/2.0+(B_i33*t8*t12*t13*t14)/2.0-B_i12*rollDot*yawDot*t7*t8*t10*2.0+B_i13*rollDot*yawDot*t7*t8*t11*2.0+B_i12*rollDot*yawDot*t2*t11*t15*2.0+B_i13*rollDot*yawDot*t2*t10*t15*2.0-B_i22*rollDot*yawDot*t2*t4*t12+B_i23*rollDot*yawDot*t2*t12*t13*2.0+B_i23*rollDot*yawDot*t4*t8*t16*2.0+B_i33*rollDot*yawDot*t2*t4*t12+B_i22*rollDot*yawDot*t8*t13*t16-B_i33*rollDot*yawDot*t8*t13*t16+B_i22*rollDot*pitchDot*t4*t8*t15-B_i23*rollDot*pitchDot*t8*t13*t15*2.0-B_i33*rollDot*pitchDot*t4*t8*t15+B_i22*yawDot*pitchDot*t4*t7*t8-B_i12*yawDot*pitchDot*t8*t11*t16*4.0-B_i13*yawDot*pitchDot*t8*t10*t16*4.0-B_i23*yawDot*pitchDot*t7*t8*t13*2.0-B_i33*yawDot*pitchDot*t4*t7*t8,
                -B_i11*rollDDot*t15+B_i11*yawDDot*t7-(B_i22*yawDDot*t7)/2.0-(B_i33*yawDDot*t7)/2.0+B_i11*rollDot*pitchDot*t12+B_i11*yawDot*pitchDot*t16*2.0-B_i22*yawDot*pitchDot*t16-B_i33*yawDot*pitchDot*t16-B_i12*rollDDot*t11*t12-B_i13*rollDDot*t10*t12+(B_i22*yawDDot*t4*t7)/2.0-B_i12*yawDDot*t11*t16*2.0-B_i13*yawDDot*t10*t16*2.0-B_i23*yawDDot*t7*t13-(B_i33*yawDDot*t4*t7)/2.0-B_i12*pitchDDot*t10*t15+B_i13*pitchDDot*t11*t15-B_i23*pitchDDot*t4*t12-(B_i22*pitchDDot*t12*t13)/2.0+(B_i33*pitchDDot*t12*t13)/2.0-B_i12*t5*t10*t12+B_i13*t5*t11*t12+B_i12*t10*t12*t14-B_i13*t11*t12*t14-B_i23*t4*t14*t15-(B_i22*t13*t14*t15)/2.0+(B_i33*t13*t14*t15)/2.0-B_i23*rollDot*yawDot*t4*t7*2.0-B_i12*rollDot*yawDot*t10*t16*2.0+B_i13*rollDot*yawDot*t11*t16*2.0-B_i22*rollDot*yawDot*t7*t13+B_i33*rollDot*yawDot*t7*t13-B_i22*rollDot*pitchDot*t4*t12+B_i23*rollDot*pitchDot*t12*t13*2.0+B_i33*rollDot*pitchDot*t4*t12+B_i12*yawDot*pitchDot*t7*t11*4.0+B_i13*yawDot*pitchDot*t7*t10*4.0+B_i22*yawDot*pitchDot*t4*t16-B_i23*yawDot*pitchDot*t13*t16*2.0-B_i33*yawDot*pitchDot*t4*t16;

  mat.col(2) << B_i23*pitchDDot*t8*-2.0-B_i22*rollDot*pitchDot*t8*2.0+B_i33*rollDot*pitchDot*t8*2.0+B_i12*rollDDot*t8*t11+B_i13*rollDDot*t8*t10-B_i12*yawDDot*t2*t10+B_i13*yawDDot*t2*t11+B_i22*yawDDot*t8*t15-B_i33*yawDDot*t8*t15-B_i22*pitchDDot*t2*t12+B_i33*pitchDDot*t2*t12+B_i23*pitchDDot*t8*t17*4.0+B_i12*t5*t8*t10-B_i13*t5*t8*t11+B_i12*t8*t9*t10-B_i13*t8*t9*t11+B_i22*t2*t9*t15-B_i22*t2*t14*t15-B_i33*t2*t9*t15+B_i33*t2*t14*t15+B_i12*rollDot*yawDot*t2*t11*2.0+B_i13*rollDot*yawDot*t2*t10*2.0-B_i23*rollDot*yawDot*t8*t15*4.0+B_i23*rollDot*pitchDot*t2*t12*4.0+B_i22*rollDot*pitchDot*t8*t17*4.0-B_i33*rollDot*pitchDot*t8*t17*4.0-B_i23*yawDot*pitchDot*t2*t18*4.0+B_i12*rollDDot*t2*t10*t12-B_i13*rollDDot*t2*t11*t12+B_i12*yawDDot*t2*t10*t18*2.0-B_i12*yawDDot*t8*t11*t12-B_i13*yawDDot*t8*t10*t12-B_i13*yawDDot*t2*t11*t18*2.0-B_i23*yawDDot*t2*t12*t15*2.0-B_i22*yawDDot*t8*t15*t17*2.0+B_i33*yawDDot*t8*t15*t17*2.0-B_i12*pitchDDot*t2*t11*t15-B_i13*pitchDDot*t2*t10*t15+B_i22*pitchDDot*t8*t10*t11*2.0+B_i22*pitchDDot*t2*t12*t17*2.0-B_i33*pitchDDot*t8*t10*t11*2.0-B_i33*pitchDDot*t2*t12*t17*2.0-B_i12*t2*t5*t11*t12-B_i13*t2*t5*t10*t12-B_i12*t2*t9*t11*t12-B_i13*t2*t9*t10*t12+B_i12*t2*t11*t12*t14+B_i13*t2*t10*t12*t14-B_i12*t8*t9*t10*t18*2.0+B_i13*t8*t9*t11*t18*2.0-B_i22*t2*t9*t15*t17*2.0+B_i23*t8*t9*t12*t15*2.0+B_i22*t2*t14*t15*t17*2.0+B_i33*t2*t9*t15*t17*2.0-B_i33*t2*t14*t15*t17*2.0-B_i12*rollDot*yawDot*t8*t10*t12*2.0-B_i12*rollDot*yawDot*t2*t11*t18*2.0-B_i13*rollDot*yawDot*t2*t10*t18*2.0+B_i13*rollDot*yawDot*t8*t11*t12*2.0-B_i22*rollDot*yawDot*t2*t12*t15*2.0+B_i33*rollDot*yawDot*t2*t12*t15*2.0+B_i23*rollDot*yawDot*t8*t15*t17*8.0-B_i23*rollDot*pitchDot*t8*t10*t11*8.0-B_i23*rollDot*pitchDot*t2*t12*t17*8.0+B_i23*yawDot*pitchDot*t2*t17*t18*8.0+B_i23*yawDDot*t8*t10*t11*t15*4.0+B_i23*yawDDot*t2*t12*t15*t17*4.0-B_i23*pitchDDot*t2*t10*t11*t12*4.0+B_i23*t2*t9*t10*t11*t15*4.0-B_i23*t2*t10*t11*t14*t15*4.0-B_i23*t8*t9*t12*t15*t17*4.0+B_i22*rollDot*yawDot*t8*t10*t11*t15*4.0+B_i22*rollDot*yawDot*t2*t12*t15*t17*4.0-B_i33*rollDot*yawDot*t8*t10*t11*t15*4.0-B_i33*rollDot*yawDot*t2*t12*t15*t17*4.0-B_i22*rollDot*pitchDot*t2*t10*t11*t12*4.0+B_i33*rollDot*pitchDot*t2*t10*t11*t12*4.0-B_i12*yawDot*pitchDot*t2*t10*t12*t15*4.0+B_i13*yawDot*pitchDot*t2*t11*t12*t15*4.0+B_i22*yawDot*pitchDot*t2*t10*t11*t18*4.0-B_i33*yawDot*pitchDot*t2*t10*t11*t18*4.0+B_i22*yawDDot*t2*t10*t11*t12*t15*2.0-B_i33*yawDDot*t2*t10*t11*t12*t15*2.0-B_i22*t8*t9*t10*t11*t12*t15*2.0+B_i33*t8*t9*t10*t11*t12*t15*2.0-B_i23*rollDot*yawDot*t2*t10*t11*t12*t15*8.0,
                B_i23*pitchDDot*t2*2.0+B_i22*rollDot*pitchDot*t2*2.0-B_i33*rollDot*pitchDot*t2*2.0-B_i12*rollDDot*t2*t11-B_i13*rollDDot*t2*t10-B_i12*yawDDot*t8*t10+B_i13*yawDDot*t8*t11-B_i22*yawDDot*t2*t15+B_i33*yawDDot*t2*t15-B_i22*pitchDDot*t8*t12-B_i23*pitchDDot*t2*t17*4.0+B_i33*pitchDDot*t8*t12-B_i12*t2*t5*t10+B_i13*t2*t5*t11-B_i12*t2*t9*t10+B_i13*t2*t9*t11+B_i22*t8*t9*t15-B_i22*t8*t14*t15-B_i33*t8*t9*t15+B_i33*t8*t14*t15+B_i12*rollDot*yawDot*t8*t11*2.0+B_i13*rollDot*yawDot*t8*t10*2.0+B_i23*rollDot*yawDot*t2*t15*4.0-B_i22*rollDot*pitchDot*t2*t17*4.0+B_i23*rollDot*pitchDot*t8*t12*4.0+B_i33*rollDot*pitchDot*t2*t17*4.0-B_i23*yawDot*pitchDot*t8*t18*4.0+B_i12*rollDDot*t8*t10*t12-B_i13*rollDDot*t8*t11*t12+B_i12*yawDDot*t2*t11*t12+B_i13*yawDDot*t2*t10*t12+B_i12*yawDDot*t8*t10*t18*2.0-B_i13*yawDDot*t8*t11*t18*2.0+B_i22*yawDDot*t2*t15*t17*2.0-B_i23*yawDDot*t8*t12*t15*2.0-B_i33*yawDDot*t2*t15*t17*2.0-B_i22*pitchDDot*t2*t10*t11*2.0-B_i12*pitchDDot*t8*t11*t15-B_i13*pitchDDot*t8*t10*t15+B_i33*pitchDDot*t2*t10*t11*2.0+B_i22*pitchDDot*t8*t12*t17*2.0-B_i33*pitchDDot*t8*t12*t17*2.0-B_i12*t5*t8*t11*t12-B_i13*t5*t8*t10*t12+B_i12*t2*t9*t10*t18*2.0-B_i12*t8*t9*t11*t12-B_i13*t8*t9*t10*t12-B_i13*t2*t9*t11*t18*2.0+B_i12*t8*t11*t12*t14+B_i13*t8*t10*t12*t14-B_i23*t2*t9*t12*t15*2.0-B_i22*t8*t9*t15*t17*2.0+B_i22*t8*t14*t15*t17*2.0+B_i33*t8*t9*t15*t17*2.0-B_i33*t8*t14*t15*t17*2.0+B_i12*rollDot*yawDot*t2*t10*t12*2.0-B_i13*rollDot*yawDot*t2*t11*t12*2.0-B_i12*rollDot*yawDot*t8*t11*t18*2.0-B_i13*rollDot*yawDot*t8*t10*t18*2.0-B_i22*rollDot*yawDot*t8*t12*t15*2.0-B_i23*rollDot*yawDot*t2*t15*t17*8.0+B_i33*rollDot*yawDot*t8*t12*t15*2.0+B_i23*rollDot*pitchDot*t2*t10*t11*8.0-B_i23*rollDot*pitchDot*t8*t12*t17*8.0+B_i23*yawDot*pitchDot*t8*t17*t18*8.0-B_i23*yawDDot*t2*t10*t11*t15*4.0+B_i23*yawDDot*t8*t12*t15*t17*4.0-B_i23*pitchDDot*t8*t10*t11*t12*4.0+B_i23*t8*t9*t10*t11*t15*4.0+B_i23*t2*t9*t12*t15*t17*4.0-B_i23*t8*t10*t11*t14*t15*4.0-B_i22*rollDot*yawDot*t2*t10*t11*t15*4.0+B_i33*rollDot*yawDot*t2*t10*t11*t15*4.0+B_i22*rollDot*yawDot*t8*t12*t15*t17*4.0-B_i33*rollDot*yawDot*t8*t12*t15*t17*4.0-B_i22*rollDot*pitchDot*t8*t10*t11*t12*4.0+B_i33*rollDot*pitchDot*t8*t10*t11*t12*4.0-B_i12*yawDot*pitchDot*t8*t10*t12*t15*4.0+B_i13*yawDot*pitchDot*t8*t11*t12*t15*4.0+B_i22*yawDot*pitchDot*t8*t10*t11*t18*4.0-B_i33*yawDot*pitchDot*t8*t10*t11*t18*4.0+B_i22*yawDDot*t8*t10*t11*t12*t15*2.0-B_i33*yawDDot*t8*t10*t11*t12*t15*2.0+B_i22*t2*t9*t10*t11*t12*t15*2.0-B_i33*t2*t9*t10*t11*t12*t15*2.0-B_i23*rollDot*yawDot*t8*t10*t11*t12*t15*8.0,
                B_i23*yawDDot*t4+(B_i22*yawDDot*t13)/2.0-(B_i33*yawDDot*t13)/2.0+B_i22*rollDot*yawDot*t4-B_i23*rollDot*yawDot*t13*2.0-B_i33*rollDot*yawDot*t4+B_i12*rollDDot*t10*t15-B_i13*rollDDot*t11*t15-B_i12*yawDDot*t7*t10+B_i13*yawDDot*t7*t11+B_i23*yawDDot*t4*t16+(B_i22*yawDDot*t13*t16)/2.0-(B_i33*yawDDot*t13*t16)/2.0+B_i12*pitchDDot*t11*t12+B_i13*pitchDDot*t10*t12+B_i22*pitchDDot*t4*t15-B_i23*pitchDDot*t13*t15*2.0-B_i33*pitchDDot*t4*t15-B_i12*t5*t11*t15-B_i13*t5*t10*t15+B_i12*t11*t14*t15+B_i13*t10*t14*t15-B_i22*t4*t12*t14+B_i23*t12*t13*t14*2.0+B_i33*t4*t12*t14+B_i12*rollDot*yawDot*t7*t11+B_i13*rollDot*yawDot*t7*t10+B_i22*rollDot*yawDot*t4*t16-B_i23*rollDot*yawDot*t13*t16*2.0-B_i33*rollDot*yawDot*t4*t16-B_i23*rollDot*pitchDot*t4*t15*4.0-B_i22*rollDot*pitchDot*t13*t15*2.0+B_i33*rollDot*pitchDot*t13*t15*2.0-B_i23*yawDot*pitchDot*t4*t7*2.0-B_i12*yawDot*pitchDot*t10*t16*2.0+B_i13*yawDot*pitchDot*t11*t16*2.0-B_i22*yawDot*pitchDot*t7*t13+B_i33*yawDot*pitchDot*t7*t13;

  return mat;
}

// Derivative of angular acceleration w.r.t euler angles
// Equarion 3.22 in Thomas report.
Eigen::Matrix3d getDerivativeAngularAccelerationWrtEulerAngles(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame,
    const motion_generation::EulerAnglesZyxDiff& eulerAccelerationZyxBodyInWorldFrame) {
  const double cosPitch  = std::cos(orientationBodyToWorld.pitch());
  const double sinPitch  = std::sin(orientationBodyToWorld.pitch());
  const double cosRoll   = std::cos(orientationBodyToWorld.roll());
  const double sinRoll   = std::sin(orientationBodyToWorld.roll());

  const double roll_yaw_dot   = eulerRatesZyxBodyInWorldFrame.roll() * eulerRatesZyxBodyInWorldFrame.yaw();
  const double pitch_yaw_dot  = eulerRatesZyxBodyInWorldFrame.pitch() * eulerRatesZyxBodyInWorldFrame.yaw();
  const double pitch_roll_dot = eulerRatesZyxBodyInWorldFrame.pitch() * eulerRatesZyxBodyInWorldFrame.roll();

  Eigen::Matrix3d matrix1;
  matrix1 <<
      0.0, -cosPitch*eulerAccelerationZyxBodyInWorldFrame.yaw(),          0.0,
      0.0, -sinRoll*sinPitch*eulerAccelerationZyxBodyInWorldFrame.yaw(),  cosRoll*cosPitch*eulerAccelerationZyxBodyInWorldFrame.yaw()-sinRoll*eulerAccelerationZyxBodyInWorldFrame.pitch(),
      0.0, -cosRoll*sinPitch*eulerAccelerationZyxBodyInWorldFrame.yaw(), -sinRoll*cosPitch*eulerAccelerationZyxBodyInWorldFrame.yaw()-cosRoll*eulerAccelerationZyxBodyInWorldFrame.pitch();

  Eigen::Matrix3d matrix2;
  matrix2 <<
      0.0,  sinPitch*pitch_yaw_dot,                                        0.0,
      0.0, -sinRoll*cosPitch*pitch_yaw_dot-cosRoll*sinPitch*roll_yaw_dot, -cosRoll*sinPitch*pitch_yaw_dot-sinRoll*cosPitch*roll_yaw_dot-cosRoll*pitch_roll_dot,
      0.0, -cosRoll*cosPitch*pitch_yaw_dot+sinRoll*sinPitch*roll_yaw_dot,  sinRoll*sinPitch*pitch_yaw_dot-cosRoll*cosPitch*roll_yaw_dot+sinRoll*pitch_roll_dot;

  return (matrix1 + matrix2);
}

Eigen::Matrix3d getDerivativeAngularAccelerationWrtEulerRates(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame) {
  const double rollDot = eulerRatesZyxBodyInWorldFrame.roll();
  const double yawDot = eulerRatesZyxBodyInWorldFrame.yaw();
  const double pitchDot = eulerRatesZyxBodyInWorldFrame.pitch();
  const double yaw = orientationBodyToWorld.yaw();
  const double pitch = orientationBodyToWorld.pitch();
  const double t2 = std::cos(yaw);
  const double t3 = std::cos(pitch);
  const double t4 = std::sin(yaw);
  const double t5 = std::sin(pitch);
  Eigen::Matrix3d mat;

  mat.col(0) << -pitchDot*t2-rollDot*t3*t4,
                -pitchDot*t4+rollDot*t2*t3,
                 0.0;
  mat.col(1) << -yawDot*t2-rollDot*t2*t5,
                -yawDot*t4-rollDot*t4*t5,
                -rollDot*t3;
  mat.col(2) << -yawDot*t3*t4-pitchDot*t2*t5,
                 yawDot*t2*t3-pitchDot*t4*t5,
                -pitchDot*t3;

  return mat;
}

motion_generation::AngularAcceleration convertEulerAnglesZYXAccelerationToAngularAccelerationInInertialFrame(
    const motion_generation::EulerAnglesZyx& orientationBodyToInertial,
    const motion_generation::EulerAnglesZyxDiff& eulerAnglesZyxVelocityBodyInInertialFrame,
    const motion_generation::EulerAnglesZyxDiff& eulerAnglesZyxAccelerationBodyInInertialFrame) {
  return motion_generation::AngularAcceleration(
      zmp::getTimeDerivativeMapEulerAnglesZyxToAngularVelocityInInertialFrame(
          orientationBodyToInertial, eulerAnglesZyxVelocityBodyInInertialFrame) * eulerAnglesZyxVelocityBodyInInertialFrame.toImplementation()
    + zmp::getMapEulerAnglesZyxToAngularVelocityInInertialFrame(orientationBodyToInertial) * eulerAnglesZyxAccelerationBodyInInertialFrame.toImplementation());
}

motion_generation::AngularAcceleration convertEulerAnglesZYXAccelerationToAngularAccelerationInBodyFrame(
    const motion_generation::EulerAnglesZyx& orientationBodyToInertial,
    const motion_generation::EulerAnglesZyxDiff& eulerAnglesZyxVelocityBodyInInertialFrame,
    const motion_generation::EulerAnglesZyxDiff& eulerAnglesZyxAccelerationBodyInInertialFrame) {
  return motion_generation::AngularAcceleration(
      zmp::getMatrixAngularRatesZyxToAngularVelocityInBodyFrameTimeDerivative(
          orientationBodyToInertial, eulerAnglesZyxVelocityBodyInInertialFrame) * eulerAnglesZyxVelocityBodyInInertialFrame.toImplementation()
    + zmp::getMapEulerAnglesZyxToAngularVelocityInBodyFrame(orientationBodyToInertial) * eulerAnglesZyxAccelerationBodyInInertialFrame.toImplementation());
}

}

