/*
 * conversions.hpp
 *
 *  Created on: Apr 2, 2019
 *      Author: Dario Bellicoso
 */

#pragma once

// motion generation utils
#include <motion_generation_utils/typedefs.hpp>

// eigen
#include <Eigen/Core>

namespace zmp {

/*! Compute the 3x3 matrix "C", function of ZYX Euler angles, that maps the
 *  time-derivative of ZYX Euler angles "alpha_dot" to angular velocity expressed
 *  in an inertial frame "I_omega", i.e.
 *    I_omega = C(alpha) * alpha_dot
 * @param orientationBodyToWorld    the set of ZYX Euler angles
 * @returns the 3x3 mapping matrix C
 */
Eigen::Matrix3d getMapEulerAnglesZyxToAngularVelocityInInertialFrame(
    const motion_generation::EulerAnglesZyx& orientationBodyToInertial);

/*! Compute the time-derivative "C_dot" of the 3x3 matrix "C", function of ZYX Euler angles,
 *  that maps the time-derivative of ZYX Euler angles "alpha_dot" to angular velocity expressed
 *  in an inertial frame "I_omega". This matrix is used as
 *     I_omega_dot = C_dot(alpha, alpha_dot) * alpha_dot + C(alpha) * alpha_ddot
 * @param orientationBodyToWorld          the set of ZYX Euler angles
 * @param eulerRatesZyxBodyInWorldFrame   the time-derivative of the set of ZYX Euler angles
 * @returns the 3x3 mapping matrix C_dot
 */
 Eigen::Matrix3d getTimeDerivativeMapEulerAnglesZyxToAngularVelocityInInertialFrame(
    const motion_generation::EulerAnglesZyx& orientationBodyToInertial,
    const motion_generation::EulerAnglesZyxDiff& orientationDiffBodyToInertial);

/*! Compute the 3x3 matrix "C", function of ZYX Euler angles, that maps the time-derivative of ZYX Euler angles "alpha_dot" to
 *  angular velocity expressed in the local body frame frame "B_omega", i.e.
 *    B_omega = C(alpha) * alpha_dot
 * @param orientationBodyToWorld    the set of ZYX Euler angles
 * @returns the 3x3 mapping matrix C
 */
Eigen::Matrix3d getMapEulerAnglesZyxToAngularVelocityInBodyFrame(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld);

Eigen::Matrix3d getMatrixAngularRatesZyxToAngularVelocityInBodyFrameTimeDerivative(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame);

Eigen::Matrix3d getMatrixAngularVelocityInInertialFrameToAngularRatesZyx(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld);

Eigen::Matrix3d getMatrixAngularVelocityInInertialFrameToAngularRatesZyxTimeDerivative(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame);

Eigen::Matrix3d getMatrixAngularVelocityInBodyFrameToAngularRatesZyx(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld);

Eigen::Matrix3d getMatrixAngularVelocityInBodyFrameToAngularRatesZyxTimeDerivative(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame);

// Jacobian of angular velocity in body frame w.r.t to euler angles.
// Equation 3.19 in Thomas report.
Eigen::Matrix3d getDerivativeAngularVelocityWrtEulerAngles(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame);

Eigen::Matrix3d getDerivativeLdotWrtEulerAngles(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerDiffZyxBodyInWorldFrame,
    const motion_generation::EulerAnglesZyxDiff& eulerDDiffZyxBodyInWorldFrame,
    const Eigen::Matrix3d& inertiaTensorInBodyFrame);

// Derivative of angular acceleration w.r.t euler angles
// Equarion 3.22 in Thomas report.
Eigen::Matrix3d getDerivativeAngularAccelerationWrtEulerAngles(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame,
    const motion_generation::EulerAnglesZyxDiff& eulerAccelerationZyxBodyInWorldFrame);

Eigen::Matrix3d getDerivativeAngularAccelerationWrtEulerRates(
    const motion_generation::EulerAnglesZyx& orientationBodyToWorld,
    const motion_generation::EulerAnglesZyxDiff& eulerRatesZyxBodyInWorldFrame);

motion_generation::AngularAcceleration convertEulerAnglesZYXAccelerationToAngularAccelerationInInertialFrame(
    const motion_generation::EulerAnglesZyx& orientationBodyToInertial,
    const motion_generation::EulerAnglesZyxDiff& eulerAnglesZyxVelocityBodyInInertialFrame,
    const motion_generation::EulerAnglesZyxDiff& eulerAnglesZyxAccelerationBodyInInertialFrame);

motion_generation::AngularAcceleration convertEulerAnglesZYXAccelerationToAngularAccelerationInBodyFrame(
    const motion_generation::EulerAnglesZyx& orientationBodyToInertial,
    const motion_generation::EulerAnglesZyxDiff& eulerAnglesZyxVelocityBodyInInertialFrame,
    const motion_generation::EulerAnglesZyxDiff& eulerAnglesZyxAccelerationBodyInInertialFrame);

} /* namespace zmp */
