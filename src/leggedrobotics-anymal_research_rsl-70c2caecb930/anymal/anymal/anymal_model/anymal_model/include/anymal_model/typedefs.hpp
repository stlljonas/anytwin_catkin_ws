/*
 * typedefs.hpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// romo
#include <romo/common/phys_typedefs.hpp>
#include <romo/common/robot_state_typedefs.hpp>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// kindr
#include <kindr/Core>

// eigen
#include <Eigen/Core>

namespace anymal_model {

// Import types from the robot model namespace.
using romo::AngleAxis;
using romo::AngularAcceleration;
using romo::EulerAnglesXyz;
using romo::EulerAnglesXyzDiff;
using romo::EulerAnglesZyx;
using romo::EulerAnglesZyxDiff;
using romo::Force;
using romo::LinearAcceleration;
using romo::LinearVelocity;
using romo::LocalAngularVelocity;
using romo::Pose;
using romo::Position;
using romo::RotationMatrix;
using romo::RotationQuaternion;
using romo::RotationQuaternionDiff;
using romo::RotationVector;
using romo::Torque;
using romo::Twist;
using romo::Vector;

// ANYmal description abbreviations
// Note: These typedefs reduce the amount of code in the anymal_model package.
//       If you need any of these types, depend on the anymal_description package directly!
using CAD = anymal_description::ConcreteAnymalDescription;
using AD = anymal_description::AnymalDescription;
using AT = anymal_description::AnymalTopology;

// Joint-Sized Vectors
template <typename Scalar_>
using JointVector = Eigen::Matrix<Scalar_, AD::getJointsDimension(), 1>;
using JointVectorD = JointVector<double>;
using JointVectorB = JointVector<bool>;
using JointVectorI = JointVector<int>;

// Joints
using JointPositions = romo::internal::JointPositions<AD::getJointsDimension()>;
using JointVelocities = romo::internal::JointVelocities<AD::getJointsDimension()>;
using JointAccelerations = romo::internal::JointAccelerations<AD::getJointsDimension()>;
using JointTorques = romo::internal::JointTorques<AD::getJointsDimension()>;

using JointPositionsLimb = romo::internal::JointPositions<AD::getNumDofLimb()>;
using JointVelocitiesLimb = romo::internal::JointVelocities<AD::getNumDofLimb()>;
using JointAccelerationsLimb = romo::internal::JointAccelerations<AD::getNumDofLimb()>;
using JointTorquesLimb = romo::internal::JointTorques<AD::getNumDofLimb()>;

// Generalized Coordinates
using GeneralizedCoordinates = romo::internal::GeneralizedCoordinates<AD::getGeneralizedCoordinatesDimension()>;
using GeneralizedVelocities = romo::internal::GeneralizedVelocities<AD::getGeneralizedVelocitiesDimension()>;
using GeneralizedAccelerations = romo::internal::GeneralizedAccelerations<AD::getGeneralizedVelocitiesDimension()>;

// Dynamics
using MassMatrix = Eigen::Matrix<double, AD::getGeneralizedVelocitiesDimension(), AD::getGeneralizedVelocitiesDimension()>;
using GravityTorqueVector = kindr::Torque<double, AD::getGeneralizedVelocitiesDimension()>;
using NonlinearEffectsVector = GravityTorqueVector;
using SelectionMatrix = Eigen::Matrix<double, AD::getJointsDimension(), AD::getGeneralizedVelocitiesDimension()>;

// Jacobians
using Jacobian = Eigen::Matrix<double, AD::getNumTranslationalDof(), AD::getGeneralizedVelocitiesDimension()>;
using TranslationalJacobian = Eigen::Matrix<double, AD::getNumTranslationalDof(), AD::getGeneralizedVelocitiesDimension()>;
using RotationalJacobian = Eigen::Matrix<double, AD::getNumRotationalDof(), AD::getGeneralizedVelocitiesDimension()>;
using SpatialJacobian =
    Eigen::Matrix<double, (AD::getNumTranslationalDof() + AD::getNumRotationalDof()), AD::getGeneralizedVelocitiesDimension()>;

}  // namespace anymal_model
