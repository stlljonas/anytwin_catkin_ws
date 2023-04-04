/*
 * typedefs.hpp
 *
 *  Created on: Feb 27, 2014
 *      Author: Christian Gehring, Péter Fankhauser, Dario Bellicoso
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// kindr
#include <kindr/Core>

// stl
#include <string>

// loco
#include <loco/common/topology.hpp>

namespace loco {
//
//// The definition of this enum matches the control modes available in the series elastic actuator framework.
// enum class ControlMode : int {
//  MODE_FREEZE = 0,                                    // Freeze motor
//  MODE_DISABLE = -1,                                  // Disable motor
//  MODE_CURRENT = 5,                                   // Track current
//  MODE_MOTOR_POSITION = 6,                            // Track motor position
//  MODE_MOTOR_VELOCITY = 1,                            // Track motor velocity
//  MODE_GEAR_POSITION = 12,                            // Track gear position
//  MODE_GEAR_VELOCITY = 13,                            // Track gear velocity
//  MODE_JOINT_POSITION = 2,                            // Track joint position
//  MODE_JOINT_VELOCITY = 4,                            // Track joint velocity
//  MODE_JOINT_TORQUE = 3,                              // Track joint torque
//  MODE_JOINT_POSITION_VELOCITY = 8,                   // Track joint position with feed-forward velocity
//  MODE_JOINT_POSITION_VELOCITY_TORQUE = 7,            // Track joint position with feed-forward velocity and torque
//  MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 11, // Track joint position with feed- forward velocity and torque using custom joint
//  position gains MODE_UNDEFINED = 99
//};

// Joint space properties.
using JointControlMode = ControlMode;
using JointPosition = double;
using JointVelocity = double;
using JointAcceleration = double;
using JointTorque = double;
using JointName = std::string;

using JointControlModes = Eigen::Matrix<JointControlMode, Eigen::Dynamic, 1>;
using JointPositions = Eigen::Matrix<JointPosition, Eigen::Dynamic, 1>;
using JointVelocities = Eigen::Matrix<JointVelocity, Eigen::Dynamic, 1>;
using JointAccelerations = Eigen::Matrix<JointAcceleration, Eigen::Dynamic, 1>;
using JointTorques = Eigen::Matrix<JointTorque, Eigen::Dynamic, 1>;
using JointNames = std::vector<JointName>;

using TranslationJacobian = Eigen::MatrixXd;
using TranslationJacobianLimb = Eigen::MatrixXd;
using RotationJacobian = Eigen::MatrixXd;
using RotationJacobianLimb = Eigen::MatrixXd;
using SpatialJacobian = Eigen::MatrixXd;

// Cylinder space properties.
using CylinderControlMode = int;
using CylinderPosition = double;
using CylinderVelocity = double;
using CylinderForce = double;
using CylinderName = std::string;

using CylinderControlModes = Eigen::Matrix<CylinderControlMode, Eigen::Dynamic, 1>;
using CylinderPositions = Eigen::Matrix<CylinderPosition, Eigen::Dynamic, 1>;
using CylinderVelocities = Eigen::Matrix<CylinderVelocity, Eigen::Dynamic, 1>;
using CylinderForces = Eigen::Matrix<CylinderForce, Eigen::Dynamic, 1>;
using CylinderNames = std::vector<CylinderName>;

// Define physical types
using Pose = kindr::HomTransformQuatD;
using Twist = kindr::TwistLocalD;
using TwistGlobal = kindr::TwistGlobalD;
using RotationQuaternion = kindr::RotationQuaternionPD;
using RotationQuaternionDiff = kindr::RotationQuaternionDiffPD;
using AngleAxis = kindr::AngleAxisPD;
using RotationMatrix = kindr::RotationMatrixPD;
using EulerAnglesZyx = kindr::EulerAnglesZyxPD;
using RotationVector = kindr::RotationVectorPD;
using EulerAnglesXyz = kindr::EulerAnglesXyzPD;
using EulerAnglesXyzDiff = kindr::EulerAnglesXyzDiffPD;
using Position = kindr::Position3D;
using LinearVelocity = kindr::Velocity3D;
using LocalAngularVelocity = kindr::LocalAngularVelocityPD;
using GlobalAngularVelocity = kindr::GlobalAngularVelocityD;
using EulerAnglesZyxDiff = kindr::EulerAnglesZyxDiffPD;
using LinearAcceleration = kindr::Acceleration3D;
using AngularAcceleration = kindr::AngularAcceleration3D;
using Force = kindr::Force3D;
using Torque = kindr::Torque3D;
using Vector = kindr::VectorTypeless3D;
using Wrench = kindr::WrenchD;

// Helper quantities.
namespace cartesian {
constexpr int wrenchVectorSize = 6;
constexpr int forceVectorSize = 3;
constexpr int torqueVectorSize = 3;

enum class TranslationalCoordinates { X = 0, Y = 1, Z = 2, XY = 3, XZ = 4, YZ = 5, XYZ = 6 };

constexpr unsigned int getTranslationalCoordinatesCount(const TranslationalCoordinates tc) {
  return (tc == TranslationalCoordinates::XYZ ? 3 : (static_cast<int>(tc) > static_cast<int>(TranslationalCoordinates::Z) ? 2 : 1));
}

}  // namespace cartesian

namespace print {
const Eigen::IOFormat CleanFmt(2, 0, ",", "\n", "[", "]");
}

}  // namespace loco
