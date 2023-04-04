/*
 * typedefs.hpp
 *
 *  Created on: Dec 14, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// kindr.
#include <kindr/Core>

// std utils.
#include <std_utils/std_utils.hpp>

// loco.
#include <loco/gait_pattern/contact_schedule.hpp>

namespace motion_generation {

// Define physical types
typedef kindr::HomTransformQuatD         Pose;
typedef kindr::TwistLocalD               Twist;
typedef kindr::RotationQuaternionPD      RotationQuaternion;
typedef kindr::RotationQuaternionDiffPD  RotationQuaternionDiff;
typedef kindr::AngleAxisPD               AngleAxis;
typedef kindr::RotationMatrixPD          RotationMatrix;
typedef kindr::EulerAnglesZyxPD          EulerAnglesZyx;
typedef kindr::RotationVectorPD          RotationVector;
typedef kindr::EulerAnglesXyzPD          EulerAnglesXyz;
typedef kindr::EulerAnglesXyzDiffPD      EulerAnglesXyzDiff;
typedef kindr::Position3D                Position;
typedef kindr::Velocity3D                LinearVelocity;
typedef kindr::LocalAngularVelocityPD    LocalAngularVelocity;
typedef kindr::EulerAnglesZyxDiffPD      EulerAnglesZyxDiff;
typedef kindr::Acceleration3D            LinearAcceleration;
typedef kindr::AngularAcceleration3D     AngularAcceleration;
typedef kindr::Force3D                   Force;
typedef kindr::Torque3D                  Torque;
typedef kindr::VectorTypeless3D          Vector;

using positionVector = std::vector<Position, Eigen::aligned_allocator<Position>>;
using velocityVector = std::vector<LinearVelocity, Eigen::aligned_allocator<LinearVelocity>>;
using accelerationVector = std::vector<LinearAcceleration, Eigen::aligned_allocator<LinearAcceleration>>;
using eulerAnglesZyxVector = std::vector<EulerAnglesZyx, Eigen::aligned_allocator<EulerAnglesZyx>>;
using eulerAnglesZyxDiffVector = std::vector<EulerAnglesZyxDiff, Eigen::aligned_allocator<EulerAnglesZyxDiff>>;

using anymalLegsBool = std_utils::EnumArray<loco::contact_schedule::LegEnumAnymal, bool>;
using anymalLegsDouble = std_utils::EnumArray<loco::contact_schedule::LegEnumAnymal, double>;
using anymalLegsPosition = std_utils::EnumArray<loco::contact_schedule::LegEnumAnymal, Position>;
using anymalLegsVector = std_utils::EnumArray<loco::contact_schedule::LegEnumAnymal, Eigen::Vector3d>;

} /* namespace motion_generation */
