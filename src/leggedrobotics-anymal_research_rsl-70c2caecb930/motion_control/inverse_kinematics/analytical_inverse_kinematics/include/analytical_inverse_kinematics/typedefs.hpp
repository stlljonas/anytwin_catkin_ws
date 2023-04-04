/**
 * @authors     Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief       Definition of types for analytical IK
 */
#pragma once

#include <kindr/Core>

namespace analytical_inverse_kinematics {

using Position = kindr::Position3D;
using EulerAnglesZyx = kindr::EulerAnglesZyxPD;
using RotationMatrix = kindr::RotationMatrixPD;

}  // namespace analytical_inverse_kinematics
