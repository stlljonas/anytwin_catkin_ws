/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Typedefs for geomerty_utils
 */

#include <any_measurements/Time.hpp>
#include <kindr/Core>

namespace geometry_utils {

// Physics definitions
using Position = kindr::Position3D;
using RotationQuaternion = kindr::RotationQuaternionD;
using EulerAnglesRpy = kindr::EulerAnglesRpyD;
using EulerAnglesZyx = kindr::EulerAnglesZyxD;
using LocalAngularVelocity = kindr::LocalAngularVelocityD;
using LinearVelocity = kindr::Velocity3D;
using Transform = kindr::HomogeneousTransformationPosition3RotationQuaternionD;

using Vector3D = Eigen::Vector3d;
using Vector4D = Eigen::Matrix<double, 4, 1>;

using Time = any_measurements::Time;

}  // namespace geometry_utils