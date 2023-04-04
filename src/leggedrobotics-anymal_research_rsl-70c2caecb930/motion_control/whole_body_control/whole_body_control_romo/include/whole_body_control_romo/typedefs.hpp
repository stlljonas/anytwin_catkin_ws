/*
 * typedefs.hpp
 *
 *  Created on: Apr 27, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

#include <Eigen/Core>

#include "whole_body_control/typedefs.hpp"
#include "romo/common/robot_state_typedefs.hpp"
#include "romo/common/phys_typedefs.hpp"
#include "loco/common/typedefs.hpp"

namespace whole_body_control_romo {
template< typename MatrixType >
using EigenRef = whole_body_control::EigenRef<MatrixType>;

template< typename MatrixType >
using EigenConstRef = whole_body_control::EigenConstRef<MatrixType>;

using JacobianTranslation = loco::TranslationJacobian;
using JacobianRotation = loco::RotationJacobian;
using JacobianSpatial = loco::SpatialJacobian;

using JacobianTranslationConstRef = EigenConstRef<JacobianTranslation>;
using JacobianRotationConstRef = EigenConstRef<JacobianRotation>;
using JacobianSpatialConstRef = EigenConstRef<JacobianSpatial>;


using Vector3dConstRef = const Eigen::Vector3d&; //EigenConstRef<Eigen::Vector3d>;
using Position = romo::Position;
using LinearVelocity = romo::LinearVelocity;
using LinearAcceleration = romo::LinearAcceleration;

using RotationQuaternion = romo::RotationQuaternion;
using RotationVector = romo::RotationVector;
using RotationMatrix = romo::RotationMatrix;
using LocalAngularVelocity = romo::LocalAngularVelocity;
using AngularAcceleration = romo::AngularAcceleration;

}
