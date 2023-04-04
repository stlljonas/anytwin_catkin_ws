//
// Created by Dario Bellicoso on 15/01/18.
//

#pragma once

// romo_measurements
#include <romo_measurements/romo_measurements.hpp>

// loco
#include <loco/common/typedefs.hpp>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>

namespace loco_anymal {

using Vector = loco::Vector;
using Force = loco::Force;
using Position = loco::Position;
using LinearVelocity = loco::LinearVelocity;
using LinearAcceleration = loco::LinearAcceleration;
using LocalAngularVelocity = loco::LocalAngularVelocity;
using AngularAcceleration = loco::AngularAcceleration;
using Torque = loco::Torque;
using RotationQuaternion = loco::RotationQuaternion;
using RotationMatrix = loco::RotationMatrix;
using RotationVector = loco::RotationVector;
using EulerAnglesZyx = loco::EulerAnglesZyx;
using AngleAxis = loco::AngleAxis;

using JointPositions = loco::JointPositions;
using JointVelocities = loco::JointVelocities;
using JointTorques = loco::JointTorques;
using JointControlModes = loco::JointControlModes;
using JointNames = loco::JointNames;

using TranslationJacobian = loco::TranslationJacobian;
using TranslationJacobianLimb = loco::TranslationJacobianLimb;
using RotationJacobian = loco::RotationJacobian;
using RotationJacobianLimb = loco::RotationJacobianLimb;
using SpatialJacobian = loco::SpatialJacobian;

using BodyEnum = anymal_description::AnymalDescription::BodyEnum;
using BodyNodeEnum = anymal_description::AnymalDescription::BodyNodeEnum;
using JointEnum = anymal_description::AnymalDescription::JointEnum;
using JointNodeEnum = anymal_description::AnymalDescription::JointNodeEnum;
using CoordinateFrameEnum = anymal_description::AnymalDescription::CoordinateFrameEnum;
using BranchEnum = anymal_description::AnymalDescription::BranchEnum;
using LimbEnum = anymal_description::AnymalDescription::LimbEnum;
using ContactEnum = anymal_description::AnymalDescription::ContactEnum;
using ActuatorEnum = anymal_description::AnymalDescription::ActuatorEnum;
using GeneralizedCoordinatesEnum = anymal_description::AnymalDescription::GeneralizedCoordinatesEnum;
using GeneralizedVelocitiesEnum = anymal_description::AnymalDescription::GeneralizedVelocitiesEnum;

// Robot model typedef
using AnymalModel = anymal_model::AnymalModel;

//! No Anymal specific wholebody properties
using WholeBodyPropertiesAnymal = romo_measurements::WholeBodyPropertiesRomo<anymal_description::ConcreteAnymalDescription,
                                                                                anymal_model::AnymalState>;

//! Forward LegPropertiesRomo for Anymal leg properties implementation
using LegPropertiesRomo = romo_measurements::LegPropertiesRomo<anymal_description::ConcreteAnymalDescription,
                                                               anymal_model::AnymalState>;

//! Forward EndEffectorRomo for Anymal properties implementation
using EndEffectorRomo = romo_measurements::EndEffectorRomo<anymal_description::ConcreteAnymalDescription,
                                                           anymal_model::AnymalState>;

//! Forward FootRomo for Anymal implementation
using FootRomo = romo_measurements::FootRomo <anymal_description::ConcreteAnymalDescription,
                                              anymal_model::AnymalState>;

//! Forward EndEffectorPropertiesRomo for Anymal shovel properties implementation
using EndEffectorPropertiesRomo = romo_measurements::EndEffectorPropertiesRomo<anymal_description::ConcreteAnymalDescription,
                                                                               anymal_model::AnymalState>;

//! Forward WholebodyRomo for Anymal implementation.
using WholeBodyRomo = romo_measurements::WholeBodyRomo<anymal_description::ConcreteAnymalDescription,
                                                       anymal_model::AnymalState>;

//! Forward LegRomo for Anymal leg implementation
using LegRomo = romo_measurements::LegRomo<anymal_description::ConcreteAnymalDescription,
                                           anymal_model::AnymalState>;

//! No Anymal specific TorsoPropertiesRomo implementation
using TorsoPropertiesAnymal = romo_measurements::TorsoPropertiesRomo<anymal_description::ConcreteAnymalDescription,
                                                                        anymal_model::AnymalState>;

//! No Anymal specific TorsoRomo implementation
using TorsoAnymal = romo_measurements::TorsoRomo<anymal_description::ConcreteAnymalDescription,
                                                    anymal_model::AnymalState>;

} /* namespace loco_anymal */
