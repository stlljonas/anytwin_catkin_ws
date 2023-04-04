/*
 * typoe_defs.hpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// Anymal model.
#include <anymal_model/typedefs.hpp>

// Anymal description.
#include <anymal_description/AnymalDescription.hpp>

// Free Gait.
#include <free_gait_core/TypeDefs.hpp>
#include <free_gait_core/TypePrints.hpp>
#include <free_gait_core/step/Step.hpp>

#include <Eigen/Core>

#include <list>

namespace locomotion_planner {

struct EnumClassHash
{
  template<typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

// Import enum aliases.
using LimbEnum = anymal_description::AnymalDescription::LimbEnum;
using BranchEnum = anymal_description::AnymalDescription::BranchEnum;
using ContactEnum = anymal_description::AnymalDescription::ContactEnum;
using free_gait::operator<<;

// Import kindr aliases.
using Transform = anymal_model::Pose;
using anymal_model::Pose;
using anymal_model::Twist;
using anymal_model::RotationQuaternion;
using anymal_model::AngleAxis;
using anymal_model::RotationMatrix;
using anymal_model::EulerAnglesZyx;
using anymal_model::RotationVector;
using anymal_model::EulerAnglesXyz;
using anymal_model::EulerAnglesXyzDiff;
using anymal_model::Position;
using Position2 = kindr::Position<double, 2>;
using anymal_model::LinearVelocity;
using anymal_model::LocalAngularVelocity;
using anymal_model::EulerAnglesZyxDiff;
using anymal_model::LinearAcceleration;
using anymal_model::AngularAcceleration;
using anymal_model::Force;
using anymal_model::Torque;
using anymal_model::Vector;

// Import robot-specific kindr quantities.
using anymal_model::GeneralizedCoordinates;
using anymal_model::GeneralizedVelocities;
using anymal_model::GeneralizedAccelerations;
using anymal_model::JointPositions;
using JointPositionsLeg = anymal_model::JointPositionsLimb;
using anymal_model::JointVelocities;
using JointVelocitiesLeg = anymal_model::JointVelocitiesLimb;
using anymal_model::JointAccelerations;
using JointAccelerationsLeg = anymal_model::JointAccelerationsLimb;
using JointEfforts = anymal_model::JointTorques ;
using JointEffortsLeg = anymal_model::JointTorquesLimb;

// Import Free Gait aliases.
using free_gait::Stance;
using free_gait::PlanarStance;
using free_gait::Step;

using PlanarPose = Eigen::Array3d;
using PlanarTwist = Eigen::Array3d;

} // namespace
