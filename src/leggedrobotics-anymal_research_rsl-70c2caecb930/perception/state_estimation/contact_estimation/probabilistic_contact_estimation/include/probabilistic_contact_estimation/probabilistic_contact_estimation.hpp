/*
 * probabilistic_contact_estimation.hpp
 *
 *  Created on: Dez 05, 2018
 *      Author: Fabian Jenelten
 */

// std_utils.
#include <std_utils/std_utils.hpp>

// robot utils.
#include "robot_utils/math/math.hpp"

// Eigen.
#include <Eigen/Core>

// anymal description.
#include <anymal_description/AnymalDescription.hpp>

// basic contact estimation
#include <basic_contact_estimation/ContactDetectorBase.hpp>

#pragma once

namespace contact_estimation {
  // Definitions.
  using Probability = Eigen::Vector2d;
  using AD = anymal_description::AnymalDescription;

  // Indices.
  static constexpr unsigned int contactId = 0u;
  static constexpr unsigned int airId = 1u;
  static constexpr unsigned int noTransitionId = 0u;
  static constexpr unsigned int transitionId = 1u;
  static const Probability initProbability = Probability(0.0, 1.0);
  static const Probability initProbabilitySlip = Probability(1.0, 0.0);

  using ContactState = basic_contact_estimation::ContactDetectorBase::ContactState;

  // Contact type.
  CONSECUTIVE_ENUM(ProbabilityEnum, Dynamics, DiffKinematics, Kinematics, Unknown);
  static std::map<ProbabilityEnum, std::string> probabilityEnumMap =
  {{ProbabilityEnum::Dynamics,        "Dynamics"},
   {ProbabilityEnum::DiffKinematics,  "DiffKinematics"},
   {ProbabilityEnum::Kinematics,      "Kinematics"},
   {ProbabilityEnum::Unknown,         "Unknown"},
   {ProbabilityEnum::SIZE,            "SIZE"}};

  static bool clip(Probability &prob) {
    constexpr double min = 0.001;
    constexpr double max = 1.0 - min;
    robot_utils::boundToRange(&prob(0), min, max);
    robot_utils::boundToRange(&prob(1), min, max);
    return true;
  }

  // A small positive double.
  static constexpr double eps_ = 1.0e-100;
  static constexpr double one_ = 1.0-eps_;

  static bool normalize2dProb(Probability& prob) {
    if (prob(contactId)<0.0 || prob(1)<0.0) { return false; }

    // Compute norm.
    const double normalization = prob(0) + prob(1);
    if (normalization <= eps_) {
      prob(0) += eps_;
      prob(1) += eps_;
      return normalize2dProb(prob);
    }

    // Normalization.
    prob(0) /= normalization;
    prob(1) /= normalization;

    return true;
  }

  static double expPdf(double value, double lambda) {
    return lambda*exp(-lambda*value);
  }

  static double expCdf(double value, double lambda) {
    return (1.0-exp(-lambda*value));
  }

} /* namespace contact_estimation */
