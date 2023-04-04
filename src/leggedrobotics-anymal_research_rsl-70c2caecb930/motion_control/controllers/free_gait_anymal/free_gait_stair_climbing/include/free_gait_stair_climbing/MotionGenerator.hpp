/*
 * MotionGenerator.hpp
 *
 *  Created on: Sep 2, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "free_gait_stair_climbing/StairsGeometry.hpp"

// Free Gait
#include <free_gait_core/free_gait_core.hpp>

// STD
#include <string>

namespace free_gait_stair_climbing {

using free_gait::LimbEnum;
using free_gait::Position;

using free_gait::AD;
using LateralEnum = AD::ConcreteTopology::LateralEnum;
using LongitudinalEnum = AD::ConcreteTopology::LongitudinalEnum;

class MotionGenerator
{
 public:
  MotionGenerator(const double baseHeight, const double footPlacementWidth,
                  const unsigned int nStepsBetweenFrontAndHindLegs);
  virtual ~MotionGenerator();

  const free_gait::StepQueue& computeMotion(const StairsGeometry& stairsGeometry);
  const free_gait::StepQueue& getMotion() const;

 private:

  void generateApproach();
  void generateStep(const size_t nextStepNr);
  void generateEgress(const size_t lastStepNr);

  Position getPosition(const LimbEnum& limb, const size_t stepNumber);
  free_gait::Step getBaseShiftStep();
  free_gait::Step getFootstepStep(const LimbEnum& limb, const Position& position);

  free_gait::StepQueue queue_;
  double baseHeight_;
  double footPlacementHalfWidth_;
  unsigned int nStepsBetweenFrontAndHindLegs_;
  double stepProfileHeight_;
  StairsGeometry stairsGeometry_;
};

} /* namespace free_gait_stair_climbing */
