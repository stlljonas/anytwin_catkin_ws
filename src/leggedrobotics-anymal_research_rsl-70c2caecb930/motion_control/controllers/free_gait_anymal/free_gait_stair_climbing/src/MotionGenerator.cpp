/*
 * MotionGenerator.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_stair_climbing/MotionGenerator.hpp"
#include "free_gait_stair_climbing/StairsGeometry.hpp"

using namespace free_gait;

namespace free_gait_stair_climbing {

MotionGenerator::MotionGenerator(const double baseHeight, const double footPlacementWidth,
                                 const unsigned int nStepsBetweenFrontAndHindLegs)
    : baseHeight_(baseHeight),
      footPlacementHalfWidth_(footPlacementWidth/2.0),
      nStepsBetweenFrontAndHindLegs_(nStepsBetweenFrontAndHindLegs),
      stepProfileHeight_(0.0)
{
  stepProfileHeight_ = 0.05;
}

MotionGenerator::~MotionGenerator()
{
}

const StepQueue& MotionGenerator::computeMotion(const StairsGeometry& stairsGeometry)
{
  stairsGeometry_ = stairsGeometry;

  queue_.clear();

  generateApproach();

  for (size_t stepNr = 3; stepNr < stairsGeometry.getNumberOfSteps(); ++stepNr) {
    generateStep(stepNr);
  }

  generateEgress(stairsGeometry.getNumberOfSteps());

  return queue_;
}

const free_gait::StepQueue& MotionGenerator::getMotion() const
{
  return queue_;
}

void MotionGenerator::generateApproach()
{
  // Approach.
  {
    Step baseHeightChangeStep;
    BaseAuto baseAuto;
    baseAuto.setHeight(baseHeight_);
    baseHeightChangeStep.addBaseMotion(baseAuto);
    queue_.add(baseHeightChangeStep);

    Position leftHindPosition(-2.5 * stairsGeometry_.getRun(1), footPlacementHalfWidth_, 0.0);
    queue_.add(getFootstepStep(LimbEnum::LH_LEG, leftHindPosition));
    queue_.add(getBaseShiftStep());
    Position rightHindPosition(leftHindPosition);
    rightHindPosition.y() *= -1.0;
    queue_.add(getFootstepStep(LimbEnum::RH_LEG, rightHindPosition));
    queue_.add(getBaseShiftStep());
  }

  // First step.
  {
    queue_.add(getFootstepStep(LimbEnum::LF_LEG, getPosition(LimbEnum::LF_LEG, 1)));
    queue_.add(getBaseShiftStep());
    queue_.add(getFootstepStep(LimbEnum::RF_LEG, getPosition(LimbEnum::RF_LEG, 1)));
    queue_.add(getBaseShiftStep());

    Position leftHindPosition(-1.1 * stairsGeometry_.getRun(1), footPlacementHalfWidth_, 0.0);
    queue_.add(getFootstepStep(LimbEnum::LH_LEG, leftHindPosition));
    queue_.add(getBaseShiftStep());
    Position rightHindPosition(leftHindPosition);
    rightHindPosition.y() *= -1.0;
    queue_.add(getFootstepStep(LimbEnum::RH_LEG, rightHindPosition));
    queue_.add(getBaseShiftStep());
  }

  // Second step.
  {
    queue_.add(getFootstepStep(LimbEnum::LF_LEG, getPosition(LimbEnum::LF_LEG, 2)));
    queue_.add(getBaseShiftStep());
    queue_.add(getFootstepStep(LimbEnum::RF_LEG, getPosition(LimbEnum::RF_LEG, 2)));
    queue_.add(getBaseShiftStep());

    // Approaching with hind legs.
    Position leftHindPosition(-0.4 * stairsGeometry_.getRun(1), footPlacementHalfWidth_, 0.0);
    queue_.add(getFootstepStep(LimbEnum::LH_LEG, leftHindPosition));
    queue_.add(getBaseShiftStep());
    Position rightHindPosition(leftHindPosition);
    rightHindPosition.y() *= -1.0;
    queue_.add(getFootstepStep(LimbEnum::RH_LEG, rightHindPosition));
    queue_.add(getBaseShiftStep());

    // Step up with hind legs.
    queue_.add(getFootstepStep(LimbEnum::LH_LEG, getPosition(LimbEnum::LH_LEG, 1)));
    queue_.add(getBaseShiftStep());
    queue_.add(getFootstepStep(LimbEnum::RH_LEG, getPosition(LimbEnum::RH_LEG, 1)));
    queue_.add(getBaseShiftStep());
  }
}

void MotionGenerator::generateStep(const size_t nextStepNr)
{
  queue_.add(getFootstepStep(LimbEnum::LF_LEG, getPosition(LimbEnum::LF_LEG, nextStepNr)));
  queue_.add(getBaseShiftStep());
  queue_.add(getFootstepStep(LimbEnum::RF_LEG, getPosition(LimbEnum::RF_LEG, nextStepNr)));
  queue_.add(getBaseShiftStep());
  queue_.add(getFootstepStep(LimbEnum::LH_LEG, getPosition(LimbEnum::LH_LEG, nextStepNr - nStepsBetweenFrontAndHindLegs_)));
  queue_.add(getBaseShiftStep());
  queue_.add(getFootstepStep(LimbEnum::RH_LEG, getPosition(LimbEnum::RH_LEG, nextStepNr - nStepsBetweenFrontAndHindLegs_)));
  queue_.add(getBaseShiftStep());
}

void MotionGenerator::generateEgress(const size_t lastStepNr)
{
}

Position MotionGenerator::getPosition(const LimbEnum& limb, const size_t stepNumber)
{
  const auto lateralEnum = AD::ConcreteDefinitions::mapLimbToLateral::at(limb);
  Position footPosition = stairsGeometry_.getCenterPosition(stepNumber);
  footPosition.y() = lateralEnum == LateralEnum::LEFT ? footPlacementHalfWidth_ : -footPlacementHalfWidth_;
  return footPosition;
}

free_gait::Step MotionGenerator::getBaseShiftStep()
{
  Step step;
  step.addBaseMotion(BaseAuto());
  return step;
}

free_gait::Step MotionGenerator::getFootstepStep(const LimbEnum& limb, const Position& position)
{
  Step step;
  step.addBaseMotion(BaseAuto());
  Footstep footstep(limb);
  footstep.setTargetPosition(stairsGeometry_.getFrameId(), position);
  footstep.setProfileHeight(stepProfileHeight_);
  footstep.setProfileType("square");
  step.addLegMotion(footstep);
  return step;
}

}
