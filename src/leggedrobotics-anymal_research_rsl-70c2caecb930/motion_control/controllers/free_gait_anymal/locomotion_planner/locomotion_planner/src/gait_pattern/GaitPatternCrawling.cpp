/*
 * GaitPatternCrawling.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */
#include "locomotion_planner/gait_pattern/GaitPatternCrawling.hpp"

#include <free_gait_core/TypePrints.hpp>

#include <stdexcept>

namespace locomotion_planner {

GaitPatternCrawling::GaitPatternCrawling() :
    GaitPatternBase(Type::CRAWLING)
{
  limbSequence_[Direction::FORWARD] = {LimbEnum::RH_LEG, LimbEnum::RF_LEG, LimbEnum::LH_LEG, LimbEnum::LF_LEG};
  limbSequence_[Direction::BACKWARD] = {LimbEnum::LF_LEG, LimbEnum::LH_LEG, LimbEnum::RF_LEG, LimbEnum::RH_LEG};
  limbSequence_[Direction::LEFT] = {LimbEnum::RH_LEG, LimbEnum::LH_LEG, LimbEnum::RF_LEG, LimbEnum::LF_LEG};
  limbSequence_[Direction::RIGHT] = {LimbEnum::LH_LEG, LimbEnum::RH_LEG, LimbEnum::LF_LEG, LimbEnum::RF_LEG};
  limbSequence_[Direction::ROTATION_LEFT] = {LimbEnum::LH_LEG, LimbEnum::RH_LEG, LimbEnum::RF_LEG, LimbEnum::LF_LEG};
  limbSequence_[Direction::ROTATION_RIGHT] = {LimbEnum::RH_LEG, LimbEnum::LH_LEG, LimbEnum::LF_LEG, LimbEnum::RF_LEG};
}

GaitPatternCrawling::~GaitPatternCrawling()
{
}

void GaitPatternCrawling::advance()
{
  currentLimb_++;
  if (currentLimb_ == limbSequence_[getDirection()].end()) {
    currentLimb_ = limbSequence_[getDirection()].begin();
  }
}

void GaitPatternCrawling::setFirstStep(const LimbEnum& limb)
{
  if (!isDirectionValid()) {
    throw std::runtime_error("Direction of gait pattern not valid.");
  }
  currentLimb_ = std::find(limbSequence_[getDirection()].begin(), limbSequence_[getDirection()].end(), limb);

  if (currentLimb_ == limbSequence_[getDirection()].end()) {
    throw std::invalid_argument("Desired limb for first step does not exist in the gait pattern definition.");
  }
}

const LimbEnum GaitPatternCrawling::getCurrentLimb() const
{
  return *currentLimb_;
}

bool GaitPatternCrawling::startedNewCycle() const
{
  return getCurrentLimb() == limbSequence_.at(getDirection()).front();
}

bool GaitPatternCrawling::endOfCycle() const
{
  return getCurrentLimb() == limbSequence_.at(getDirection()).back();
}

const std::vector<LimbEnum> GaitPatternCrawling::getGaitPattern(const Direction& direction)
{
  return limbSequence_.at(direction);
}

bool GaitPatternCrawling::isDirectionValid() const
{
  if (getDirection() == Direction::UNDEFINED) return false;
  return true;
}

} /* namespace locomotion_planner */
