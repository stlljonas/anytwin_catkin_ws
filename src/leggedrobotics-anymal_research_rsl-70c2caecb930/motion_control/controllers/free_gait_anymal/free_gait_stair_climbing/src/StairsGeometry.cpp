/*
 * StairsGeometry.cpp
 *
 *  Created on: Nov 24, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "free_gait_stair_climbing/StairsGeometry.hpp"

namespace free_gait_stair_climbing {

StairsGeometry::StairsGeometry()
    : nSteps_(0)
{
}

StairsGeometry::StairsGeometry(const StairsGeometry& other)
{
  *this = other;
}

StairsGeometry::~StairsGeometry()
{
}

StairsGeometry& StairsGeometry::operator =(const StairsGeometry& other)
{
  frameId_ = other.frameId_;
  nSteps_ = other.nSteps_;
  generalStep_ = other.generalStep_;
  if (other.firstStep_) firstStep_.reset(new Step(*(other.firstStep_)));
  if (other.lastStep_) lastStep_.reset(new Step(*(other.lastStep_)));
  return *this;
}

void StairsGeometry::setFrameId(const std::string& frameId)
{
  frameId_ = frameId;
}

const std::string& StairsGeometry::getFrameId() const
{
  return frameId_;
}

void StairsGeometry::setNumberOfSteps(const size_t steps)
{
  nSteps_ = steps;
}

unsigned int StairsGeometry::getNumberOfSteps() const
{
  return nSteps_;
}

void StairsGeometry::setGeneralStepGeometry(const double rise, const double run)
{
  generalStep_.rise_ = rise;
  generalStep_.run_ = run;
}

void StairsGeometry::setFirstStepGeometry(const double rise, const double run)
{
  firstStep_.reset(new Step());
  firstStep_->rise_ = rise;
  firstStep_->run_ = run;
}

void StairsGeometry::setLastStepGeometry(const double rise, const double run)
{
  lastStep_.reset(new Step());
  lastStep_->rise_ = rise;
  lastStep_->run_ = run;
}

double StairsGeometry::getRise(const size_t stepNumber) const
{
  if (stepNumber == 0) {
    throw std::out_of_range("Step number 0 has no rise.");
  } else if (stepNumber == 1 && firstStep_) {
    return firstStep_->rise_;
  } else if (stepNumber == nSteps_ && lastStep_) {
    return lastStep_->rise_;
  } else {
    return generalStep_.rise_;
  }
}

double StairsGeometry::getRun(const size_t stepNumber) const
{
  if (stepNumber == 0) {
    throw std::out_of_range("Step number 0 has no run.");
  } else if (stepNumber == 1 && firstStep_) {
    return firstStep_->run_;
  } else if (stepNumber == nSteps_ && lastStep_) {
    return lastStep_->run_;
  } else {
    return generalStep_.run_;
  }
}

Position StairsGeometry::getCenterPosition(const size_t stepNumber) const
{
  Position centerPosition = Position::Zero();
  for (size_t i = 1; i < stepNumber; ++i) {
    centerPosition.x() += getRun(i);
    centerPosition.z() += getRise(i);
  }
  centerPosition.x() += 0.5 * getRun(stepNumber);
  centerPosition.z() += getRise(stepNumber);
  return centerPosition;
}

} /* namespace free_gait_stair_climbing */
