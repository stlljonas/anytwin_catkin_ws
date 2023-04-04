/*
 * MotionPlanBase.cpp
 *
 *  Created on: 03.26, 2017
 *      Author: Fabian Jenelten
 */


// motion generation
#include <motion_generation_utils/motion_generation.hpp>
#include <motion_generation_utils/MotionPlanBase.hpp>

// message logger
#include <message_logger/message_logger.hpp>


namespace motion_generation {

MotionPlanBase::MotionPlanBase()
    : initialRobotStateInPlaneFrame_(zmp::ZmpOptState(0.0)),
      finalRobotStateInPlaneFrame_(zmp::ZmpOptState(0.0)),
      terminationState_(zmp::TerminationState::Undefined),
      optimizationHorizon_(0.0),
      virtualPlaneFrame_(),
      posePreviousPlaneToWorld_(),
      enforceHardFinalConstraints_(true),
      skipHardInitialAccelConstraints_(true),
      optimizationDofs_(),
      isInitialized_(false)
{

}

bool MotionPlanBase::initialize() {
  optimizationDofs_.clear();
  terminationState_ = zmp::TerminationState::Undefined;
  isInitialized_ = true;
  return true;
}

const zmp::ZmpOptStateVector& MotionPlanBase::getInitialRobotStateInPlaneFrame() const {
  return initialRobotStateInPlaneFrame_;
}

const zmp::ZmpOptStateVector& MotionPlanBase::getFinalRobotStateInPlaneFrame() const {
  return finalRobotStateInPlaneFrame_;
}

void MotionPlanBase::setOptimizationDofs(const std::vector<zmp::CogDim>& optimizationDofs) {
  optimizationDofs_ = optimizationDofs;
}

const std::vector<zmp::CogDim>& MotionPlanBase::getOptimizationDofs() const {
  return optimizationDofs_;
}

zmp::TerminationState MotionPlanBase::getTerminationState() const {
  return terminationState_;
}

bool MotionPlanBase::didOptimizationSucceeded() const {
  return (
      terminationState_==zmp::TerminationState::returnInitialGuess ||
      terminationState_==zmp::TerminationState::returnOptimizedSolution
  );
}

double MotionPlanBase::getOptimizationHorizon() const {
  return optimizationHorizon_ ;
}

void MotionPlanBase::setTerminationState(zmp::TerminationState terminationState) {
  terminationState_ = terminationState;
}

const VirtualPlaneFrameBase& MotionPlanBase::getVirtualPlaneFrame() const {
  return virtualPlaneFrame_;
}

const Pose& MotionPlanBase::getPosePreviousPlaneToWorld() const {
  return posePreviousPlaneToWorld_;
}

bool MotionPlanBase::getEnforceHardFinalConstraints() const {
  return enforceHardFinalConstraints_;
}

bool MotionPlanBase::getSkipHardInitialAccelConstraints() const {
  return skipHardInitialAccelConstraints_;
}

void MotionPlanBase::setEqualityConstraints(bool enforceHardFinalConstraints, bool skipHardInitialAccelConstraints) {
  enforceHardFinalConstraints_ = enforceHardFinalConstraints;
  skipHardInitialAccelConstraints_ = skipHardInitialAccelConstraints;
}

} /* namespace motion_generation */
