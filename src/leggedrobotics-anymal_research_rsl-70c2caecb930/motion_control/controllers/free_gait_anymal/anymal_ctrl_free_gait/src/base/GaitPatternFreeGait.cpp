/*
 * GaitPatternFreeGait.cpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "anymal_ctrl_free_gait/base/GaitPatternFreeGait.hpp"
#include <anymal_ctrl_free_gait/base/StateLoco.hpp>
#include "tinyxml_tools/tinyxml_tools.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

// Loco
#include "robot_utils/math/math.hpp"

namespace loco {

using AD = anymal_description::AnymalDescription;

GaitPatternFreeGait::GaitPatternFreeGait(loco_anymal::LegsAnymal& legs, TorsoBase& torso, anymal_model::AnymalModel& anymalModel,
                                         free_gait::Executor& executor)
    : isInitialized_(false), torso_(torso), legs_(legs), anymalModel_(anymalModel), executor_(executor) {}

bool GaitPatternFreeGait::initialize(double dt) {
  //  for (auto leg : *legs_) {
  //    leg->setShouldBeGrounded(leg->isGrounded());
  //  }
  isInitialized_ = true;
  return isInitialized_;
}

bool GaitPatternFreeGait::reset() {
  //  for (auto leg : *legs_) {
  //    leg->setShouldBeGrounded(leg->isGrounded());
  //  }
  return true;
}

bool GaitPatternFreeGait::advance(double dt) {
  for (auto leg : legs_) {
    leg->getContactSchedulePtr()->setShouldBeGrounded(shouldBeLegGrounded(leg->getId()));
    leg->getContactSchedulePtr()->setSwingDuration(getSwingDuration(leg->getId()));
    leg->getContactSchedulePtr()->setSwingPhase(getSwingPhaseForLeg(leg->getId()));
    leg->getContactSchedulePtr()->setStanceDuration(getStanceDuration(leg->getId()));
    leg->getContactSchedulePtr()->setStancePhase(getStancePhaseForLeg(leg->getId()));

    //    leg->setPreviousSwingPhase(leg->getSwingPhase());
    //    leg->setPreviousStancePhase(leg->getStancePhase());
  }
  return true;
}

double GaitPatternFreeGait::getSwingPhaseForLeg(int iLeg) const {
  const auto& limb = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(iLeg);
  try {
    const auto& state = dynamic_cast<const free_gait::StateLoco&>(executor_.getState());
    return state.getSwingPhase(limb);
  } catch (...) {
    MELO_ERROR_STREAM("[GaitPatternFreeGait::getSwingPhaseForLeg] Could not cast state!");
    return 0.0;
  }
}

double GaitPatternFreeGait::getStanceDuration(int iLeg) const {
  // This is just a hack, doesn't make much sense in the context of free gait.
  //  if (stepQueue_->empty()) return std::numeric_limits<double>::max();
  //  Step& step = stepQueue_->getCurrentStep();
  //  for (const auto& stepData : step.getSwingData()) {
  //    if (step.getState() == Step::State::PreStep) {
  //      if (stepData.first == legs_->getLegById(iLeg)->getName()) {
  //        // Leg is going to take a step.
  //        return step.getCurrentBaseShiftData().getDuration();
  //      }
  //    } else if (step.getState() == Step::State::AtStep) {
  //      if (stepData.first == legs_->getLegById(iLeg)->getName()) {
  //        // Leg is stepping.
  //        return step.getCurrentBaseShiftData().getDuration();
  //      }
  //    }
  //  }
  return std::numeric_limits<double>::max();
}

double GaitPatternFreeGait::getStrideDuration() const {
  throw std::runtime_error("GaitPatternFreeGait::getStrideDuration not implemented yet!");
  return 0.0;
}

void GaitPatternFreeGait::setStrideDuration(double strideDuration) {
  throw std::runtime_error("GaitPatternFreeGait::setStrideDuration not implemented yet!");
}

double GaitPatternFreeGait::getStancePhaseForLeg(int iLeg) const {
  // This is just a hack, doesn't make much sense.
  //  if (stepQueue_->empty()) return 0.0;
  //  Step& step = stepQueue_->getCurrentStep();
  //  for (const auto& stepData : step.getSwingData()) {
  //    if (step.getState() == Step::State::PreStep) {
  //      if (stepData.first == legs_->getLegById(iLeg)->getName()) {
  //        // Leg is going to take a step.
  //        return mapTo01Range(step.getTime(), 0.0, step.getPreStepDuration());
  //      }
  //    } else if (step.getState() == Step::State::AtStep) {
  //      if (stepData.first == legs_->getLegById(iLeg)->getName()) {
  //        // Leg is stepping.
  //        return -1.0;
  //      }
  //    }
  //  }
  return 0.0;
}

bool GaitPatternFreeGait::loadParameters(const TiXmlHandle& hParameterSet) {
  return true;
}

bool GaitPatternFreeGait::saveParameters(TiXmlHandle& hParameterSet) {
  throw std::runtime_error("GaitPatternFreeGait::saveParameters not implemented yet!");
  return false;
}

unsigned long int GaitPatternFreeGait::getNGaitCycles() const {
  throw std::runtime_error("GaitPatternFreeGait::getNGaitCycles not implemented yet!");
  return 0;
}

bool GaitPatternFreeGait::shouldBeLegGrounded(int iLeg) const {
  const auto& limb = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(iLeg);
  return executor_.getState().isSupportLeg(limb);
}

double GaitPatternFreeGait::getStridePhase() const {
  //  throw std::runtime_error("GaitPatternFreeGait::getStridePhase not implemented yet!");
  // TODO(pfankhauser): Once Christian stops hacking.
  return 0.0;
}

void GaitPatternFreeGait::setStridePhase(double stridePhase) {
  throw std::runtime_error("GaitPatternFreeGait::setStridePhase not implemented yet!");
}

bool GaitPatternFreeGait::isInitialized() {
  return isInitialized_;
}

double GaitPatternFreeGait::getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getTimeLeftInStance not implemented yet!");
  return 0.0;
}

double GaitPatternFreeGait::getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getTimeLeftInSwing not implemented yet!");
  return 0.0;
}

double GaitPatternFreeGait::getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getTimeSpentInStance not implemented yet!");
  return 0.0;
}

double GaitPatternFreeGait::getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getTimeSpentInSwing not implemented yet!");
  return 0.0;
}

double GaitPatternFreeGait::getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getTimeUntilNextStancePhase not implemented yet!");
  return 0.0;
}

double GaitPatternFreeGait::getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getTimeUntilNextSwingPhase not implemented yet!");
  return 0.0;
}

double GaitPatternFreeGait::getSwingDuration(int iLeg, double strideDuration) const {
  const auto& limb = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(iLeg);
  const auto& state = dynamic_cast<const free_gait::StateLoco&>(executor_.getState());
  return state.getSwingDuration(limb);
}

double GaitPatternFreeGait::getStanceDuration(int iLeg, double strideDuration) const {
  throw std::runtime_error("GaitPatternFreeGait::getStanceDuration not implemented yet!");
  return 0.0;
}

double GaitPatternFreeGait::getSwingPhaseForLeg(int iLeg, double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getSwingPhaseForLeg not implemented yet!");
  return 0.0;
}

double GaitPatternFreeGait::getStancePhaseForLeg(int iLeg, double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getStancePhaseForLeg not implemented yet!");
  return 0.0;
}

int GaitPatternFreeGait::getNumberOfStanceLegs(double stridePhase) const {
  throw std::runtime_error("GaitPatternFreeGait::getNumberOfStanceLegs not implemented yet!");
  return 0;
}

}  // namespace loco
