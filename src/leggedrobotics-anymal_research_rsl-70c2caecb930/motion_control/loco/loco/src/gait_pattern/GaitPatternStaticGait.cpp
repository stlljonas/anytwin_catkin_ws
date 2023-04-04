/*
 * GaitPatternStaticGait.cpp
 *
 *  Created on: Feb 3, 2015
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/gait_pattern/GaitPatternStaticGait.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace color = message_logger::color;

namespace loco {

GaitPatternStaticGait::GaitPatternStaticGait(WholeBody& wholeBody)
    : GaitPatternFlightPhases(wholeBody),
      stanceDiagonalDuration_(0.0),
      stanceLateralDuration_(0.0),
      swingDuration_(0.0),
      stanceDiagonalDurationInit_(0.0),
      stanceLateralDurationInit_(0.0),
      strideDurationInit_(0.0),
      stanceDiagonalDurationMin_(0.0),
      stanceLateralDurationMin_(0.0),
      strideDurationMin_(0.0),
      maxHeadingVelocity_(0.0),
      useGaitPatternFw_(true),
      swingLegIndexes_(wholeBody.getLegs().size()),
      swingLegIndexNow_(-1),
      swingLegIndexLast_(-1),
      swingLegIndexNext_(-1),
      swingLegIndexOverNext_(-1) {}

bool GaitPatternStaticGait::addVariablesToLog(bool /*update*/) {
  signal_logger::add(strideDuration_, "/stride_duration", "/loco/gp", "s");
  signal_logger::add(stanceDiagonalDuration_, "/stance_diagonal_duration", "/loco/gp", "s");
  signal_logger::add(stanceLateralDuration_, "/stance_lateral_duration", "/loco/gp", "s");
  return true;
}

const std::vector<int>& GaitPatternStaticGait::getSwingLegIndexes() const {
  return swingLegIndexes_;
}

double GaitPatternStaticGait::getDiagonalStanceDuration() const {
  return stanceDiagonalDuration_;
}

double GaitPatternStaticGait::getDiagonalStanceDurationSeconds() const {
  return stanceDiagonalDuration_ * strideDuration_;
}

double GaitPatternStaticGait::getLateralStanceDuration() const {
  return stanceLateralDuration_;
}

double GaitPatternStaticGait::getLateralStanceDurationSeconds() const {
  return stanceLateralDuration_ * strideDuration_;
}

double GaitPatternStaticGait::getSwingDuration() const {
  return swingDuration_;
}

double GaitPatternStaticGait::getSwingDurationSeconds() const {
  return swingDuration_ * strideDuration_;
}

bool GaitPatternStaticGait::advance(double dt) {
  if (!isLocked_) {
    updateGaitPattern(dt);
  }

  updateSwingLegsIndexes();

  torso_.setStridePhase(getStridePhase());
  double headingVelocity = 0.0;
  const double angularVelocity = torso_.getDesiredState().getAngularVelocityBaseInControlFrame().z();

  const double angularVelocityThreshold = 0.01;

  if (std::abs(angularVelocity) <= angularVelocityThreshold) {
    headingVelocity = torso_.getDesiredState().getLinearVelocityTargetInControlFrame().x();
  }

  headingVelocity = headingVelocityFilter_.advance(headingVelocity);

  const double tsHat = strideDurationMin_ - strideDurationInit_;
  const double tsTilde = tsHat / maxHeadingVelocity_ * std::abs(headingVelocity);
  strideDuration_ = strideDurationInit_ + tsTilde;
  updateFootfallPattern();

  for (auto leg : legs_) {
    const int iLeg = static_cast<int>(leg->getId());
    //--- defined by the "planning" / timing
    leg->getContactSchedulePtr()->setShouldBeGrounded(shouldBeLegGrounded(iLeg));
    leg->getContactSchedulePtr()->setStanceDuration(getStanceDuration(iLeg));
    leg->getContactSchedulePtr()->setSwingDuration(getStrideDuration() - getStanceDuration(iLeg));
    //---

    //--- timing measurements
    leg->getContactSchedulePtr()->setPreviousSwingPhase(leg->getContactSchedule().getSwingPhase());
    leg->getContactSchedulePtr()->setSwingPhase(getSwingPhaseForLeg(iLeg));
    leg->getContactSchedulePtr()->setPreviousStancePhase(leg->getContactSchedule().getStancePhase());
    leg->getContactSchedulePtr()->setStancePhase(getStancePhaseForLeg(iLeg));
    //---
  }

  return true;
}

std::ostream& operator<<(std::ostream& out, const GaitPatternStaticGait& gaitPattern) {
  out << std::endl;
  out << "----------------------------" << std::endl;
  out << "Class: GaitPatternStaticGait" << std::endl;
  out << "swing leg indexes: " << std::endl
      << color::blue << "\tcurrent: " << color::red << gaitPattern.getCurrentSwingLeg() << color::blue << "\tlast: " << color::red
      << gaitPattern.getLastSwingLeg() << color::blue << "\tnext: " << color::red << gaitPattern.getNextSwingLeg() << color::blue
      << "\tovernext: " << color::red << gaitPattern.getOverNextSwingLeg() << color::def << std::endl;
  out << "----------------------------" << std::endl;
  out << std::endl;

  return out;
}

int GaitPatternStaticGait::getIndexOfSwingLeg() const {
  int swingLeg = -1;
  for (auto leg : legs_) {
    //    if (leg->getSwingPhase() != -1) {
    if (getSwingPhaseForLeg(static_cast<int>(leg->getId())) != -1) {
      swingLeg = static_cast<int>(leg->getId());
    }
  }
  return swingLeg;
}

int GaitPatternStaticGait::getIndexOfSwingLeg(double stridePhase) const {
  int swingLeg = -1;
  for (auto leg : legs_) {
    //    if (leg->getSwingPhase() != -1) {
    if (getSwingPhaseForLeg(static_cast<int>(leg->getId()), stridePhase) != -1) {
      swingLeg = static_cast<int>(leg->getId());
    }
  }
  return swingLeg;
}

int GaitPatternStaticGait::getIndexOfNextSwingLeg(const int currentSwingFoot) const {
  int nextSwingFoot;
  int currentSwingFootVectorIndex =
      std::find(swingLegIndexes_.begin(), swingLegIndexes_.end(), currentSwingFoot) - swingLegIndexes_.begin();
  if (currentSwingFootVectorIndex == (legs_.size() - 1)) {
    nextSwingFoot = swingLegIndexes_[0];
  } else {
    nextSwingFoot = swingLegIndexes_[currentSwingFootVectorIndex + 1];
  }

  return nextSwingFoot;
}

int GaitPatternStaticGait::getIndexOfPreviousSwingLeg(const int currentSwingFoot) const {
  int previousSwingFoot;
  int currentSwingFootVectorIndex =
      std::find(swingLegIndexes_.begin(), swingLegIndexes_.end(), currentSwingFoot) - swingLegIndexes_.begin();

  if (currentSwingFootVectorIndex == 0) {
    previousSwingFoot = swingLegIndexes_.back();
  } else {
    previousSwingFoot = swingLegIndexes_[currentSwingFootVectorIndex - 1];
  }

  return previousSwingFoot;
}

void GaitPatternStaticGait::updateSwingLegsIndexes() {
  swingLegIndexNow_ = getIndexOfSwingLeg();

  int swingLeg = getIndexOfSwingLeg();

  if (swingLeg != -1) {
    swingLegIndexLast_ = swingLegIndexNow_;
    swingLegIndexNext_ = getIndexOfNextSwingLeg(swingLegIndexNow_);
    swingLegIndexOverNext_ = getIndexOfNextSwingLeg(swingLegIndexNext_);
  }
}

bool GaitPatternStaticGait::isFullStancePhase() const {
  //  if (getIndexOfSwingLeg() == -1) {
  //    return true;
  //  }
  //  else {
  //    return false;
  //  }
  bool isFullStancePhase = true;
  for (auto leg : legs_) {
    isFullStancePhase &= leg->getContactSchedule().shouldBeGrounded();
  }
  return isFullStancePhase;
}

bool GaitPatternStaticGait::loadParameters(const TiXmlHandle& handle) {
  initCyclePhase_ = 0.0;

  TiXmlElement* pElem;

  /* gait pattern */
  TiXmlHandle hFootFallPattern(handle.FirstChild("LimbCoordination").FirstChild("GaitPattern").FirstChild("StaticGait"));
  pElem = hFootFallPattern.Element();
  if (pElem == nullptr) {
    printf("Could not find GaitPattern::StaticGait\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("cycleDuration", &strideDurationInit_) != TIXML_SUCCESS) {
    printf("Could not find GaitPattern::StaticGait::cycleDuration\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("cycleDurationMin", &strideDurationMin_) != TIXML_SUCCESS) {
    printf("Could not find GaitPattern::StaticGait::cycleDurationMin\n");
    return false;
  }

  // diagonal stance
  pElem = hFootFallPattern.FirstChild("DiagonalSwitch").Element();
  if (pElem == nullptr) {
    printf("Could not find GaitPattern::StaticGait::DiagonalSwitch\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("initial", &stanceDiagonalDurationInit_) != TIXML_SUCCESS) {
    printf("Could not find GaitPattern::StaticGait::DiagonalSwitch::initial\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("min", &stanceDiagonalDurationMin_) != TIXML_SUCCESS) {
    printf("Could not find GaitPattern::StaticGait::DiagonalSwitch::min\n");
    return false;
  }

  // lateral stance
  pElem = hFootFallPattern.FirstChild("LateralSwitch").Element();
  if (pElem == nullptr) {
    printf("Could not find GaitPattern::StaticGait::LateralSwitch\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("initial", &stanceLateralDurationInit_) != TIXML_SUCCESS) {
    printf("Could not find GaitPattern::StaticGait::LateralSwitch::initial\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("min", &stanceLateralDurationMin_) != TIXML_SUCCESS) {
    printf("Could not find GaitPattern::StaticGait::LateralSwitch::min\n");
    return false;
  }

  // swing duration
  pElem = hFootFallPattern.FirstChild("SwingDuration").Element();
  if (pElem == nullptr) {
    printf("Could not find GaitPattern::StaticGait::SwingDuration\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("initial", &swingDuration_) != TIXML_SUCCESS) {
    printf("Could not find GaitPattern::StaticGait::SwingDuration::initial\n");
    return false;
  }

  // max velocity
  pElem = hFootFallPattern.FirstChild("Maximum").Element();
  if (pElem == nullptr) {
    printf("Could not find GaitPattern::StaticGait::Maximum\n");
    return false;
  }
  if (pElem->QueryDoubleAttribute("headingSpeed", &maxHeadingVelocity_) != TIXML_SUCCESS) {
    printf("Could not find GaitPattern::StaticGait::Maximum::headingSpeed\n");
    return false;
  }

  stanceDiagonalDuration_ = stanceDiagonalDurationInit_;
  stanceLateralDuration_ = stanceLateralDurationInit_;
  strideDuration_ = strideDurationInit_;

  updateFootfallPattern();

  numGaitCycles_ = 0;

  return true;
}

bool GaitPatternStaticGait::initialize(double dt) {
  bool success = true;

  headingVelocityFilter_.setFilterParameters(dt, 2.5, 1.0, 0.0);

  success &= GaitPatternFlightPhases::initialize(dt);

  resetSwingIndexes(true);

  return success;
}

void GaitPatternStaticGait::resetSwingIndexes(bool wasStanding) {
  if (wasStanding) {
    swingLegIndexLast_ = swingLegIndexes_[3];
  } else {
    double stridePhase = cyclePhase_;
    while (getIndexOfSwingLeg(stridePhase) == -1) {
      stridePhase -= 0.01;
    }
    swingLegIndexLast_ = getIndexOfSwingLeg(stridePhase);
  }
  swingLegIndexNow_ = swingLegIndexLast_;
  swingLegIndexNext_ = getIndexOfNextSwingLeg(swingLegIndexLast_);
  swingLegIndexOverNext_ = getIndexOfNextSwingLeg(swingLegIndexNext_);
}

void GaitPatternStaticGait::setUseGaitPatternFw(bool setGaitPattern) {
  useGaitPatternFw_ = setGaitPattern;
}

int GaitPatternStaticGait::getNextSwingLeg() const {
  return swingLegIndexNext_;
}
int GaitPatternStaticGait::getLastSwingLeg() const {
  return swingLegIndexLast_;
}
int GaitPatternStaticGait::getOverNextSwingLeg() const {
  return swingLegIndexOverNext_;
}
int GaitPatternStaticGait::getCurrentSwingLeg() const {
  return swingLegIndexNow_;
}

void GaitPatternStaticGait::updateFootfallPattern() {
  /* foot fall pattern */
  stepPatterns_.clear();
  double liftOff;
  double touchDown;

  if (useGaitPatternFw_) {
    // RH
    liftOff = stanceDiagonalDuration_;
    touchDown = liftOff + swingDuration_;
    stepPatterns_.emplace_back(3, liftOff, touchDown);

    // RF
    liftOff = touchDown + stanceLateralDuration_;
    touchDown = liftOff + swingDuration_;
    stepPatterns_.emplace_back(1, liftOff, touchDown);

    // LH
    liftOff = touchDown + stanceDiagonalDuration_;
    touchDown = liftOff + swingDuration_;
    stepPatterns_.emplace_back(2, liftOff, touchDown);

    // LF
    liftOff = touchDown + stanceLateralDuration_;
    touchDown = liftOff + swingDuration_;
    stepPatterns_.emplace_back(0, liftOff, touchDown);

    swingLegIndexes_[0] = 3;
    swingLegIndexes_[1] = 1;
    swingLegIndexes_[2] = 2;
    swingLegIndexes_[3] = 0;

  } else {
    // RF
    liftOff = stanceDiagonalDuration_;
    touchDown = liftOff + swingDuration_;
    stepPatterns_.emplace_back(1, liftOff, touchDown);

    // RH
    liftOff = touchDown + stanceLateralDuration_;
    touchDown = liftOff + swingDuration_;
    stepPatterns_.emplace_back(3, liftOff, touchDown);

    // LF
    liftOff = touchDown + stanceDiagonalDuration_;
    touchDown = liftOff + swingDuration_;
    stepPatterns_.emplace_back(0, liftOff, touchDown);

    // LH
    liftOff = touchDown + stanceLateralDuration_;
    touchDown = liftOff + swingDuration_;
    stepPatterns_.emplace_back(2, liftOff, touchDown);

    swingLegIndexes_[0] = 1;
    swingLegIndexes_[1] = 3;
    swingLegIndexes_[2] = 0;
    swingLegIndexes_[3] = 2;
  }
}

} /* namespace loco */
