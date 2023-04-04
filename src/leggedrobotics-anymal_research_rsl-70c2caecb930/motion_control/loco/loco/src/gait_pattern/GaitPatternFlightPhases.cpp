/*
 * GaitPatternFlightPhases.cpp
 *
 *  Created on: Mar 14, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

// loco
#include <loco/gait_pattern/GaitPatternFlightPhases.hpp>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utis
#include <robot_utils/math/math.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// stl
#include <stdexcept>

namespace loco {

GaitPatternFlightPhases::GaitPatternFlightPhases(WholeBody& wholeBody)
    : torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      isInitialized_(false),
      cyclePhase_(0.0),
      strideDuration_(0.0),
      initCyclePhase_(0.0),
      numGaitCycles_(0),
      stepPatterns_(wholeBody.getLegs().size()) {}

bool GaitPatternFlightPhases::addVariablesToLog(bool /*update*/) {
  signal_logger::add(cyclePhase_, "/cycle_phase", "/gait_pattern", "s");
  signal_logger::add(strideDuration_, "/stride_duration", "/gait_pattern", "s");
  signal_logger::add(numGaitCycles_, "/num_gait_cycles", "/gait_pattern", "s");

  return true;
}

bool GaitPatternFlightPhases::loadParameters(const TiXmlHandle& handle) {
  // Get handles.
  TiXmlHandle limbCoordinationHandle = handle;
  if (!tinyxml_tools::getChildHandle(limbCoordinationHandle, handle, "LimbCoordination")) {
    return false;
  }
  TiXmlHandle gaitPatternHandle = handle;
  if (!tinyxml_tools::getChildHandle(gaitPatternHandle, limbCoordinationHandle, "GaitPattern")) {
    return false;
  }
  TiXmlHandle flightPhasesHandle = handle;
  if (!tinyxml_tools::getChildHandle(flightPhasesHandle, gaitPatternHandle, "FlightPhases")) {
    return false;
  }

  // Load parameters.
  if (!tinyxml_tools::loadParameter(strideDuration_, flightPhasesHandle, "cycleDuration")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(initCyclePhase_, flightPhasesHandle, "initCyclePhase")) {
    return false;
  }

  // Load foot fall pattern.
  TiXmlHandle lfHandle = handle;
  if (!tinyxml_tools::getChildHandle(lfHandle, flightPhasesHandle, "LF")) {
    return false;
  }
  TiXmlHandle rfHandle = handle;
  if (!tinyxml_tools::getChildHandle(rfHandle, flightPhasesHandle, "RF")) {
    return false;
  }
  TiXmlHandle lhHandle = handle;
  if (!tinyxml_tools::getChildHandle(lhHandle, flightPhasesHandle, "LH")) {
    return false;
  }
  TiXmlHandle rhHandle = handle;
  if (!tinyxml_tools::getChildHandle(rhHandle, flightPhasesHandle, "RH")) {
    return false;
  }
  stepPatterns_.clear();

  double liftOff = 0.0;
  double touchDown = 0.0;

  if (!tinyxml_tools::loadParameter(liftOff, lfHandle, "liftOff")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(touchDown, lfHandle, "touchDown")) {
    return false;
  }
  stepPatterns_.emplace_back(FootFallPattern(0, liftOff, touchDown));

  if (!tinyxml_tools::loadParameter(liftOff, rfHandle, "liftOff")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(touchDown, rfHandle, "touchDown")) {
    return false;
  }
  stepPatterns_.emplace_back(FootFallPattern(1, liftOff, touchDown));

  if (!tinyxml_tools::loadParameter(liftOff, lhHandle, "liftOff")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(touchDown, lhHandle, "touchDown")) {
    return false;
  }
  stepPatterns_.emplace_back(FootFallPattern(2, liftOff, touchDown));

  if (!tinyxml_tools::loadParameter(liftOff, rhHandle, "liftOff")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(touchDown, rhHandle, "touchDown")) {
    return false;
  }
  stepPatterns_.emplace_back(FootFallPattern(3, liftOff, touchDown));

  numGaitCycles_ = 0;
  return true;
}

double GaitPatternFlightPhases::getStrideDuration() const {
  return strideDuration_;
}

void GaitPatternFlightPhases::setStrideDuration(double strideDuration) {
  strideDuration_ = strideDuration;
}

double GaitPatternFlightPhases::getSwingPhaseForLeg(int iLeg) const {
  return getSwingPhaseForLeg(iLeg, cyclePhase_);
}

double GaitPatternFlightPhases::getSwingPhaseForLeg(int iLeg, double stridePhase) const {
  int pIndex = getStepPatternIndexForLeg(iLeg);
  // by default all limbs are in stance mode
  if (pIndex == -1) {
    return -1;
  }

  if (stepPatterns_[pIndex].liftOffPhase_ == stepPatterns_[pIndex].strikePhase_) {
    return -1;  // added by Christian
  }

  const double timeUntilFootLiftOff = stepPatterns_[pIndex].getPhaseLeftUntilLiftOff(stridePhase);
  const double timeUntilFootStrike = stepPatterns_[pIndex].getPhaseLeftUntilStrike(stridePhase);

  // see if we're not in swing mode...
  if (timeUntilFootStrike > timeUntilFootLiftOff) {
    return -1;
  }

  const double swingPhaseRange = stepPatterns_[pIndex].strikePhase_ - stepPatterns_[pIndex].liftOffPhase_;

  return 1 - timeUntilFootStrike / swingPhaseRange;
}

double GaitPatternFlightPhases::getStancePhaseForLeg(int iLeg, double stridePhase) const {
  const int pIndex = getStepPatternIndexForLeg(iLeg);
  if (pIndex == -1) {
    return -1;  // changed with new gait pattern from SC
  }

  const double timeUntilFootLiftOff = stepPatterns_[pIndex].getPhaseLeftUntilLiftOff(stridePhase);
  const double timeUntilFootStrike = stepPatterns_[pIndex].getPhaseLeftUntilStrike(stridePhase);

  if (stepPatterns_[pIndex].strikePhase_ == stepPatterns_[pIndex].liftOffPhase_) {
    return 0.0;  // added (Christian)
  }

  // see if we're in swing mode...
  if (timeUntilFootStrike < timeUntilFootLiftOff) {
    //    return 0.0>;
    return -1;  // changed with new gait pattern from SC
  }

  const double swingPhaseRange = stepPatterns_[pIndex].strikePhase_ - stepPatterns_[pIndex].liftOffPhase_;
  const double timeSinceFootStrike = 1.0 - timeUntilFootLiftOff - swingPhaseRange;

  return timeSinceFootStrike / (timeSinceFootStrike + timeUntilFootLiftOff);
}

double GaitPatternFlightPhases::getStancePhaseForLeg(int iLeg) const {
  return getStancePhaseForLeg(iLeg, cyclePhase_);
}

double GaitPatternFlightPhases::getStanceDuration(int iLeg) const {
  return getStanceDuration(iLeg, strideDuration_);
}

double GaitPatternFlightPhases::getSwingDuration(int iLeg) const {
  return getSwingDuration(iLeg, strideDuration_);
}

double GaitPatternFlightPhases::getSwingDuration(int iLeg, double strideDuration) const {
  return strideDuration - getStanceDuration(iLeg, strideDuration);
}

double GaitPatternFlightPhases::getStanceDuration(int iLeg, double strideDuration) const {
  int pIndex = getStepPatternIndexForLeg(iLeg);
  return (1.0 - (stepPatterns_[pIndex].strikePhase_ - stepPatterns_[pIndex].liftOffPhase_)) * strideDuration;
}

unsigned long int GaitPatternFlightPhases::getNGaitCycles() const {
  return numGaitCycles_;
}

bool GaitPatternFlightPhases::initialize(double /*dt*/) {
  numGaitCycles_ = 0;
  cyclePhase_ = initCyclePhase_;
  isInitialized_ = true;
  updateTorsoAndLegs();
  return true;
}

bool GaitPatternFlightPhases::isInitialized() const {
  return isInitialized_;
}

void GaitPatternFlightPhases::updateGaitPattern(double dt) {
  if (strideDuration_ == 0.0) {
    cyclePhase_ = 0.0;
    return;
  }
  cyclePhase_ += dt / strideDuration_;
  if (cyclePhase_ >= 1.0) {
    cyclePhase_ = 0.0;
    ++numGaitCycles_;
  }
}

void GaitPatternFlightPhases::setStridePhaseAndUpdate(double stridePhase) {
  cyclePhase_ = stridePhase;
  updateTorsoAndLegs();  // todo?
}

bool GaitPatternFlightPhases::advance(double dt) {
  if (!isLocked_) {
    updateGaitPattern(dt);
  }
  // If the gait pattern has been changed in the meanwhile, the legs will be still updated.
  updateTorsoAndLegs();
  return true;
}

void GaitPatternFlightPhases::updateTorsoAndLegs() {
  torso_.setStridePhase(getStridePhase());
  torso_.setStrideDuration(getStrideDuration());

  for (auto leg : legs_) {
    const auto iLeg = leg->getId();

    //--- defined by the "planning" / timing
    leg->getContactSchedulePtr()->setShouldBeGrounded(shouldBeLegGrounded(iLeg));
    leg->getContactSchedulePtr()->setStanceDuration(getStanceDuration(iLeg));
    leg->getContactSchedulePtr()->setSwingDuration(getSwingDuration(iLeg));
    //---

    //--- timing measurements
    leg->getContactSchedulePtr()->setPreviousSwingPhase(leg->getContactSchedule().getSwingPhase());
    leg->getContactSchedulePtr()->setSwingPhase(getSwingPhaseForLeg(iLeg));
    leg->getContactSchedulePtr()->setPreviousStancePhase(leg->getContactSchedule().getStancePhase());
    leg->getContactSchedulePtr()->setStancePhase(getStancePhaseForLeg(iLeg));
    //---
  }
}

bool GaitPatternFlightPhases::shouldBeLegGrounded(int iLeg) const {
  //  printf("leg %d: stance phase: %f\n",iLeg, getStancePhaseForLeg(iLeg));
  //  return (getStancePhaseForLeg(iLeg)!=0.0);
  return (getStancePhaseForLeg(iLeg) >= 0.0);  // changed since returns now -1 for swing mode
  //  return (getStancePhaseForLeg(iLeg) !=0.0 || footFallPatterns[iLeg].liftOffPhase == footFallPatterns[iLeg].strikePhase); // added
  //  (Christian)
}

double GaitPatternFlightPhases::getStridePhase() const {
  return cyclePhase_;
}

void GaitPatternFlightPhases::setStridePhase(double stridePhase) {
  cyclePhase_ = stridePhase;
}

inline double GaitPatternFlightPhases::getRelativePhaseFromAbsolutePhaseInRange(double phase, double start, double end) const {
  if (start < 0) {
    end -= start;
    phase -= start;
    start -= start;
    if (phase > 1) {
      phase -= 1;
    }
  }
  if (end > 1) {
    start -= (end - 1);
    phase -= (end - 1);
    end -= (end - 1);
    if (phase < 0) {
      phase += 1;
    }
  }
  //  printf("start: %lf, end: %lf\n", start, end);
  if (start == end) {
    return -1;
  }  // added (Christian)
  double result = (phase - start) / (end - start);

  if (result < 0 || result > 1) {
    result = -1;
  }

  return result;
}

double GaitPatternFlightPhases::getFootLiftOffPhase(int iLeg) const {
  int pIndex = getStepPatternIndexForLeg(iLeg);
  assert(pIndex != -1);
  return stepPatterns_[pIndex].liftOffPhase_;
}
double GaitPatternFlightPhases::getFootTouchDownPhase(int iLeg) const {
  int pIndex = getStepPatternIndexForLeg(iLeg);
  assert(pIndex != -1);
  return stepPatterns_[pIndex].strikePhase_;
}

double GaitPatternFlightPhases::getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const {
  double stancePhase = getStancePhaseForLeg(iLeg, stridePhase);
  if (stancePhase < 0 || stancePhase > 1) {
    return 0;
  }
  return getStanceDuration(iLeg, strideDuration) * (1 - stancePhase);
}

double GaitPatternFlightPhases::getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const {
  double swingPhase = getSwingPhaseForLeg(iLeg, stridePhase);
  if (swingPhase < 0 || swingPhase > 1) {
    return 0;
  }
  return getSwingDuration(iLeg, strideDuration) * (1 - swingPhase);
}

double GaitPatternFlightPhases::getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const {
  double stancePhase = getStancePhaseForLeg(iLeg, stridePhase);
  if (stancePhase < 0 || stancePhase > 1) {
    return 0;
  }
  return getStanceDuration(iLeg, strideDuration) * stancePhase;
}

double GaitPatternFlightPhases::getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const {
  double swingPhase = getSwingPhaseForLeg(iLeg, stridePhase);
  if (swingPhase < 0 || swingPhase > 1) {
    return 0;
  }
  return getSwingDuration(iLeg, strideDuration) * swingPhase;
}

double GaitPatternFlightPhases::getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const {
  double stancePhase = getStancePhaseForLeg(iLeg, stridePhase);
  // the limb is already in stance phase, so the time until the next stance phase starts is one whole stridePhase minus the amount of time
  // it's already spent in stance mode
  if (stancePhase >= 0 && stancePhase <= 1) {
    return strideDuration - getTimeSpentInStance(iLeg, strideDuration, stridePhase);
  }
  // if the limb is in swing phase, then we have as much time until the swing phase ends
  return getTimeLeftInSwing(iLeg, strideDuration, stridePhase);
}

double GaitPatternFlightPhases::getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const {
  double swingPhase = getSwingPhaseForLeg(iLeg, stridePhase);
  //  printf("Swing phase: %lf\n", swingPhase);
  // the limb is already in swing phase, so the time until the next swing phase starts is one whole stridePhase minus the amount of time
  // it's already spent in swing mode
  if (swingPhase >= 0 && swingPhase <= 1) {
    return strideDuration - getTimeSpentInSwing(iLeg, strideDuration, stridePhase);
  }
  // if the limb is in stance phase, then we have as much time until the stance phase ends
  return getTimeLeftInStance(iLeg, strideDuration, stridePhase);
}

int GaitPatternFlightPhases::getNumberOfStanceLegs(double stridePhase) const {
  int nStanceLegs = 0;
  for (int i = 0; i < stepPatterns_.size(); i++) {
    if (getStancePhaseForLeg(i, stridePhase) >= 0.0) {
      nStanceLegs++;
    }
  }
  return nStanceLegs;
}

bool GaitPatternFlightPhases::setToInterpolated(const GaitPatternBase& gaitPattern1, const GaitPatternBase& gaitPattern2, double t) {
  const auto& gait1 = dynamic_cast<const GaitPatternFlightPhases&>(gaitPattern1);
  const auto& gait2 = dynamic_cast<const GaitPatternFlightPhases&>(gaitPattern2);

  robot_utils::boundToRange(&t, 0, 1);
  // assume an exact correspondance here (i.e. each foot has the same number of foot fall patterns during a stride).
  // we'll also assume that the order the leg foot pattern is specified is the same...
  strideDuration_ = robot_utils::linearlyInterpolate(gait1.strideDuration_, gait2.strideDuration_, 0, 1, t);

  stepPatterns_.clear();
  //  if (gait1.stepPatterns_.size() != gait2.stepPatterns_.size())
  //    throw std::runtime_error("Don't know how to interpolated between incompatible foot fall patterns");
  //  for (uint i=0;i<gait1.stepPatterns_.size();i++){
  //    const FootFallPattern& stepPattern1  = gait1.stepPatterns_[i];
  //    int pIndex = getStepPatternIndexForLeg(stepPattern1.legId_);
  //    assert(pIndex != 0);
  //    const FootFallPattern& stepPattern2 = gait2.stepPatterns_[pIndex];
  //    stepPatterns_.push_back(FootFallPattern(stepPattern1.legId_, robot_utils::linearlyInterpolate(stepPattern1.liftOffPhase,
  //    stepPattern2.liftOffPhase, 0, 1, t),
  //                                               robot_utils::linearlyInterpolate(stepPattern1.strikePhase, stepPattern2.strikePhase, 0,
  //                                               1, t)));
  //  }
  //
  if (gait1.stepPatterns_.size() != gait2.stepPatterns_.size()) {
    throw std::runtime_error("Don't know how to interpolated between incompatible foot fall patterns");
  }
  for (int i = 0; i < static_cast<int>(gait1.stepPatterns_.size()); i++) {
    stepPatterns_.emplace_back(
        gait1.stepPatterns_[i].legId_,
        robot_utils::linearlyInterpolate(gait1.stepPatterns_[i].liftOffPhase_, gait2.stepPatterns_[i].liftOffPhase_, 0, 1, t),
        robot_utils::linearlyInterpolate(gait1.stepPatterns_[i].strikePhase_, gait2.stepPatterns_[i].strikePhase_, 0, 1, t));
  }
  return true;
}

void GaitPatternFlightPhases::clear() {
  stepPatterns_.clear();
}

int GaitPatternFlightPhases::getNumberOfLegs() const {
  return static_cast<int>(stepPatterns_.size());
}

void GaitPatternFlightPhases::addFootFallPattern(int legId, double liftOffPhase, double strikePhase) {
  // ASSUMPTIONS:
  //  - the limb was going to be in swing mode at all times in the interval liftOffPhase and strikePhase
  //  - liftOffPhase happens before strikePhase
  //  - either times can exceed the 0-1 phase range which just means that the foot should already (still) be
  //    in swing mode, but at least one needs to be in the correct range (0-1)

  // see if this limb was added before - if not, add a new one...
  if (liftOffPhase >= strikePhase && (liftOffPhase != -1.0 && strikePhase != -1.0)) {
    throw std::runtime_error("addFootFallPattern: liftOffPhase needs to happen before the foot strike.");
  }

  if (strikePhase - liftOffPhase > 1 && (liftOffPhase != -1.0 && strikePhase != -1.0)) {
    throw std::runtime_error("addFootFallPattern: liftOffPhase and strikePhase should not be offset by more than 1.");
  }

  if ((liftOffPhase < 0 || liftOffPhase > 1) && (strikePhase < 0 || strikePhase > 1) && (liftOffPhase != -1.0 && strikePhase != -1.0)) {
    throw std::runtime_error("addFootFallPattern: Either liftOffPhase or strikePhase needs to be in the range 0-1.");
  }

  int pIndex = getStepPatternIndexForLeg(legId);
  if (pIndex == -1) {
    stepPatterns_.emplace_back(legId, liftOffPhase, strikePhase);
  } else {
    std::string msg{};
    msg.append("There is already a footfall pattern for this leg! ");
    msg.append("Index was ");
    msg.append(std::to_string(pIndex));
    throw std::runtime_error(msg.c_str());
  }
}

int GaitPatternFlightPhases::getStepPatternIndexForLeg(int legId) const {
  for (uint i = 0; i < stepPatterns_.size(); i++) {
    if (stepPatterns_[i].legId_ == legId) {
      return i;
    }
  }

  return -1;
}

void GaitPatternFlightPhases::setInitialPhase(double phase) {
  initCyclePhase_ = phase;
}

double GaitPatternFlightPhases::getInitialPhase() const {
  return initCyclePhase_;
}

std::ostream& operator<<(std::ostream& out, const GaitPatternFlightPhases& gaitPattern) {
  for (const auto& stepPattern : gaitPattern.stepPatterns_) {
    out << "Leg " << stepPattern.legId_;
    out << "\tlift-off: " << stepPattern.liftOffPhase_;
    out << "\ttouch-down: " << stepPattern.strikePhase_;
    out << std::endl;
  }
  return out;
}

} /* namespace loco */
