/******************************************************************************
 * Authors:                                                                   *
 *    C. Dario Bellicoso                                                      *
 ******************************************************************************/

#include "rqt_loco_visualizer/GaitPatternContainer.h"

namespace rqt_loco_visualizer {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

GaitPatternContainer::GaitPatternContainer()
    : liftOffPhase_(4, 0),
      touchDownPhase_(4, 0),
      duration_(0.0),
      startPhase_(0.0),
      stancePhase_(4, 0),
      swingPhase_(4, 0),
      shouldBeGrounded_(4, true),
      stridePhase_(0.0) {
  isInitialized_ = true;
}

GaitPatternContainer::~GaitPatternContainer() {

}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

bool GaitPatternContainer::isInitialized() const {
  return isInitialized_;
}

double GaitPatternContainer::getFootLiftOffPhase(int leg) const {
  return liftOffPhase_[leg];
}

double GaitPatternContainer::getFootTouchDownPhase(int leg) const {
  return touchDownPhase_[leg];
}

double GaitPatternContainer::getStancePhaseForLeg(int leg) const {
  return stancePhase_[leg];
}

double GaitPatternContainer::getSwingPhaseForLeg(int leg) const {
  return swingPhase_[leg];
}

double GaitPatternContainer::getStridePhase() const {
  return stridePhase_;
}

bool GaitPatternContainer::shouldBeGrounded(int leg) const {
  return shouldBeGrounded_[leg];
}

void GaitPatternContainer::setLiftOffPhase(double phase, int leg) {
  liftOffPhase_[leg] = phase;
}

void GaitPatternContainer::setTouchDownPhase(double phase, int leg) {
  touchDownPhase_[leg] = phase;
}

void GaitPatternContainer::setStancePhase(double phase, int leg) {
  stancePhase_[leg] = phase;
}

void GaitPatternContainer::setSwingPhase(double phase, int leg) {
  swingPhase_[leg] = phase;
}

void GaitPatternContainer::setStridePhase(double phase) {
  stridePhase_ = phase;
}

void GaitPatternContainer::setShouldBeGrounded(bool shouldBeGrounded, int leg) {
  shouldBeGrounded_[leg] = shouldBeGrounded;
}

void GaitPatternContainer::setDuration(double duration) {
  duration_ = duration;
}

double GaitPatternContainer::getDuration() const {
  return duration_;
}

void GaitPatternContainer::setStartPhase(double startPhase) {
  startPhase_ = startPhase;
}

double GaitPatternContainer::getStartPhase() const {
  return startPhase_;
}

} // namespace
