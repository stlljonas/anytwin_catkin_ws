/*
 * ContactSchedule.cpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/common/legs/ContactSchedule.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

namespace loco {

ContactSchedule::ContactSchedule()
    : stancePhase_(0.0),
      previousStancePhase_(0.0),
      swingPhase_(0.0),
      previousSwingPhase_(0.0),
      stanceDuration_(0.0),
      swingDuration_(0.0),
      isGrounded_(false),
      wasGrounded_(false),
      shouldBeGrounded_(false),
      isSlipping_(false),
      isLosingContact_(false),
      isInStandConfiguration_(false),
      lastOrCurrentContactPhase_(0.0),
      interpolationParameter_(0.0) {}

bool ContactSchedule::initialize(double) {
  stancePhase_ = 0.0;
  previousStancePhase_ = 0.0;
  swingPhase_ = 0.0;
  previousSwingPhase_ = 0.0;
  stanceDuration_ = 0.0;
  swingDuration_ = 0.0;
  isGrounded_ = false;
  wasGrounded_ = false;
  shouldBeGrounded_ = false;
  isSlipping_ = false;
  isLosingContact_ = false;
  isInStandConfiguration_ = false;
  lastOrCurrentContactPhase_ = 0.0;
  interpolationParameter_ = 0.0;

  return true;
}

void ContactSchedule::setLastOrCurrentContactPhase(double stridePhase) {
  lastOrCurrentContactPhase_ = stridePhase;
}

double ContactSchedule::getLastOrCurrentContactPhase() const {
  return lastOrCurrentContactPhase_;
}

double ContactSchedule::getStancePhase() const {
  return stancePhase_;
}
double ContactSchedule::getSwingPhase() const {
  return swingPhase_;
}

double ContactSchedule::getStanceDuration() const {
  return stanceDuration_;
}

double ContactSchedule::getSwingDuration() const {
  return swingDuration_;
}

void ContactSchedule::setPreviousStancePhase(double previousStancePhase) {
  previousStancePhase_ = previousStancePhase;
}

double ContactSchedule::getPreviousStancePhase() const {
  return previousStancePhase_;
}

void ContactSchedule::setPreviousSwingPhase(double previousSwingPhase) {
  previousSwingPhase_ = previousSwingPhase;
}

double ContactSchedule::getPreviousSwingPhase() const {
  return previousSwingPhase_;
}

bool ContactSchedule::isGrounded() const {
  return isGrounded_;
}

bool ContactSchedule::wasGrounded() const {
  return wasGrounded_;
}

bool ContactSchedule::shouldBeGrounded() const {
  return shouldBeGrounded_;
}

bool ContactSchedule::isAndShouldBeGrounded() const {
  return (isGrounded_ && shouldBeGrounded_);
}

bool ContactSchedule::isSlipping() const {
  return isSlipping_;
}

void ContactSchedule::setStancePhase(double phase) {
  stancePhase_ = phase;
}

void ContactSchedule::setSwingPhase(double phase) {
  swingPhase_ = phase;
}

void ContactSchedule::setStanceDuration(double duration) {
  stanceDuration_ = duration;
}

void ContactSchedule::setSwingDuration(double duration) {
  swingDuration_ = duration;
}

void ContactSchedule::setIsGrounded(bool isGrounded) {
  isGrounded_ = isGrounded;
}

void ContactSchedule::setWasGrounded(bool wasGrounded) {
  wasGrounded_ = wasGrounded;
}

void ContactSchedule::setShouldBeGrounded(bool shouldBeGrounded) {
  shouldBeGrounded_ = shouldBeGrounded;
}

void ContactSchedule::setIsSlipping(bool isSlipping) {
  isSlipping_ = isSlipping;
}

bool ContactSchedule::isLosingContact() const {
  return isLosingContact_;
}

void ContactSchedule::setIsLosingContact(bool isLosingContact) {
  isLosingContact_ = isLosingContact;
}

void ContactSchedule::setIsInStandConfiguration(bool isInStandConfiguration) {
  isInStandConfiguration_ = isInStandConfiguration;
}

bool ContactSchedule::isInStandConfiguration() const {
  return isInStandConfiguration_;
}

void ContactSchedule::setInterpolationParameter(double interpolationParameter) {
  interpolationParameter_ = interpolationParameter;
}

double ContactSchedule::getInterpolationParameter() const {
  return interpolationParameter_;
}

bool ContactSchedule::addVariablesToLog(const std::string& ns) const {
  // Log leg state flags.
  signal_logger::add(isGrounded_, "isGrounded", ns);
  signal_logger::add(shouldBeGrounded_, "shouldBeGrounded", ns);
  signal_logger::add(isSlipping_, "isSlipping", ns);
  signal_logger::add(isLosingContact_, "isLosingContact", ns);
  signal_logger::add(isInStandConfiguration_, "isInStandConfiguration", ns);

  // Log gait plan data.
  signal_logger::add(stancePhase_, "stancePhase", ns);
  signal_logger::add(swingPhase_, "swingPhase", ns);
  signal_logger::add(stanceDuration_, "stanceDuration", ns);
  signal_logger::add(swingDuration_, "swingDuration", ns);

  // Log motion plan parameters.
  signal_logger::add(interpolationParameter_, "interpolationParameter", ns);

  return true;
}

} /* namespace loco */
