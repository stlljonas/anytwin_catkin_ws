/*
 * SteeringControllerModule.cpp
 *
 *  Created on: Oct 1, 2015
 *      Author: Philipp Leemann, Gabriel Hottiger
 */

// loco
#include "loco/driving_control/SteeringControllerModule.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace loco {

SteeringControllerModule::SteeringControllerModule() : steeringModes_(), activeMode_(0), observers_(), needsInitialization_(true) {}

bool SteeringControllerModule::addSteeringMode(const unsigned int imode, std::unique_ptr<SteeringModeModule>&& mode) {
  std::string name = mode->getModuleName();

  if (!steeringModes_.emplace(imode, std::move(mode)).second) {
    MELO_WARN("Could not add SteeringMode %s(Id: %d). A SteeringMode with same index already exists!", name.c_str(), imode);
  } else {
    MELO_INFO("Added SteeringMode %s(Id: %d).", name.c_str(), imode);
  }
  return true;
}

bool SteeringControllerModule::setActiveMode(const std::string& modeName, double dt) {
  auto it = std::find_if(steeringModes_.begin(), steeringModes_.end(),
                         [&](std::pair<const unsigned int, std::unique_ptr<loco::SteeringModeModule> >& mode) {
                           return (mode.second->getModuleName() == modeName);
                         });
  if (it == steeringModes_.end()) {
    MELO_INFO("Cannot activate unkown mode %s!", modeName.c_str());
    return false;
  }

  activeMode_ = it->first;
  for (auto observer : observers_) {
    observer->steeringModeChanged(steeringModes_.at(activeMode_)->getModuleName());
  }
  MELO_INFO("Switch to steering mode %s(Id: %d).", steeringModes_.at(activeMode_)->getModuleName().c_str(), activeMode_);
  needsInitialization_ = true;

  return true;
}

bool SteeringControllerModule::setActiveMode(const unsigned int imode, double dt) {
  if (steeringModes_.find(imode) == steeringModes_.end()) {
    MELO_INFO("Cannot activate unkown mode (Id: %d)!", imode);
    return false;
  }

  activeMode_ = imode;
  for (auto observer : observers_) {
    observer->steeringModeChanged(steeringModes_.at(activeMode_)->getModuleName());
  }
  MELO_INFO("Switch to steering mode %s(Id: %d).", steeringModes_.at(activeMode_)->getModuleName().c_str(), activeMode_);
  needsInitialization_ = true;

  return true;
}

bool SteeringControllerModule::nextMode(double dt) {
  auto mode = steeringModes_.find(activeMode_);

  if (mode == steeringModes_.end()) {
    MELO_INFO("Cannot activate next controller mode. Active mode is corrupted!");
    return false;
  }

  if (++mode == steeringModes_.end()) {
    mode = steeringModes_.begin();
  }

  activeMode_ = mode->first;
  for (auto observer : observers_) {
    observer->steeringModeChanged(steeringModes_.at(activeMode_)->getModuleName());
  }
  MELO_INFO("Switch to steering mode %s(Id: %d).", steeringModes_.at(activeMode_)->getModuleName().c_str(), activeMode_);
  needsInitialization_ = true;

  return true;
}

unsigned int SteeringControllerModule::getActiveMode() const {
  return activeMode_;
}

std::string SteeringControllerModule::getActiveModeName() const {
  if (steeringModes_.find(activeMode_) == steeringModes_.end()) {
    return std::string("ERROR: Active mode is corrupted.");
  }
  return steeringModes_.at(activeMode_)->getModuleName();
}

std::vector<unsigned int> SteeringControllerModule::getModes() const {
  std::vector<unsigned int> modeIds(steeringModes_.size());
  for (unsigned int i = 0; i < modeIds.size(); ++i) {
    modeIds[i] = std::next(steeringModes_.begin(), i)->first;
  }
  return modeIds;
}

std::vector<std::string> SteeringControllerModule::getModeNames() const {
  std::vector<std::string> modeNames(steeringModes_.size());
  for (unsigned int i = 0; i < modeNames.size(); ++i) {
    modeNames[i] = std::next(steeringModes_.begin(), i)->second->getModuleName();
  }
  return modeNames;
}

void SteeringControllerModule::setActiveModeSteeringAngle(const double steeringAngle) {
  if (steeringModes_.find(activeMode_) == steeringModes_.end()) {
    MELO_ERROR("[SteeringControllerModule] ERROR: Active mode is corrupted.");
    return;
  }
  steeringModes_.at(activeMode_)->setSteeringAngle(steeringAngle);
}

double SteeringControllerModule::getActiveModeSteeringAngle() const {
  if (steeringModes_.find(activeMode_) == steeringModes_.end()) {
    MELO_ERROR("[SteeringControllerModule] ERROR: Active mode is corrupted.");
    return 0.0;
  }
  return steeringModes_.at(activeMode_)->getSteeringAngle();
}

void SteeringControllerModule::addObserver(SteeringControllerObserver* observer) {
  auto it = std::find(observers_.begin(), observers_.end(), observer);
  if (it == observers_.end()) {
    observers_.push_back(observer);
  }
}

void SteeringControllerModule::removeObserver(SteeringControllerObserver* observer) {
  auto it = std::find(observers_.begin(), observers_.end(), observer);
  if (it != observers_.end()) {
    observers_.erase(it);
  }
}

const Position& SteeringControllerModule::getPositionBaseProjectionToCoRInControlFrame() {
  return steeringModes_.at(activeMode_)->getPositionBaseProjectionToCoRInControlFrame();
}

bool SteeringControllerModule::initialize(double dt, loco::WholeBody& wholeBody) {
  needsInitialization_ = true;
  return true;
}

bool SteeringControllerModule::advance(double dt, loco::WholeBody& wholeBody) {
  if (steeringModes_.empty()) {
    MELO_WARN_THROTTLE(1.0, "Could not advance steering controller. No steering modes added!");
    return false;
  }

  if (needsInitialization_) {
    steeringModes_[activeMode_]->initialize(dt, wholeBody);
    needsInitialization_ = false;
  }

  /** Advance the steering mode:
   *  1. Sets commands for steering joint(s)
   *  2. Sets wheel speeds
   */
  return steeringModes_[activeMode_]->advance(dt, wholeBody);
}

bool SteeringControllerModule::loadModuleParameters(const TiXmlHandle& handle) {
  bool allFine = true;
  TiXmlHandle steeringControllerHandle(handle);
  if (tinyxml_tools::getChildHandle(steeringControllerHandle, handle, "SteeringController")) {
    for (auto& steeringMode : steeringModes_) {
      allFine = allFine && steeringMode.second->loadModuleParameters(steeringControllerHandle);
    }
  }
  return allFine;
}

bool SteeringControllerModule::addModuleVariablesToLog(const std::string& ns) const {
  bool allFine = true;
  for (auto& steeringMode : steeringModes_) {
    allFine &= steeringMode.second->addModuleVariablesToLog(ns);
  }
  return allFine;
}

bool SteeringControllerModule::addModuleParametersToHandler(const std::string& ns) {
  bool allFine = true;
  for (auto& steeringMode : steeringModes_) {
    allFine &= steeringMode.second->addModuleParametersToHandler(ns);
  }
  return allFine;
}

} /* namespace loco */
