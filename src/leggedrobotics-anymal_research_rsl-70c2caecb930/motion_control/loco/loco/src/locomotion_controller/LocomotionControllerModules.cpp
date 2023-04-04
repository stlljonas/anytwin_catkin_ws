/*!
 * @file     LocomotionControllerModules.cpp
 * @author   Gabriel Hottiger
 * @date     Jan, 2016
 */

// loco
#include "loco/locomotion_controller/LocomotionControllerModules.hpp"

namespace loco {

LocomotionControllerModules::LocomotionControllerModules() : LocomotionControllerBase() {}

bool LocomotionControllerModules::initialize(double dt) {
  for (auto module : measurementModules_) {
    if (!module.second->initialize(dt)) {
      MELO_ERROR("Failed to initialize measurement module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  for (auto module : setpointModules_) {
    if (!module.second->initialize(dt)) {
      MELO_ERROR("Failed to initialize setpoint module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  return true;
}

bool LocomotionControllerModules::loadParameters(const TiXmlHandle& handle) {
  for (auto module : measurementModules_) {
    if (!module.second->loadParameters(handle)) {
      MELO_ERROR("Failed to load parameters for measurement module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  for (auto module : setpointModules_) {
    if (!module.second->loadParameters(handle)) {
      MELO_ERROR("Failed to load parameters for setpoint module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  return true;
}

bool LocomotionControllerModules::advanceMeasurements(double dt) {
  for (auto module : measurementModules_) {
    if (!module.second->advance(dt)) {
      MELO_ERROR("Failed to advance measurement module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  return true;
}

bool LocomotionControllerModules::advanceSetPoints(double dt) {
  for (auto module : setpointModules_) {
    if (!module.second->advance(dt)) {
      MELO_ERROR("Failed to advance setpoint module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  return true;
}

bool LocomotionControllerModules::addVariablesToLog(const std::string& ns) const {
  for (auto module : measurementModules_) {
    if (!module.second->addVariablesToLog(ns)) {
      MELO_ERROR("Failed to add variables to log for measurement module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  for (auto module : setpointModules_) {
    if (!module.second->addVariablesToLog(ns)) {
      MELO_ERROR("Failed to add variables to log for setpoint module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  return true;
}

bool LocomotionControllerModules::addParametersToHandler(const std::string& ns) {
  for (auto module : measurementModules_) {
    if (!module.second->addParametersToHandler(ns)) {
      MELO_ERROR("Failed to add parameters to handler for measurement module %s at index %d", module.second->getName().c_str(),
                 module.first);
      return false;
    }
  }
  for (auto module : setpointModules_) {
    if (!module.second->addParametersToHandler(ns)) {
      MELO_ERROR("Failed to add parameters to handler for setpoint module %s at index %d", module.second->getName().c_str(), module.first);
      return false;
    }
  }
  return true;
}

bool LocomotionControllerModules::removeParametersFromHandler() {
  bool success = true;
  for (auto module : measurementModules_) {
    if (!module.second->removeParametersFromHandler()) {
      MELO_ERROR("Failed to remove parameters to handler for measurement module %s at index %d", module.second->getName().c_str(),
                 module.first);
      success = false;
    }
  }
  for (auto module : setpointModules_) {
    if (!module.second->removeParametersFromHandler()) {
      MELO_ERROR("Failed to remove parameters to handler for setpoint module %s at index %d", module.second->getName().c_str(),
                 module.first);
      success = false;
    }
  }
  return success;
}

void LocomotionControllerModules::print(std::ostream& out) const {
  LocomotionControllerBase::print(out);
  out << "Measurements:" << std::endl;
  out << "=============" << std::endl;
  for (auto module : measurementModules_) {
    out << "---> " << module.second->getName() << " <---" << std::endl;
    out << module.second << std::endl;
  }
  out << "Set Points:" << std::endl;
  out << "===========" << std::endl;
  for (auto module : setpointModules_) {
    out << "---> " << module.second->getName() << " <---" << std::endl;
    out << module.second << std::endl;
  }
}

bool LocomotionControllerModules::stop() {
  bool success = true;
  for (auto module : measurementModules_) {
    success &= module.second->stop();
  }
  for (auto module : setpointModules_) {
    success &= module.second->stop();
  }

  return success;
}

void LocomotionControllerModules::addMeasurementModule(const unsigned int index, loco::ModuleBase* module) {
  measurementModules_.insert(std::make_pair(index, module));
}

void LocomotionControllerModules::pushBackMeasurementModule(loco::ModuleBase* module) {
  const unsigned int index = (!measurementModules_.empty()) ? (measurementModules_.rbegin()->first + 1) : 0u;
  addMeasurementModule(index, module);
}

void LocomotionControllerModules::pushBackMeasurementModules(const std::vector<loco::ModuleBase*>& modules) {
  for (auto module : modules) {
    pushBackMeasurementModule(module);
  }
}

void LocomotionControllerModules::removeMeasurementModule(const unsigned int index) {
  if (measurementModules_.erase(index) == 0) {
    MELO_WARN("There is no measurement module in the locomotion controller with index %d", index);
  }
}

void LocomotionControllerModules::addSetPointModule(const unsigned int index, loco::ModuleBase* module) {
  setpointModules_.insert(std::make_pair(index, module));
}

void LocomotionControllerModules::pushBackSetPointModule(loco::ModuleBase* module) {
  const unsigned int index = (!setpointModules_.empty()) ? (setpointModules_.rbegin()->first + 1) : 0u;
  addSetPointModule(index, module);
}

void LocomotionControllerModules::pushBackSetPointModules(const std::vector<loco::ModuleBase*>& modules) {
  for (auto module : modules) {
    pushBackSetPointModule(module);
  }
}

void LocomotionControllerModules::removeSetPointModule(const unsigned int index) {
  if (setpointModules_.erase(index) == 0) {
    MELO_WARN("There is no setpoint module in the locomotion controller with index %d", index);
  }
}

} /* namespace loco */
