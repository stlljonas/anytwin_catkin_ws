/*
 * SteeringController.hpp
 *
 *  Created on: Oct 1, 2015
 *      Author: Philipp Leemann, Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/driving_control/SteeringControllerObserver.hpp"
#include "loco/driving_control/SteeringModeModule.hpp"

// STL
#include <map>
#include <memory>
#include <string>

class TiXmlHandle;

namespace loco {

//! Wrapper class for different steering modes.
class SteeringControllerModule {
 public:
  //! Default Constructor
  SteeringControllerModule();
  //! Default Destructor
  virtual ~SteeringControllerModule() = default;

  //! -- Module Base Implementation
  virtual bool initialize(double dt, loco::WholeBody& wholeBody);
  virtual bool advance(double dt, loco::WholeBody& wholeBody);
  virtual bool loadModuleParameters(const TiXmlHandle& handle);
  virtual bool addModuleVariablesToLog(const std::string& ns) const;
  virtual bool addModuleParametersToHandler(const std::string& ns);

  bool addSteeringMode(const unsigned int imode, SteeringModeModulePtr&& mode);

  bool setActiveMode(const std::string& modeName, double dt);
  bool setActiveMode(const unsigned int imode, double dt);
  bool nextMode(double dt);

  unsigned int getActiveMode() const;
  std::string getActiveModeName() const;

  std::vector<unsigned int> getModes() const;
  std::vector<std::string> getModeNames() const;

  void setActiveModeSteeringAngle(const double steeringAngle);
  double getActiveModeSteeringAngle() const;

  void addObserver(SteeringControllerObserver* observer);
  void removeObserver(SteeringControllerObserver* observer);

  const Position& getPositionBaseProjectionToCoRInControlFrame();

 protected:
  std::map<unsigned int, SteeringModeModulePtr> steeringModes_;
  unsigned int activeMode_;
  std::vector<SteeringControllerObserver*> observers_;
  bool needsInitialization_;
};

} /* namespace loco */
