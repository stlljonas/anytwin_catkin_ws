/*
 * SteeringController.hpp
 *
 *  Created on: Oct 1, 2015
 *      Author: Philipp Leemann, Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/driving_control/SteeringControllerModule.hpp"

// tinyxml
#include <tinyxml.h>

// STL
#include <string>

namespace loco {

//! Wrapper class for different steering modes.
class SteeringController : public ModuleBase, public SteeringControllerModule {
 public:
  //! Default Constructor
  SteeringController(loco::WholeBody& wholeBody) : ModuleBase("SteeringController"), SteeringControllerModule(), wholeBody_(wholeBody) {}

  //! Default Destructor
  virtual ~SteeringController() {}

  //! -- Module Base Implementation
  virtual bool initialize(double dt) override { return SteeringControllerModule::initialize(dt, wholeBody_); }

  virtual bool advance(double dt) override { return SteeringControllerModule::advance(dt, wholeBody_); }

  virtual bool loadParameters(const TiXmlHandle& handle) override { return SteeringControllerModule::loadModuleParameters(handle); }

  virtual bool addVariablesToLog(const std::string& ns) const override { return SteeringControllerModule::addModuleVariablesToLog(ns); }

  virtual bool addParametersToHandler(const std::string& ns) override { return SteeringControllerModule::addModuleParametersToHandler(ns); }

 protected:
  loco::WholeBody& wholeBody_;
};

} /* namespace loco */
