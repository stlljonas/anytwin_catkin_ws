/*
 * SteeringModeBase.hpp
 *
 *  Created on: Aug 20, 2015
 *      Author: Gabriel Hottiger, Philipp Leemann
 */

#pragma once

// parameter handler
#include "parameter_handler/parameter_handler.hpp"

// loco
#include "loco/driving_control/SteeringModeModule.hpp"

#include "loco/common/ModuleBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/common/typedefs.hpp"

// Tinyxml
#include <tinyxml.h>

// STL
#include <memory>

namespace loco {

class SteeringModeBase : public ModuleBase, public SteeringModeModule {
 public:
  /** Constructor
   * @param name  name of the steering mode
   * @param wholebody container for mainbody and limbs
   * @return object of type SteeringModeBase
   */
  SteeringModeBase(const std::string& name, loco::WholeBody& wholeBody)
      : ModuleBase(name), SteeringModeModule(name), wholeBody_(wholeBody) {}

  //! Default destructor
  virtual ~SteeringModeBase() {}

  virtual bool initialize(double dt) override { return SteeringModeModule::initialize(dt, wholeBody_); }

  virtual bool advance(double dt) override { return SteeringModeModule::advance(dt, wholeBody_); }

  virtual bool loadParameters(const TiXmlHandle& handle) override { return SteeringModeModule::loadModuleParameters(handle); }

  virtual bool addParametersToHandler(const std::string& ns = "") override { return SteeringModeModule::addModuleParametersToHandler(ns); }

  virtual bool addVariablesToLog(const std::string& ns) const override { return SteeringModeModule::addModuleVariablesToLog(ns); }

 protected:
  //! Robot components
  loco::WholeBody& wholeBody_;
};

using SteeringModeBasePtr = std::unique_ptr<SteeringModeBase>;

} /* namespace loco */
