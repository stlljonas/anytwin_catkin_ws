/*
 * SteeringModeModule.cpp
 *
 *  Created on: Aug 20, 2017
 *      Author: Gabriel Hottiger
 */

// loco
#include "loco/driving_control/SteeringModeModule.hpp"

#include <utility>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace loco {

SteeringModeModule::SteeringModeModule(std::string name)
    : name_(std::move(name)),
      positionBaseProjectionToCoRInControlFrame_(Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity())),
      steeringAngle_(0.0),
      minSteeringAngle_(-defaultSteeringAngleLimit_),
      maxSteeringAngle_(defaultSteeringAngleLimit_),
      steeringIncrement_(1.0, -10.0, 10.0),
      velocityFactor_(1.0, -10.0, 10.0) {}

bool SteeringModeModule::loadModuleParameters(const TiXmlHandle& handle) {
  if (handle.Element() != nullptr) {
    tinyxml_tools::loadParameter(steeringIncrement_, handle, "steeringIncrement", parameter_handler::Parameter<double>(0.0));
    tinyxml_tools::loadParameter(velocityFactor_, handle, "velocityFactor", parameter_handler::Parameter<double>(0.0));
    tinyxml_tools::loadParameter(minSteeringAngle_, handle, "minSteeringAngle", -defaultSteeringAngleLimit_);
    tinyxml_tools::loadParameter(maxSteeringAngle_, handle, "maxSteeringAngle", defaultSteeringAngleLimit_);
  } else {
    MELO_WARN("Could not read TiXmlHandle SteeringController::%s", this->getModuleName().c_str());
  }
  return true;
}

bool SteeringModeModule::addModuleParametersToHandler(const std::string& ns) {
  parameter_handler::handler->addParam(ns + getModuleName() + std::string{"/steeringIncrement"}, steeringIncrement_);
  parameter_handler::handler->addParam(ns + getModuleName() + std::string{"/velocityFactor"}, velocityFactor_);
  return true;
}

bool SteeringModeModule::addModuleVariablesToLog(const std::string& ns) const {}

} /* namespace loco */
