/*
 * CommandTranslator.hpp
 *
 *  Created on: Jan 4, 2017
 *      Author: Christian Gehring, Peter Fankhauser, Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/legs/Legs.hpp>
#include <loco/common/typedefs.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>

// anymal_roco
#include <anymal_roco/RocoCommand.hpp>

// anymal Moco
#include <anymal_motion_control/Command.hpp>

// loco_sea
#include <loco_sea/command_modes.hpp>

class TiXmlHandle;

namespace loco_anymal {

class CommandTranslator {
 public:
  using AD = anymal_description::AnymalDescription;

  CommandTranslator() = default;
  virtual ~CommandTranslator() = default;

  series_elastic_actuator::SeActuatorCommand::PidGains& getPidGains(const AD::ActuatorNodeEnum joint);

  // roco
  bool setPidGains(anymal_roco::RocoCommand& command);
  bool setPidGains(AD::LimbEnum limb, anymal_roco::RocoCommand& command);
  void setCommands(anymal_roco::RocoCommand& command, const loco::Legs& legs);

  // moco
  bool setPidGains(anymal_motion_control::Command& command);
  bool setPidGains(AD::LimbEnum limb, anymal_motion_control::Command& command);
  void setCommands(anymal_motion_control::Command& command, const loco::Legs& legs);

  bool loadParameters(const TiXmlHandle& handle, const std::string& parameterName = "Commands");

 protected:
  series_elastic_actuator::SeActuatorCommand::PidGains gainsHaa_;
  series_elastic_actuator::SeActuatorCommand::PidGains gainsHfe_;
  series_elastic_actuator::SeActuatorCommand::PidGains gainsKfe_;
};

} // namespace
