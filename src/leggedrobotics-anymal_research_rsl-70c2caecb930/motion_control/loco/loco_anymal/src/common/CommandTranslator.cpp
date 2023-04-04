/*
 * CommandTranslator.hpp
 *
 *  Created on: Jan 4, 2017
 *      Author: Christian Gehring, Peter Fankhauser, Dario Bellicoso
 */

// loco_anymal
#include <loco_anymal/common/CommandTranslator.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco_anymal {

using AD = CommandTranslator::AD;

series_elastic_actuator::SeActuatorCommand::PidGains& CommandTranslator::getPidGains(const AD::ActuatorNodeEnum joint) {
  switch (joint) {
    case AD::ActuatorNodeEnum::HAA:
      return gainsHaa_;
    case AD::ActuatorNodeEnum::HFE:
      return gainsHfe_;
    case AD::ActuatorNodeEnum::KFE:
      return gainsKfe_;
    default:
      throw std::invalid_argument("No parameter found for given joint in getPidGains(...)!");
  }
}

bool CommandTranslator::setPidGains(anymal_motion_control::Command& command) {
  for (auto& com : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::HAA].values()) {
    com->setPidGains(gainsHaa_);
  }
  for (auto& com : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::HFE].values()) {
    com->setPidGains(gainsHfe_);
  }
  for (auto& com : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::KFE].values()) {
    com->setPidGains(gainsKfe_);
  }
  return true;
}

bool CommandTranslator::setPidGains(AD::LimbEnum limb, anymal_motion_control::Command& command) {
  auto& commandForLimb = command.getBranchNodeActuatorCommands()[limb];
  commandForLimb[AD::ActuatorNodeEnum::HAA]->setPidGains(gainsHaa_);
  commandForLimb[AD::ActuatorNodeEnum::HFE]->setPidGains(gainsHfe_);
  commandForLimb[AD::ActuatorNodeEnum::KFE]->setPidGains(gainsKfe_);
  return true;
}

void CommandTranslator::setCommands(anymal_motion_control::Command& command, const loco::Legs& legs) {
  for (auto leg : legs) {
    const auto limb = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt());
    for (int i=0; i<leg->getLimbStateDesired().getJointControlModes().size(); ++i) {
      const auto node = AD::mapKeyIdToKeyEnum<AD::ActuatorNodeEnum>(i);
      auto& nodeCommand = command.getBranchNodeActuatorCommands()[limb][node];
      nodeCommand->setMode(command_mode::getSeaModeFromLocoMode(leg->getLimbStateDesired().getJointControlModes()(i)));
      nodeCommand->setJointPosition(leg->getLimbStateDesired().getJointPositions()(i));
      nodeCommand->setJointVelocity(leg->getLimbStateDesired().getJointVelocities()(i));
      nodeCommand->setJointTorque(leg->getLimbStateDesired().getJointTorques()(i));
    }
  }
}

bool CommandTranslator::setPidGains(anymal_roco::RocoCommand& command) {
  for (auto& com : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::HAA].values()) {
    com->setPidGains(gainsHaa_);
  }
  for (auto& com : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::HFE].values()) {
    com->setPidGains(gainsHfe_);
  }
  for (auto& com : command.getNodeBranchActuatorCommands()[AD::ActuatorNodeEnum::KFE].values()) {
    com->setPidGains(gainsKfe_);
  }
  return true;
}

bool CommandTranslator::setPidGains(AD::LimbEnum limb, anymal_roco::RocoCommand& command) {
  auto& commandForLimb = command.getBranchNodeActuatorCommands()[limb];
  commandForLimb[AD::ActuatorNodeEnum::HAA]->setPidGains(gainsHaa_);
  commandForLimb[AD::ActuatorNodeEnum::HFE]->setPidGains(gainsHfe_);
  commandForLimb[AD::ActuatorNodeEnum::KFE]->setPidGains(gainsKfe_);
  return true;
}

void CommandTranslator::setCommands(anymal_roco::RocoCommand& command, const loco::Legs& legs) {
  for (auto leg : legs) {
    const auto limb = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt());
    for (int i=0; i<leg->getLimbStateDesired().getJointControlModes().size(); ++i) {
      const auto node = AD::mapKeyIdToKeyEnum<AD::ActuatorNodeEnum>(i);
      auto& nodeCommand = command.getBranchNodeActuatorCommands()[limb][node];
      nodeCommand->setMode(command_mode::getSeaModeFromLocoMode(leg->getLimbStateDesired().getJointControlModes()(i)));
      nodeCommand->setJointPosition(leg->getLimbStateDesired().getJointPositions()(i));
      nodeCommand->setJointVelocity(leg->getLimbStateDesired().getJointVelocities()(i));
      nodeCommand->setJointTorque(leg->getLimbStateDesired().getJointTorques()(i));
    }
  }
}

bool CommandTranslator::loadParameters(const TiXmlHandle& handle, const std::string& parameterName) {
  TiXmlHandle commandsHandle = handle;
  if (!tinyxml_tools::getChildHandle(commandsHandle, handle, parameterName)) { return false; }

  TiXmlHandle gainsHaaHandle = handle;
  if (!tinyxml_tools::getChildHandle(gainsHaaHandle, commandsHandle, "HAA")) { return false; }

  TiXmlHandle gainsHfeHandle = handle;
  if (!tinyxml_tools::getChildHandle(gainsHfeHandle, commandsHandle, "HFE")) { return false; }

  TiXmlHandle gainsKfeHandle = handle;
  if (!tinyxml_tools::getChildHandle(gainsKfeHandle, commandsHandle, "KFE")) { return false; }

  if (!tinyxml_tools::loadParameter(gainsHaa_.pGain_, gainsHaaHandle, "pGain", 0.0)) { return false; }
  if (!tinyxml_tools::loadParameter(gainsHaa_.iGain_, gainsHaaHandle, "iGain", 0.0)) { return false; }
  if (!tinyxml_tools::loadParameter(gainsHaa_.dGain_, gainsHaaHandle, "dGain", 0.0)) { return false; }

  if (!tinyxml_tools::loadParameter(gainsHfe_.pGain_, gainsHfeHandle, "pGain", 0.0)) { return false; }
  if (!tinyxml_tools::loadParameter(gainsHfe_.iGain_, gainsHfeHandle, "iGain", 0.0)) { return false; }
  if (!tinyxml_tools::loadParameter(gainsHfe_.dGain_, gainsHfeHandle, "dGain", 0.0)) { return false; }

  if (!tinyxml_tools::loadParameter(gainsKfe_.pGain_, gainsKfeHandle, "pGain", 0.0)) { return false; }
  if (!tinyxml_tools::loadParameter(gainsKfe_.iGain_, gainsKfeHandle, "iGain", 0.0)) { return false; }
  if (!tinyxml_tools::loadParameter(gainsKfe_.dGain_, gainsKfeHandle, "dGain", 0.0)) { return false; }

  return true;
}

} // namespace
