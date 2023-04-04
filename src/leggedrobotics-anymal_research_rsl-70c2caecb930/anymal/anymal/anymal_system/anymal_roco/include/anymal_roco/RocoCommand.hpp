/*!
 * @file     Command.hpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Dec, 2014
 */

#pragma once

// anymal model
#include <anymal_model/actuator_containers.hpp>
#include <anymal_model/contact_force_calibrator_containers.hpp>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// robot utils
#include <robot_utils/force_calibrators/ForceCalibratorCommand.hpp>

// roco
#include <roco/model/CommandInterface.hpp>

// stl
#include <ostream>


namespace anymal_roco {

class RocoCommand: public roco::CommandInterface
{
 public:
    using Mode = anymal_model::ActuatorCommandRobotContainer::Item::Mode;

private:
    static constexpr auto numLegs = anymal_description::AnymalDescription::getNumLegs();

 public:
  RocoCommand();
  virtual ~RocoCommand() = default;

  /*! Limits the command
   * @returns false if there is a NaN or Inf.
   */
  bool limitCommand();

  const anymal_model::ActuatorCommandRobotContainer& getActuatorCommands() const;
  anymal_model::ActuatorCommandRobotContainer& getActuatorCommands();
  void setActuatorCommands(const anymal_model::ActuatorCommandRobotContainer& commands);

  const anymal_model::ActuatorCommandPtrBranchNodeContainer& getBranchNodeActuatorCommands() const;
  anymal_model::ActuatorCommandPtrBranchNodeContainer& getBranchNodeActuatorCommands();
  const anymal_model::ActuatorCommandPtrNodeBranchContainer& getNodeBranchActuatorCommands() const;
  anymal_model::ActuatorCommandPtrNodeBranchContainer& getNodeBranchActuatorCommands();

  anymal_model::ContactForceCalibratorCommandContainer& getForceCalibratorCommands();
  const anymal_model::ContactForceCalibratorCommandContainer& getForceCalibratorCommands() const;
  void addVariablesToLog(bool updateLogger);

  friend std::ostream& operator <<(std::ostream& out, const RocoCommand& command);
 protected:
    anymal_model::ActuatorCommandRobotContainer actuatorCommands_;
    anymal_model::ActuatorCommandPtrBranchNodeContainer branchNodeActuatorCommands_;
    anymal_model::ActuatorCommandPtrNodeBranchContainer nodeBranchActuatorCommands_;
    anymal_model::ContactForceCalibratorCommandContainer forceCalibratorCommands_;

};

}
