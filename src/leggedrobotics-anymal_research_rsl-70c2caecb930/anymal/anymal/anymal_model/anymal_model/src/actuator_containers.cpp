// anymal model
#include "anymal_model/actuator_containers.hpp"

namespace anymal_model {

void initializeActuatorCommandsFromLimits(ActuatorCommandRobotContainer& actuatorCommands, const LimitsAnymal& limits,
                                          const LegConfigurations& /*legConfigAnymal*/) {
  for (const auto& key : AD::getActuatorKeys()) {
    auto& actuator = actuatorCommands[key.getEnum()];
    actuator.setJointPositionMin(limits.getActuatorMinPosition(key.getEnum()));
    actuator.setJointPositionMax(limits.getActuatorMaxPosition(key.getEnum()));
    actuator.setJointVelocityMin(limits.getActuatorMinVelocity(key.getEnum()));
    actuator.setJointVelocityMax(limits.getActuatorMaxVelocity(key.getEnum()));
    actuator.setJointTorqueMin(limits.getActuatorMinCommandEffort(key.getEnum()));
    actuator.setJointTorqueMax(limits.getActuatorMaxCommandEffort(key.getEnum()));
    actuator.setGearVelocityMin(limits.getActuatorMinGearVelocity(key.getEnum()));
    actuator.setGearVelocityMax(limits.getActuatorMaxGearVelocity(key.getEnum()));
    actuator.setCurrentMin(limits.getActuatorMinCurrent(key.getEnum()));
    actuator.setCurrentMax(limits.getActuatorMaxCurrent(key.getEnum()));
  }
}

}  // namespace anymal_model
