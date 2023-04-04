// anymal roco
#include <anymal_roco/anymal_roco.hpp>

// anymal model
#include <anymal_model/actuator_containers.hpp>

namespace anymal_roco {

void initializeState(anymal_roco::RocoState& state) {
  state.setStatus(anymal_roco::RocoState::StateStatus::STATUS_ERROR_UNKNOWN);
}

void initializeCommand(anymal_roco::RocoCommand& command, const anymal_model::LimitsAnymal& limits, const anymal_model::LegConfigurations& legConfigAnymal) {
  anymal_model::initializeActuatorCommandsFromLimits(command.getActuatorCommands(), limits, legConfigAnymal);
}

} /* anymal_roco */
