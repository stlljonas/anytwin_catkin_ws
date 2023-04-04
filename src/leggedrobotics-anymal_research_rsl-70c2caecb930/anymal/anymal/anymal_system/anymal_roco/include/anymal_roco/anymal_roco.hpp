#pragma once

// anymal roco
#include <anymal_roco/RocoCommand.hpp>
#include <anymal_roco/RocoState.hpp>

// anymal model
#include <anymal_model/LimitsAnymal.hpp>

namespace anymal_roco {

void initializeState(anymal_roco::RocoState& state);

void initializeCommand(anymal_roco::RocoCommand& command, const anymal_model::LimitsAnymal& limits, const anymal_model::LegConfigurations& legConfigAnymal);

} /* anymal_roco */
