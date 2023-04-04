#pragma once


// conditional state machine
#include <conditional_state_machine/Substates.hpp>


namespace anymal_lowlevel_controller {
namespace state_machine {


enum class AnydrivesStateEnum
{
  Error, // At least one device is in error state.
  Fatal, // At least one device is in fatal state.
  Missing, // At least one device is missing.
  NA, // Not set, invalid.
  Operational, // All devices are in operational state.
  Other // All other combinations.
};


using Substates = conditional_state_machine::Substates<AnydrivesStateEnum>;


} // state_machine
} // anymal_lowlevel_controller
