#pragma once

#include "anydrive/Reading.hpp"

namespace anydrive {

//! Extended reading of the ANYdrive containing the command and the extended state.
using ReadingExtended = ReadingT<Command, StateExtended>;

}  // namespace anydrive
