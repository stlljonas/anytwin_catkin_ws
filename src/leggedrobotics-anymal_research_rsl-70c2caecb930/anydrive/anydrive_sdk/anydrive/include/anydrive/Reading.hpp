#pragma once

#include "anydrive/Command.hpp"
#include "anydrive/State.hpp"
#include "anydrive/StateExtended.hpp"

namespace anydrive {

//! Reading of the ANYdrive containing arbitrary command and state information.
template <typename CommandT, typename StateT>
class ReadingT {
 public:
  using Command = CommandT;
  using State = StateT;

 protected:
  //! Command which was sent to the ANYdrive.
  Command commanded_;
  //! Current state of the ANYdrive.
  State state_;

 public:
  ReadingT() = default;
  virtual ~ReadingT() = default;

  const Command& getCommanded() const { return commanded_; }

  Command& getCommanded() { return commanded_; }

  void setCommanded(const Command& commanded) { commanded_ = commanded; }

  const State& getState() const { return state_; }

  State& getState() { return state_; }

  void setState(const State& state) { state_ = state; }

  std::string asString(const std::string& prefix = "") const {
    std::stringstream ss;
    ss << prefix << commanded_.asString("  ") << std::endl;
    ss << prefix << state_.asString("  ") << std::endl;
    return ss.str();
  }
};

template <typename CommandT, typename StateT>
std::ostream& operator<<(std::ostream& out, const ReadingT<CommandT, StateT>& reading) {
  out << "Reading:" << std::endl;
  out << reading.asString("  ") << std::endl;
  return out;
}

//! Reading of the ANYdrive containing the command and the state.
using Reading = ReadingT<Command, State>;

}  // namespace anydrive
