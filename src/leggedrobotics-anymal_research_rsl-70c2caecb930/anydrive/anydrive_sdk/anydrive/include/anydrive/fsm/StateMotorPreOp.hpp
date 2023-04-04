#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateMotorPreOp : public StateBase {
 public:
  StateMotorPreOp(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateMotorPreOp() override = default;
};

}  // namespace fsm
}  // namespace anydrive
