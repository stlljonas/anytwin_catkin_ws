#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateMotorOp : public StateBase {
 public:
  StateMotorOp(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateMotorOp() override = default;
};

}  // namespace fsm
}  // namespace anydrive
