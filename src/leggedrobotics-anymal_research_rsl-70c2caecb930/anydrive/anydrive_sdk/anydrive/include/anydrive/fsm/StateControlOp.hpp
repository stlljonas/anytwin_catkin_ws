#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateControlOp : public StateBase {
 public:
  StateControlOp(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateControlOp() override = default;

 protected:
  void enterDerived() override;
  void leaveDerived() override;
};

}  // namespace fsm
}  // namespace anydrive
