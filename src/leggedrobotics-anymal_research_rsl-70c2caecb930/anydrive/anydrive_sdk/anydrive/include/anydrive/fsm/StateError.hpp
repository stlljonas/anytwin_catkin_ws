#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateError : public StateBase {
 public:
  StateError(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateError() override = default;

 protected:
  void enterDerived() override;
  void leaveDerived() override;
};

}  // namespace fsm
}  // namespace anydrive
