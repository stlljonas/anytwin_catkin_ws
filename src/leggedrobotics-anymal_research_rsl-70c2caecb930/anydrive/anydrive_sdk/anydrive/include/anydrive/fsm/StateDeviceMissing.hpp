#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateDeviceMissing : public StateBase {
 public:
  StateDeviceMissing(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateDeviceMissing() override = default;

 protected:
  void enterDerived() override;
  void leaveDerived() override;
};

}  // namespace fsm
}  // namespace anydrive
