#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateStandby : public StateBase {
 public:
  StateStandby(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateStandby() override = default;
};

}  // namespace fsm
}  // namespace anydrive
