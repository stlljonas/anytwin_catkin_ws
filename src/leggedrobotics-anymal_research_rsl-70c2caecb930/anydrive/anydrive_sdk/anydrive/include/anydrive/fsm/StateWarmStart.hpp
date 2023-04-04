#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateWarmStart : public StateBase {
 public:
  StateWarmStart(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateWarmStart() override = default;
};

}  // namespace fsm
}  // namespace anydrive
