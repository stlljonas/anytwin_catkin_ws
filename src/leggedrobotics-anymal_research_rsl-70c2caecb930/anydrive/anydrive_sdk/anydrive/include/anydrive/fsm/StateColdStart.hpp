#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateColdStart : public StateBase {
 public:
  StateColdStart(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateColdStart() override = default;
};

}  // namespace fsm
}  // namespace anydrive
