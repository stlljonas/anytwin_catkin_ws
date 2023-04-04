#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateFatal : public StateBase {
 public:
  StateFatal(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateFatal() override = default;

 protected:
  void enterDerived() override;
  void leaveDerived() override;
};

}  // namespace fsm
}  // namespace anydrive
