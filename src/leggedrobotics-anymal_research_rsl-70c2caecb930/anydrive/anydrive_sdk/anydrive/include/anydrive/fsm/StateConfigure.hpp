#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateConfigure : public StateBase {
 protected:
  unsigned int step_ = 0;
  unsigned int stepMode_ = 0;
  // Used to retry sending the error state behavior if it was unsuccessful.
  const unsigned int stepErrorStateBehaviorThreshold_ = 20;
  unsigned int stepErrorStateBehavior_ = 0;

 public:
  StateConfigure(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateConfigure() override = default;

 protected:
  void enterDerived() override;
  void updateDerived() override;
};

}  // namespace fsm
}  // namespace anydrive
