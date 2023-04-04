#pragma once

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateCalibrate : public StateBase {
 public:
  StateCalibrate(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum);
  ~StateCalibrate() override = default;
};

}  // namespace fsm
}  // namespace anydrive
