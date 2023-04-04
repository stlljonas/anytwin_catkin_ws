#include "anydrive/fsm/StateDeviceMissing.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateDeviceMissing::StateDeviceMissing(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::DeviceMissing) {}

void StateDeviceMissing::enterDerived() {
  ANYDRIVE_NAMED_WARN("The device is disconnected.");
  anydrive_.deviceDisconnectedCb();
}

void StateDeviceMissing::leaveDerived() {
  ANYDRIVE_NAMED_INFO("The device is reconnected.");
  anydrive_.deviceReconnectedCb();
}

}  // namespace fsm
}  // namespace anydrive
