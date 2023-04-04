#include "anydrive/fsm/StateError.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateError::StateError(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum) : StateBase(anydrive, goalStateEnum, StateEnum::Error) {}

void StateError::enterDerived() {
  anydrive_.errorCb();
}

void StateError::leaveDerived() {
  anydrive_.errorRecoveredCb();
}

}  // namespace fsm
}  // namespace anydrive
