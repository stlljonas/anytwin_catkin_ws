#include "anydrive/fsm/StateFatal.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateFatal::StateFatal(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum) : StateBase(anydrive, goalStateEnum, StateEnum::Fatal) {}

void StateFatal::enterDerived() {
  anydrive_.fatalCb();
}

void StateFatal::leaveDerived() {
  anydrive_.fatalRecoveredCb();
}

}  // namespace fsm
}  // namespace anydrive
