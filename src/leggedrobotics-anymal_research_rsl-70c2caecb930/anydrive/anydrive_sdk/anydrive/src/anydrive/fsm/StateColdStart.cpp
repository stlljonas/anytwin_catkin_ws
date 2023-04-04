#include "anydrive/fsm/StateColdStart.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateColdStart::StateColdStart(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::ColdStart) {}

}  // namespace fsm
}  // namespace anydrive
