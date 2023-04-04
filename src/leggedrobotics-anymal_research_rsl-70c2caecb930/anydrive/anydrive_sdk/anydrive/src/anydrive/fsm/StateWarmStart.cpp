#include "anydrive/fsm/StateWarmStart.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateWarmStart::StateWarmStart(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::WarmStart) {}

}  // namespace fsm
}  // namespace anydrive
