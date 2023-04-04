#include "anydrive/fsm/StateMotorPreOp.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateMotorPreOp::StateMotorPreOp(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::MotorPreOp) {}

}  // namespace fsm
}  // namespace anydrive
