#include "anydrive/fsm/StateMachine.hpp"
#include "anydrive/Anydrive.hpp"
#include "anydrive/common/Macros.hpp"
#include "anydrive/fsm/StateCalibrate.hpp"
#include "anydrive/fsm/StateColdStart.hpp"
#include "anydrive/fsm/StateConfigure.hpp"
#include "anydrive/fsm/StateControlOp.hpp"
#include "anydrive/fsm/StateDeviceMissing.hpp"
#include "anydrive/fsm/StateError.hpp"
#include "anydrive/fsm/StateFatal.hpp"
#include "anydrive/fsm/StateMotorOp.hpp"
#include "anydrive/fsm/StateMotorPreOp.hpp"
#include "anydrive/fsm/StateStandby.hpp"
#include "anydrive/fsm/StateWarmStart.hpp"

namespace anydrive {
namespace fsm {

StateMachine::StateMachine(Anydrive& anydrive) : anydrive_(anydrive) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  activeStateEnum_ = StateEnum::NA;
  goalStateEnum_ = StateEnum::NA;
  addState(StateBasePtr(new StateCalibrate(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateColdStart(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateConfigure(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateControlOp(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateDeviceMissing(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateError(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateFatal(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateMotorOp(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateMotorPreOp(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateStandby(anydrive_, goalStateEnum_)));
  addState(StateBasePtr(new StateWarmStart(anydrive_, goalStateEnum_)));
}

void StateMachine::updateActiveState(const StateEnum newActiveStateEnum) {
  // Check if the active state changed.
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto activeStateIt = states_.find(activeStateEnum_);
  StateBasePtr activeState = (activeStateIt == states_.end()) ? StateBasePtr() : activeStateIt->second;
  if (newActiveStateEnum != activeStateEnum_) {
    auto newActiveStateIt = states_.find(newActiveStateEnum);
    if (newActiveStateIt == states_.end()) {
      ANYDRIVE_NAMED_WARN("New active FSM state '" << stateEnumToName(newActiveStateEnum) << "' has not been found in the list of states.");
    } else {
      if (activeState) {
        activeState->leave();
      }
      activeState = newActiveStateIt->second;
      activeStateEnum_ = newActiveStateEnum;
      activeState->enter();
    }
  }

  // Update the active state.
  if (activeState) {
    activeState->update();
  }
}

StateEnum StateMachine::getActiveStateEnum() const {
  return activeStateEnum_;
}

StateEnum StateMachine::getGoalStateEnum() const {
  return goalStateEnum_;
}

bool StateMachine::goalStateHasBeenReached() const {
  return goalStateEnum_ == StateEnum::NA;
}

void StateMachine::setGoalStateEnum(const StateEnum goalStateEnum) {
  if (goalStateEnum == StateEnum::NA) {
    return;
  }
  if (goalStateEnum == StateEnum::DeviceMissing) {
    ANYDRIVE_NAMED_WARN("Cannot set goal FSM state to DeviceMissing.");
    return;
  }
  if (goalStateEnum == StateEnum::Error || goalStateEnum == StateEnum::Fatal) {
    ANYDRIVE_NAMED_WARN("Cannot set goal FSM state to Error or Fatal.");
    return;
  }
  if (goalStateEnum == StateEnum::ColdStart || goalStateEnum == StateEnum::WarmStart || goalStateEnum == StateEnum::MotorPreOp) {
    ANYDRIVE_NAMED_WARN("Cannot set goal FSM state to ColdStart, WarmStart or MotorPreOp (auto-transition states).");
    return;
  }
  if (goalStateEnum == getActiveStateEnum()) {
    ANYDRIVE_NAMED_DEBUG("Device is already in goal state.");
    return;
  }
  ANYDRIVE_NAMED_DEBUG("Setting goal FSM state to '" << stateEnumToName(goalStateEnum) << "'.");
  goalStateEnum_ = goalStateEnum;
}

void StateMachine::clearGoalStateEnum() {
  goalStateEnum_ = StateEnum::NA;
  ANYDRIVE_NAMED_DEBUG("The goal FSM state has been cleared.");
}

std::string StateMachine::getName() const {
  return anydrive_.getName();
}

void StateMachine::addState(const StateBasePtr& state) {
  states_.insert({state->getStateEnum(), state});
}

}  // namespace fsm
}  // namespace anydrive
