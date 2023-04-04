#include "anydrive/fsm/StateBase.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateBase::StateBase(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum, const StateEnum stateEnum,
                     std::map<StateEnum, uint8_t> goalStateEnumToControlword)
    : anydrive_(anydrive),
      goalStateEnum_(goalStateEnum),
      stateEnum_(stateEnum),
      name_(stateEnumToName(stateEnum)),
      goalStateEnumToControlword_(std::move(goalStateEnumToControlword)) {}

StateEnum StateBase::getStateEnum() const {
  return stateEnum_;
}

void StateBase::enter() {
  enterBase();
  this->enterDerived();
}

void StateBase::update() {
  updateBase();
  this->updateDerived();
}

void StateBase::leave() {
  this->leaveDerived();
  leaveBase();
}

std::string StateBase::getName() {
  return anydrive_.getName();
}

void StateBase::enterBase() {
  ANYDRIVE_NAMED_INFO("Entered FSM state '" << name_ << "'.");
  enteredCounter_++;
  controlwordSentForState_ = StateEnum::NA;

  // Reset the goal state enum if it has been reached
  if (stateEnum_ == goalStateEnum_) {
    ANYDRIVE_NAMED_DEBUG("Reached goal FSM state.");
    goalStateEnum_ = StateEnum::NA;
  }
}

void StateBase::updateBase() {
  if (!isDone_) {
    return;
  }

  // Send a controlword if a goal state is set and it has not been sent yet.
  if (goalStateEnum_ == StateEnum::NA) {
    return;
  }
  if (controlwordSentForState_ == goalStateEnum_) {
    return;
  }
  auto it = goalStateEnumToControlword_.find(goalStateEnum_);
  if (it == goalStateEnumToControlword_.end()) {
    return;
  }
  anydrive_.sendControlword(it->second);
  controlwordSentForState_ = goalStateEnum_;
}

void StateBase::leaveBase() {
  // Reset controlword.
  anydrive_.getCommunicationInterface()->resetControlword();

  ANYDRIVE_NAMED_DEBUG("Left FSM state '" << name_ << "'.");
}

}  // namespace fsm
}  // namespace anydrive
