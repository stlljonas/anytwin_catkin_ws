#pragma once


// c++
#include <atomic>
#include <mutex>

// conditional state machine
#include "conditional_state_machine/States.hpp"
#include "conditional_state_machine/Substates.hpp"


namespace conditional_state_machine {


template <typename... _Types>
class StateMachine;

template <typename _StateEnumType, typename... _SubstateTypes>
class StateMachine<_StateEnumType, Substates<_SubstateTypes...>>
{
protected:
  using StateEnumType = _StateEnumType;
  using SubstatesType = Substates<_SubstateTypes...>;
  using ConditionsType = Conditions<_SubstateTypes...>;
  using StatesType = States<StateEnumType, ConditionsType>;
  using StatePtrType = StatePtr<StateEnumType, ConditionsType>;

  StatesType states_;

  std::mutex activeSubstatesMutex_;
  SubstatesType activeSubstates_;

  std::mutex activeStateMutex_;
  StatePtrType activeState_;

  std::atomic<StateEnumType> activeStateEnum_;
  std::atomic<StateEnumType> lastActiveStateEnum_;
  std::atomic<StateEnumType> goalStateEnum_;

public:
  StateMachine(const StatesType& states, const SubstatesType& initialSubstates, const StateEnumType& initialState)
  : states_(states),
    activeSubstates_(initialSubstates),
    activeStateEnum_(initialState),
    lastActiveStateEnum_(StateEnumType::NA),
    goalStateEnum_(StateEnumType::NA)
  {
    std::lock_guard<std::mutex> activeStateLock(activeStateMutex_);
    activeState_ = states_.getState(activeStateEnum_);
    activeState_->enter();
  }

  virtual ~StateMachine() {}

  SubstatesType getActiveSubstates()
  {
    std::lock_guard<std::mutex> lock(activeSubstatesMutex_);
    return activeSubstates_;
  }

  void setActiveSubstates(const SubstatesType& activeSubstates)
  {
    std::lock_guard<std::mutex> lock(activeSubstatesMutex_);
    activeSubstates_ = activeSubstates;
  }

  template <typename __SubstateType>
  void setActiveSubstate(__SubstateType activeSubstate)
  {
    std::lock_guard<std::mutex> lock(activeSubstatesMutex_);
    activeSubstates_.template accessElementOfType<__SubstateType>() = activeSubstate;
  }

  StatePtrType getActiveState()
  {
    std::lock_guard<std::mutex> lock(activeStateMutex_);
    return activeState_;
  }

  StateEnumType getActiveStateEnum()
  {
    return activeStateEnum_;
  }

  StateEnumType getLastActiveStateEnum()
  {
    return lastActiveStateEnum_;
  }

  StateEnumType getGoalStateEnum()
  {
    return goalStateEnum_;
  }

  void setGoalStateEnum(StateEnumType goalStateEnum)
  {
    if (goalStateEnum == StateEnumType::NA)
      return;

    if (goalStateEnum == activeStateEnum_)
      return;

    goalStateEnum_ = goalStateEnum;
    CONDITIONAL_STATE_MACHINE_INFO("Goal state enum has been set.");
  }

  void update()
  {
    if (goalStateEnum_ != StateEnumType::NA)
      tryToEnterGoalState();

    std::lock_guard<std::mutex> lock(activeStateMutex_);
    activeState_->update();
  }

  bool checkActiveConditions()
  {
    std::lock_guard<std::mutex> activeStateLock(activeStateMutex_);
    if (!activeState_)
      return false;

    std::lock_guard<std::mutex> activeSubstatesLock(activeSubstatesMutex_);
    return activeState_->conditionsAreFulfilledBy(activeSubstates_);
  }

protected:
  bool tryToEnterGoalState()
  {
    StatePtrType goalState = states_.getState(goalStateEnum_);

    {
      std::lock_guard<std::mutex> activeSubstatesLock(activeSubstatesMutex_);
      if (!goalState->conditionsAreFulfilledBy(activeSubstates_))
      {
        goalStateEnum_ = StateEnumType::NA;
        return false;
      }
    }

    std::lock_guard<std::mutex> activeStateLock(activeStateMutex_);
    if (activeState_)
    {
      activeState_->leave();
    }
    activeState_ = goalState;
    lastActiveStateEnum_ = activeStateEnum_.load();
    activeStateEnum_ = goalStateEnum_.load();
    goalStateEnum_ = StateEnumType::NA;
    activeState_->enter();
    return true;
  }
};


} // conditional_state_machine
