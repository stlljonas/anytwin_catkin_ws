#pragma once

#include <map>
#include <mutex>

#include "anydrive/fsm/StateBase.hpp"

namespace anydrive {
namespace fsm {

class StateMachine {
 protected:
  Anydrive& anydrive_;

  mutable std::recursive_mutex mutex_;
  std::atomic<StateEnum> activeStateEnum_{StateEnum::NA};
  std::atomic<StateEnum> goalStateEnum_{StateEnum::NA};
  std::map<StateEnum, StateBasePtr> states_;

 public:
  explicit StateMachine(Anydrive& anydrive);
  virtual ~StateMachine() = default;

  void updateActiveState(const StateEnum newActiveStateEnum);

  StateEnum getActiveStateEnum() const;
  StateEnum getGoalStateEnum() const;
  bool goalStateHasBeenReached() const;
  void setGoalStateEnum(const StateEnum goalStateEnum);
  void clearGoalStateEnum();

 protected:
  std::string getName() const;

  void addState(const StateBasePtr& state);
};

}  // namespace fsm
}  // namespace anydrive
