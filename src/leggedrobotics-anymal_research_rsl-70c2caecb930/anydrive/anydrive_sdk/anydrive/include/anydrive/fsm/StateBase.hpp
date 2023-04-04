#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>

#include "anydrive/fsm/Controlword.hpp"
#include "anydrive/fsm/StateEnum.hpp"

namespace anydrive {

class Anydrive;

namespace fsm {

class StateBase {
 public:
  bool isDone_ = true;

 protected:
  Anydrive& anydrive_;
  std::atomic<StateEnum>& goalStateEnum_;

  StateEnum stateEnum_;
  std::string name_;
  unsigned int enteredCounter_ = 0;

  std::map<StateEnum, uint8_t> goalStateEnumToControlword_;
  StateEnum controlwordSentForState_ = StateEnum::NA;

 protected:
  StateBase(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum, const StateEnum stateEnum,
            std::map<StateEnum, uint8_t> goalStateEnumToControlword = {});
  virtual ~StateBase() = default;

 public:
  StateEnum getStateEnum() const;

  void enter();
  void update();
  void leave();

 protected:
  std::string getName();

  void enterBase();
  virtual void enterDerived() {}
  void updateBase();
  virtual void updateDerived() {}
  void leaveBase();
  virtual void leaveDerived() {}
};

using StateBasePtr = std::shared_ptr<StateBase>;

}  // namespace fsm
}  // namespace anydrive
