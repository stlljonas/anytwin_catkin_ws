#pragma once

#include <message_logger/message_logger.hpp>
#include <anymal_description/AnymalDescription.hpp>

// anymal lowlevel controller
#include "anymal_lowlevel_controller/state_machine/StateMachine.hpp"


namespace anymal_lowlevel_controller {
namespace state_machine {

using AnymalConditions = conditional_state_machine::Conditions<AnydrivesStateEnum>;


class State : public conditional_state_machine::State<StateEnum, AnymalConditions>
{
protected:
  using Base = conditional_state_machine::State<StateEnum, AnymalConditions>;
  any_measurements::Time enterTime_;
  StateMachine& stateMachine_;

public:
  State(StateEnum stateEnum, const AnymalConditions& conditions, StateMachine& stateMachine)
  : Base(stateEnum, conditions),
    stateMachine_(stateMachine) {}

  virtual ~State() {}

protected:
  StateMachine& getStateMachine()
  {
    return stateMachine_;
  }

  anydrive::AnydriveManagerPtr getAnydriveManager()
  {
    return stateMachine_.getSystem()->getAnydriveManager();
  }

  void enter()
  {
    enterBase();
    this->enterDerived();
  }

  void update()
  {
    updateBase();
    this->updateDerived();
  }

  void leave()
  {
    this->leaveDerived();
    leaveBase();
  }

  void enterBase()
  {
    CONDITIONAL_STATE_MACHINE_INFO("Entering state '" << stateEnumToName(this->stateEnum_) << "' ...");
    enterTime_.setNowWallClock();
  }

  void updateBase()
  {
    CONDITIONAL_STATE_MACHINE_DEBUG("Updating state '" << stateEnumToName(this->stateEnum_) << "' ...");
  }

  void leaveBase()
  {
    CONDITIONAL_STATE_MACHINE_INFO("Leaving state '" << stateEnumToName(this->stateEnum_) << "' after " << enterTime_.getElapsedTime().toSeconds() << " s ...");
  }

  virtual void enterDerived() {}
  virtual void updateDerived() {}
  virtual void leaveDerived() {}
};


class Action : public State
{
public:
  Action(StateEnum stateEnum, const AnymalConditions& conditions, StateMachine& stateMachine)
  : State(stateEnum, conditions, stateMachine) {}

  virtual ~Action() {}

  virtual bool isDone() = 0;

  void updateDerived()
  {
    if (isDone())
      getStateMachine().setGoalStateEnum(stateMachine_.getLastActiveStateEnum());
  }
};

class ActionActuatorsClearErrors : public Action
{
public:
  ActionActuatorsClearErrors(StateMachine& stateMachine)
  : Action(
      StateEnum::ActionActuatorsClearErrors,
      AnymalConditions(
          conditional_state_machine::Condition<AnydrivesStateEnum>({AnydrivesStateEnum::Error})),
      stateMachine) {}

  virtual ~ActionActuatorsClearErrors() {}

  bool isDone()
  {
    return getAnydriveManager()->noDeviceIsInErrorState();
  }

  void enterDerived()
  {
#ifdef ANYDRIVE1X
    getAnydriveManager()->sendClearErrors();
#else
    getAnydriveManager()->sendControlwords(ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_MOTOR_OP);
#endif
  }
};

class ActionActuatorsDisable : public Action
{
public:
  ActionActuatorsDisable(StateMachine& stateMachine)
  : Action(
      StateEnum::ActionActuatorsDisable,
      AnymalConditions(
          conditional_state_machine::Condition<AnydrivesStateEnum>({}, true)),
      stateMachine) {}

  virtual ~ActionActuatorsDisable() {}

  bool isDone()
  {
    return true;
  }

  void enterDerived()
  {
    getAnydriveManager()->setGoalStatesEnum(anydrive::fsm::StateEnum::Standby);
  }
};

class ActionActuatorsEnable : public Action
{
public:
  ActionActuatorsEnable(StateMachine& stateMachine)
  : Action(
      StateEnum::ActionActuatorsEnable,
      AnymalConditions(
          conditional_state_machine::Condition<AnydrivesStateEnum>({AnydrivesStateEnum::Operational, AnydrivesStateEnum::Other})),
      stateMachine) {}

  virtual ~ActionActuatorsEnable() {}

  bool isDone()
  {
    return getAnydriveManager()->allDevicesAreInTheState(anydrive::fsm::StateEnum::ControlOp);
  }

  void enterDerived()
  {
    getAnydriveManager()->setGoalStatesEnum(anydrive::fsm::StateEnum::ControlOp);
  }
};

class ActionActuatorsWarmReset : public Action
{
public:
  ActionActuatorsWarmReset(StateMachine& stateMachine)
  : Action(
      StateEnum::ActionActuatorsWarmReset,
      AnymalConditions(
          conditional_state_machine::Condition<AnydrivesStateEnum>({AnydrivesStateEnum::Fatal})),
      stateMachine) {}

  virtual ~ActionActuatorsWarmReset() {}

  bool isDone()
  {
    return getAnydriveManager()->noDeviceIsInFatalState();
  }

  void enterDerived()
  {

  }
};

class ActionConnectActuators : public Action
{
public:
  ActionConnectActuators(StateMachine& stateMachine)
  : Action(
      StateEnum::ActionConnectActuators,
      AnymalConditions(
          conditional_state_machine::Condition<AnydrivesStateEnum>({}, true)),
      stateMachine) {}

  virtual ~ActionConnectActuators() {}

  bool isDone()
  {
    MELO_INFO("Startup actuators.")
    return stateMachine_.getSystem()->startup();
  }

  void enterDerived()
  {
    MELO_INFO("Enter action connect actuator.")
    stateMachine_.getSystem()->cleanup();
  }

  void leaveDerived()
  {
    MELO_INFO("Leave action connect actuator.")
  }
};

class StateFatal : public State
{
public:
  StateFatal(StateMachine& stateMachine)
  : State(
        StateEnum::StateFatal,
        AnymalConditions(
            conditional_state_machine::Condition<AnydrivesStateEnum>({}, true)),
        stateMachine) {}

  virtual ~StateFatal() {}

  void enterDerived()
  {
    getAnydriveManager()->setGoalStatesEnum(anydrive::fsm::StateEnum::Standby);
  }
};

class StateOperational : public State
{
public:
  StateOperational(StateMachine& stateMachine)
  : State(
        StateEnum::StateOperational,
        AnymalConditions(
            conditional_state_machine::Condition<AnydrivesStateEnum>({AnydrivesStateEnum::Operational})),
        stateMachine) {}

  virtual ~StateOperational() {}

  void leaveDerived()
  {
    getAnydriveManager()->stageFreezes();
  }
};

class StateIdle : public State
{
public:
  StateIdle(StateMachine& stateMachine)
  : State(
        StateEnum::StateIdle,
        AnymalConditions(
            conditional_state_machine::Condition<AnydrivesStateEnum>({}, true)),
        stateMachine) {}

  virtual ~StateIdle() {}
};

class StateZeroJointTorque : public State
{
public:
  StateZeroJointTorque(StateMachine& stateMachine)
  : State(
        StateEnum::StateZeroJointTorque,
        AnymalConditions(
            conditional_state_machine::Condition<AnydrivesStateEnum>({AnydrivesStateEnum::Operational})),
        stateMachine) {}

  virtual ~StateZeroJointTorque() {}

  void enterDerived()
  {
    getAnydriveManager()->stageZeroJointTorques();
  }

  void leaveDerived()
  {
    getAnydriveManager()->stageFreezes();
  }
};


} // state_machine
} // anymal_lowlevel_controller
