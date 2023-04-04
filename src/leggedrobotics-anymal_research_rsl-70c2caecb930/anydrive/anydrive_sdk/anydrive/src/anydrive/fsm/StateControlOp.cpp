#include "anydrive/fsm/StateControlOp.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateControlOp::StateControlOp(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::ControlOp,
                {{StateEnum::Configure, ANYDRIVE_CW_ID_CONTROL_OP_TO_STANDBY},
                 {StateEnum::Standby, ANYDRIVE_CW_ID_CONTROL_OP_TO_STANDBY},
                 {StateEnum::Calibrate, ANYDRIVE_CW_ID_CONTROL_OP_TO_STANDBY},
                 {StateEnum::MotorOp, ANYDRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP}}) {}

void StateControlOp::enterDerived() {
  //  anydrive_.getCommunicationInterface()->configureHeartBeat(false);
}

void StateControlOp::leaveDerived() {
  /* If auto stage last command is enabled, a freeze has to overwrite the staged command.
   * Otherwise a potentially unsafe staged command will again be executed next time ControlOp
   * is reached.
   */
  if (anydrive_.getConfiguration().getAutoStageLastCommand()) {
    anydrive_.stageFreeze();
  }

  //  anydrive_.getCommunicationInterface()->configureHeartBeat(true);
}

}  // namespace fsm
}  // namespace anydrive
