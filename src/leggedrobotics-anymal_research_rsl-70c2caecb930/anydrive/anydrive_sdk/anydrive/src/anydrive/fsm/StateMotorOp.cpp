#include "anydrive/fsm/StateMotorOp.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateMotorOp::StateMotorOp(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::MotorOp,
                {{StateEnum::Configure, ANYDRIVE_CW_ID_MOTOR_OP_TO_STANDBY},
                 {StateEnum::Standby, ANYDRIVE_CW_ID_MOTOR_OP_TO_STANDBY},
                 {StateEnum::Calibrate, ANYDRIVE_CW_ID_MOTOR_OP_TO_STANDBY},
                 {StateEnum::ControlOp, ANYDRIVE_CW_ID_MOTOR_OP_TO_CONTROL_OP}}) {}

}  // namespace fsm
}  // namespace anydrive
