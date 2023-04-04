#include "anydrive/fsm/StateStandby.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateStandby::StateStandby(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::Standby,
                {{StateEnum::Configure, ANYDRIVE_CW_ID_STANDBY_TO_CONFIGURE},
                 {StateEnum::Calibrate, ANYDRIVE_CW_ID_STANDBY_TO_CONFIGURE},
                 {StateEnum::MotorOp, ANYDRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP},
                 {StateEnum::ControlOp, ANYDRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP}}) {}

}  // namespace fsm
}  // namespace anydrive
