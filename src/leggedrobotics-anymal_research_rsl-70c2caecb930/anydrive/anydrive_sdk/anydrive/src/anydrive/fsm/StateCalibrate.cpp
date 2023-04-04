#include "anydrive/fsm/StateCalibrate.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateCalibrate::StateCalibrate(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::Calibrate,
                {{StateEnum::Configure, ANYDRIVE_CW_ID_CALIBRATE_TO_CONFIGURE},
                 {StateEnum::Standby, ANYDRIVE_CW_ID_CALIBRATE_TO_CONFIGURE},
                 {StateEnum::MotorOp, ANYDRIVE_CW_ID_CALIBRATE_TO_CONFIGURE},
                 {StateEnum::ControlOp, ANYDRIVE_CW_ID_CALIBRATE_TO_CONFIGURE}}) {}

}  // namespace fsm
}  // namespace anydrive
