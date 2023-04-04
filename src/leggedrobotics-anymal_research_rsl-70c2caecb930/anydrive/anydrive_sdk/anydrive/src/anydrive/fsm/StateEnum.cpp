#include "anydrive/fsm/StateEnum.hpp"

namespace anydrive {
namespace fsm {

uint16_t stateEnumToId(const StateEnum stateEnum) {
  if (stateEnum == StateEnum::Calibrate) {
    return ANYDRIVE_STATE_ID_CALIBRATE;
  }
  if (stateEnum == StateEnum::ColdStart) {
    return ANYDRIVE_STATE_ID_COLDSTART;
  }
  if (stateEnum == StateEnum::Configure) {
    return ANYDRIVE_STATE_ID_CONFIGURE;
  }
  if (stateEnum == StateEnum::ControlOp) {
    return ANYDRIVE_STATE_ID_CONTROLOP;
  }
  if (stateEnum == StateEnum::DeviceMissing) {
    return ANYDRIVE_STATE_ID_DEVICE_MISSING;
  }
  if (stateEnum == StateEnum::Error) {
    return ANYDRIVE_STATE_ID_ERROR;
  }
  if (stateEnum == StateEnum::Fatal) {
    return ANYDRIVE_STATE_ID_FATAL;
  }
  if (stateEnum == StateEnum::MotorOp) {
    return ANYDRIVE_STATE_ID_MOTOROP;
  }
  if (stateEnum == StateEnum::MotorPreOp) {
    return ANYDRIVE_STATE_ID_MOTORPREOP;
  }
  if (stateEnum == StateEnum::Standby) {
    return ANYDRIVE_STATE_ID_STANDBY;
  }
  if (stateEnum == StateEnum::WarmStart) {
    return ANYDRIVE_STATE_ID_WARMSTART;
  }
  return ANYDRIVE_STATE_ID_NA;
}

StateEnum stateIdToEnum(uint16_t stateId) {
  if (stateId == ANYDRIVE_STATE_ID_CALIBRATE) {
    return StateEnum::Calibrate;
  }
  if (stateId == ANYDRIVE_STATE_ID_COLDSTART) {
    return StateEnum::ColdStart;
  }
  if (stateId == ANYDRIVE_STATE_ID_CONFIGURE) {
    return StateEnum::Configure;
  }
  if (stateId == ANYDRIVE_STATE_ID_CONTROLOP) {
    return StateEnum::ControlOp;
  }
  if (stateId == ANYDRIVE_STATE_ID_DEVICE_MISSING) {
    return StateEnum::DeviceMissing;
  }
  if (stateId == ANYDRIVE_STATE_ID_ERROR) {
    return StateEnum::Error;
  }
  if (stateId == ANYDRIVE_STATE_ID_FATAL) {
    return StateEnum::Fatal;
  }
  if (stateId == ANYDRIVE_STATE_ID_MOTOROP) {
    return StateEnum::MotorOp;
  }
  if (stateId == ANYDRIVE_STATE_ID_MOTORPREOP) {
    return StateEnum::MotorPreOp;
  }
  if (stateId == ANYDRIVE_STATE_ID_STANDBY) {
    return StateEnum::Standby;
  }
  if (stateId == ANYDRIVE_STATE_ID_WARMSTART) {
    return StateEnum::WarmStart;
  }
  return StateEnum::NA;
}

std::string stateEnumToName(const StateEnum stateEnum) {
  if (stateEnum == StateEnum::Calibrate) {
    return ANYDRIVE_STATE_NAME_CALIBRATE;
  }
  if (stateEnum == StateEnum::ColdStart) {
    return ANYDRIVE_STATE_NAME_COLDSTART;
  }
  if (stateEnum == StateEnum::Configure) {
    return ANYDRIVE_STATE_NAME_CONFIGURE;
  }
  if (stateEnum == StateEnum::ControlOp) {
    return ANYDRIVE_STATE_NAME_CONTROLOP;
  }
  if (stateEnum == StateEnum::DeviceMissing) {
    return ANYDRIVE_STATE_NAME_DEVICE_MISSING;
  }
  if (stateEnum == StateEnum::Error) {
    return ANYDRIVE_STATE_NAME_ERROR;
  }
  if (stateEnum == StateEnum::Fatal) {
    return ANYDRIVE_STATE_NAME_FATAL;
  }
  if (stateEnum == StateEnum::MotorOp) {
    return ANYDRIVE_STATE_NAME_MOTOROP;
  }
  if (stateEnum == StateEnum::MotorPreOp) {
    return ANYDRIVE_STATE_NAME_MOTORPREOP;
  }
  if (stateEnum == StateEnum::Standby) {
    return ANYDRIVE_STATE_NAME_STANDBY;
  }
  if (stateEnum == StateEnum::WarmStart) {
    return ANYDRIVE_STATE_NAME_WARMSTART;
  }
  return ANYDRIVE_STATE_NAME_NA;
}

StateEnum stateNameToEnum(const std::string& string) {
  if (string == ANYDRIVE_STATE_NAME_CALIBRATE) {
    return StateEnum::Calibrate;
  }
  if (string == ANYDRIVE_STATE_NAME_COLDSTART) {
    return StateEnum::ColdStart;
  }
  if (string == ANYDRIVE_STATE_NAME_CONFIGURE) {
    return StateEnum::Configure;
  }
  if (string == ANYDRIVE_STATE_NAME_CONTROLOP) {
    return StateEnum::ControlOp;
  }
  if (string == ANYDRIVE_STATE_NAME_DEVICE_MISSING) {
    return StateEnum::DeviceMissing;
  }
  if (string == ANYDRIVE_STATE_NAME_ERROR) {
    return StateEnum::Error;
  }
  if (string == ANYDRIVE_STATE_NAME_FATAL) {
    return StateEnum::Fatal;
  }
  if (string == ANYDRIVE_STATE_NAME_MOTOROP) {
    return StateEnum::MotorOp;
  }
  if (string == ANYDRIVE_STATE_NAME_MOTORPREOP) {
    return StateEnum::MotorPreOp;
  }
  if (string == ANYDRIVE_STATE_NAME_STANDBY) {
    return StateEnum::Standby;
  }
  if (string == ANYDRIVE_STATE_NAME_WARMSTART) {
    return StateEnum::WarmStart;
  }
  return StateEnum::NA;
}

std::ostream& operator<<(std::ostream& out, const StateEnum stateEnum) {
  return out << stateEnumToName(stateEnum);
}

}  // namespace fsm
}  // namespace anydrive
