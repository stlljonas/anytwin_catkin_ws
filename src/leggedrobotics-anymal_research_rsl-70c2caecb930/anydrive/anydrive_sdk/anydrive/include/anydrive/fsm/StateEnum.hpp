#pragma once

#include <cstdint>
#include <iostream>
#include <string>

namespace anydrive {
namespace fsm {

// clang-format off

// FSM state IDs.
// NOTE: This mapping must be equal to the mapping in the firmware.
#define ANYDRIVE_STATE_ID_CALIBRATE             (4)
#define ANYDRIVE_STATE_ID_COLDSTART             (1)
#define ANYDRIVE_STATE_ID_CONFIGURE             (3)
#define ANYDRIVE_STATE_ID_CONTROLOP             (7)
#define ANYDRIVE_STATE_ID_DEVICE_MISSING        (11)
#define ANYDRIVE_STATE_ID_ERROR                 (8)
#define ANYDRIVE_STATE_ID_FATAL                 (9)
#define ANYDRIVE_STATE_ID_MOTOROP               (6)
#define ANYDRIVE_STATE_ID_MOTORPREOP            (10)
#define ANYDRIVE_STATE_ID_NA                    (0)
#define ANYDRIVE_STATE_ID_STANDBY               (5)
#define ANYDRIVE_STATE_ID_WARMSTART             (2)

// FSM state names.
#define ANYDRIVE_STATE_NAME_CALIBRATE           ("Calibrate")
#define ANYDRIVE_STATE_NAME_COLDSTART           ("ColdStart")
#define ANYDRIVE_STATE_NAME_CONFIGURE           ("Configure")
#define ANYDRIVE_STATE_NAME_CONTROLOP           ("ControlOp")
#define ANYDRIVE_STATE_NAME_DEVICE_MISSING      ("DeviceMissing")
#define ANYDRIVE_STATE_NAME_ERROR               ("Error")
#define ANYDRIVE_STATE_NAME_FATAL               ("Fatal")
#define ANYDRIVE_STATE_NAME_MOTOROP             ("MotorOp")
#define ANYDRIVE_STATE_NAME_MOTORPREOP          ("MotorPreOp")
#define ANYDRIVE_STATE_NAME_NA                  ("N/A")
#define ANYDRIVE_STATE_NAME_STANDBY             ("Standby")
#define ANYDRIVE_STATE_NAME_WARMSTART           ("WarmStart")

// clang-format on

// FSM state enumerators.
enum class StateEnum {
  Calibrate,
  ColdStart,
  Configure,
  ControlOp,
  DeviceMissing,
  Error,
  Fatal,
  MotorOp,
  MotorPreOp,
  NA,
  Standby,
  WarmStart
};

uint16_t stateEnumToId(const StateEnum stateEnum);
StateEnum stateIdToEnum(uint16_t stateId);

std::string stateEnumToName(const StateEnum stateEnum);
StateEnum stateNameToEnum(const std::string& string);

std::ostream& operator<<(std::ostream& out, const StateEnum stateEnum);

}  // namespace fsm
}  // namespace anydrive
