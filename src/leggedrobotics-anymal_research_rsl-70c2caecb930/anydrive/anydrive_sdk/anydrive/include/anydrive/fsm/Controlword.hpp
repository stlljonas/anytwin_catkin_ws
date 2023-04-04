#pragma once

#include <cstdint>
#include <string>

namespace anydrive {
namespace fsm {

// clang-format off

// Controlword IDs.
// Note: This mapping must be equal to the mapping in the firmware.
#define ANYDRIVE_CW_ID_CALIBRATE_TO_CONFIGURE              (0x05)
#define ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_MOTOR_OP            (0x02)
#define ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY             (0x0C)
#define ANYDRIVE_CW_ID_CONFIGURE_TO_CALIBRATE              (0x06)
#define ANYDRIVE_CW_ID_CONFIGURE_TO_STANDBY                (0x04)
#define ANYDRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP              (0x09)
#define ANYDRIVE_CW_ID_CONTROL_OP_TO_STANDBY               (0x0B)
#define ANYDRIVE_CW_ID_MOTOR_OP_TO_CONTROL_OP              (0x0A)
#define ANYDRIVE_CW_ID_MOTOR_OP_TO_STANDBY                 (0x07)
#define ANYDRIVE_CW_ID_NA                                  (0x00)
#define ANYDRIVE_CW_ID_STANDBY_TO_CONFIGURE                (0x03)
#define ANYDRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP              (0x08)
#define ANYDRIVE_CW_ID_WARM_RESET                          (0x01)

// Controlword names.
#define ANYDRIVE_CW_NAME_CALIBRATE_TO_CONFIGURE            ("Calibrate to Configure")
#define ANYDRIVE_CW_NAME_CLEAR_ERRORS_TO_MOTOR_OP          ("Clear Errors to MotorOp")
#define ANYDRIVE_CW_NAME_CLEAR_ERRORS_TO_STANDBY           ("Clear Errors to Standby")
#define ANYDRIVE_CW_NAME_CONFIGURE_TO_CALIBRATE            ("Configure to Calibrate")
#define ANYDRIVE_CW_NAME_CONFIGURE_TO_STANDBY              ("Configure to Standby")
#define ANYDRIVE_CW_NAME_CONTROL_OP_TO_MOTOR_OP            ("ControlOp to MotorOp")
#define ANYDRIVE_CW_NAME_CONTROL_OP_TO_STANDBY             ("ControlOp to Standby")
#define ANYDRIVE_CW_NAME_MOTOR_OP_TO_CONTROL_OP            ("MotorOp to ControlOp")
#define ANYDRIVE_CW_NAME_MOTOR_OP_TO_STANDBY               ("MotorOp to Standby")
#define ANYDRIVE_CW_NAME_NA                                ("N/A")
#define ANYDRIVE_CW_NAME_STANDBY_TO_CONFIGURE              ("Standby to Configure")
#define ANYDRIVE_CW_NAME_STANDBY_TO_MOTOR_PREOP            ("Standby to MotorPreOp")
#define ANYDRIVE_CW_NAME_WARM_RESET                        ("Warm Reset")

// clang-format on

std::string controlwordIdToString(const uint16_t controlwordId);
uint16_t controlwordStringToId(const std::string& controlwordString);

}  // namespace fsm
}  // namespace anydrive
