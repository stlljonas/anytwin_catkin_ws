#pragma once

#include <cstdint>
#include <iostream>
#include <string>

namespace anydrive {
namespace mode {

// clang-format off

// Mode IDs.
// This mapping must be equal to the firmware mapping.
#define ANYDRIVE_MODE_ID_CURRENT                              (3)  // Track current
#define ANYDRIVE_MODE_ID_DEMO_CHANGE_GRAVITY                  (14) // Demo mode: Compensate or change gravity. Do not use this.
#define ANYDRIVE_MODE_ID_DEMO_SAFE_JOINT_VEL                  (15) // Demo mode: Joint velocity control with torque supervision. Do not use this.
#define ANYDRIVE_MODE_ID_DISABLE                              (2)  // Disable motor
#define ANYDRIVE_MODE_ID_FREEZE                               (1)  // Freeze motor
#define ANYDRIVE_MODE_ID_GEAR_POS                             (6)  // Track gear position
#define ANYDRIVE_MODE_ID_GEAR_VEL                             (7)  // Track gear velocity
#define ANYDRIVE_MODE_ID_JOINT_POS                            (8)  // Track joint position
#define ANYDRIVE_MODE_ID_JOINT_POS_VEL                        (11) // Track joint position with feedforward velocity
#define ANYDRIVE_MODE_ID_JOINT_POS_VEL_TOR                    (12) // Track joint position with feedforward velocity and torque
#define ANYDRIVE_MODE_ID_JOINT_POS_VEL_TOR_PID                (13) // Track joint position with feedforward velocity and torque using custom joint position gains
#define ANYDRIVE_MODE_ID_JOINT_TOR                            (10) // Track joint torque
#define ANYDRIVE_MODE_ID_JOINT_VEL                            (9)  // Track joint velocity
#define ANYDRIVE_MODE_ID_MOTOR_POS                            (4)  // Track motor position
#define ANYDRIVE_MODE_ID_MOTOR_VEL                            (5)  // Track motor velocity
#define ANYDRIVE_MODE_ID_NA                                   (0)  // Not available

// Mode names.
#define ANYDRIVE_MODE_NAME_CURRENT                            ("Current")
#define ANYDRIVE_MODE_NAME_DEMO_CHANGE_GRAVITY                ("DemoChangeGravity")
#define ANYDRIVE_MODE_NAME_DEMO_SAFE_JOINT_VEL                ("DemoSafeJointVelocity")
#define ANYDRIVE_MODE_NAME_DISABLE                            ("Disable")
#define ANYDRIVE_MODE_NAME_FREEZE                             ("Freeze")
#define ANYDRIVE_MODE_NAME_GEAR_POS                           ("GearPosition")
#define ANYDRIVE_MODE_NAME_GEAR_VEL                           ("GearVelocity")
#define ANYDRIVE_MODE_NAME_JOINT_POS                          ("JointPosition")
#define ANYDRIVE_MODE_NAME_JOINT_POS_VEL                      ("JointPositionVelocity")
#define ANYDRIVE_MODE_NAME_JOINT_POS_VEL_TOR                  ("JointPositionVelocityTorque")
#define ANYDRIVE_MODE_NAME_JOINT_POS_VEL_TOR_PID              ("JointPositionVelocityTorquePidGains")
#define ANYDRIVE_MODE_NAME_JOINT_TOR                          ("JointTorque")
#define ANYDRIVE_MODE_NAME_JOINT_VEL                          ("JointVelocity")
#define ANYDRIVE_MODE_NAME_MOTOR_POS                          ("MotorPosition")
#define ANYDRIVE_MODE_NAME_MOTOR_VEL                          ("MotorVelocity")
#define ANYDRIVE_MODE_NAME_NA                                 ("N/A")

// Mode short names.
#define ANYDRIVE_MODE_NAME_SHORT_CURRENT                      ("C")
#define ANYDRIVE_MODE_NAME_SHORT_DEMO_CHANGE_GRAVITY          ("DCG")
#define ANYDRIVE_MODE_NAME_SHORT_DEMO_SAFE_JOINT_VEL          ("DSJV")
#define ANYDRIVE_MODE_NAME_SHORT_DISABLE                      ("D")
#define ANYDRIVE_MODE_NAME_SHORT_FREEZE                       ("F")
#define ANYDRIVE_MODE_NAME_SHORT_GEAR_POS                     ("GP")
#define ANYDRIVE_MODE_NAME_SHORT_GEAR_VEL                     ("GV")
#define ANYDRIVE_MODE_NAME_SHORT_JOINT_POS                    ("JP")
#define ANYDRIVE_MODE_NAME_SHORT_JOINT_POS_VEL                ("JPV")
#define ANYDRIVE_MODE_NAME_SHORT_JOINT_POS_VEL_TOR            ("JPVT")
#define ANYDRIVE_MODE_NAME_SHORT_JOINT_POS_VEL_TOR_PID        ("JPVTPID")
#define ANYDRIVE_MODE_NAME_SHORT_JOINT_TOR                    ("JT")
#define ANYDRIVE_MODE_NAME_SHORT_JOINT_VEL                    ("JV")
#define ANYDRIVE_MODE_NAME_SHORT_MOTOR_POS                    ("MP")
#define ANYDRIVE_MODE_NAME_SHORT_MOTOR_VEL                    ("MV")
#define ANYDRIVE_MODE_NAME_SHORT_NA                           ("N/A")

// clang-format on

// Mode enumerators.
enum class ModeEnum {
  Current,
  DemoChangeGravity,
  DemoSafeJointVelocity,
  Disable,
  Freeze,
  GearPosition,
  GearVelocity,
  JointPosition,
  JointPositionVelocity,
  JointPositionVelocityTorque,
  JointPositionVelocityTorquePidGains,
  JointTorque,
  JointVelocity,
  MotorPosition,
  MotorVelocity,
  NA
};

uint16_t modeEnumToId(const ModeEnum modeEnum);
ModeEnum modeIdToEnum(const uint16_t modeId);

std::string modeEnumToName(const ModeEnum modeEnum);
std::string modeEnumToShortName(const ModeEnum modeEnum);
ModeEnum modeNameToEnum(const std::string& string);

std::ostream& operator<<(std::ostream& out, const ModeEnum modeEnum);

}  // namespace mode
}  // namespace anydrive
