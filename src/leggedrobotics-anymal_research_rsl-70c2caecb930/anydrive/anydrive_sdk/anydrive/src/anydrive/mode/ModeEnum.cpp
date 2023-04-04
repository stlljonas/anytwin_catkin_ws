#include "anydrive/mode/ModeEnum.hpp"

namespace anydrive {
namespace mode {

uint16_t modeEnumToId(const ModeEnum modeEnum) {
  if (modeEnum == ModeEnum::Current) {
    return ANYDRIVE_MODE_ID_CURRENT;
  }
  if (modeEnum == ModeEnum::DemoChangeGravity) {
    return ANYDRIVE_MODE_ID_DEMO_CHANGE_GRAVITY;
  }
  if (modeEnum == ModeEnum::DemoSafeJointVelocity) {
    return ANYDRIVE_MODE_ID_DEMO_SAFE_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::Disable) {
    return ANYDRIVE_MODE_ID_DISABLE;
  }
  if (modeEnum == ModeEnum::Freeze) {
    return ANYDRIVE_MODE_ID_FREEZE;
  }
  if (modeEnum == ModeEnum::GearPosition) {
    return ANYDRIVE_MODE_ID_GEAR_POS;
  }
  if (modeEnum == ModeEnum::GearVelocity) {
    return ANYDRIVE_MODE_ID_GEAR_VEL;
  }
  if (modeEnum == ModeEnum::JointPosition) {
    return ANYDRIVE_MODE_ID_JOINT_POS;
  }
  if (modeEnum == ModeEnum::JointPositionVelocity) {
    return ANYDRIVE_MODE_ID_JOINT_POS_VEL;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorque) {
    return ANYDRIVE_MODE_ID_JOINT_POS_VEL_TOR;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorquePidGains) {
    return ANYDRIVE_MODE_ID_JOINT_POS_VEL_TOR_PID;
  }
  if (modeEnum == ModeEnum::JointTorque) {
    return ANYDRIVE_MODE_ID_JOINT_TOR;
  }
  if (modeEnum == ModeEnum::JointVelocity) {
    return ANYDRIVE_MODE_ID_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::MotorPosition) {
    return ANYDRIVE_MODE_ID_MOTOR_POS;
  }
  if (modeEnum == ModeEnum::MotorVelocity) {
    return ANYDRIVE_MODE_ID_MOTOR_VEL;
  }
  return ANYDRIVE_MODE_ID_NA;
}

ModeEnum modeIdToEnum(const uint16_t modeId) {
  if (modeId == ANYDRIVE_MODE_ID_CURRENT) {
    return ModeEnum::Current;
  }
  if (modeId == ANYDRIVE_MODE_ID_DEMO_CHANGE_GRAVITY) {
    return ModeEnum::DemoChangeGravity;
  }
  if (modeId == ANYDRIVE_MODE_ID_DEMO_SAFE_JOINT_VEL) {
    return ModeEnum::DemoSafeJointVelocity;
  }
  if (modeId == ANYDRIVE_MODE_ID_DISABLE) {
    return ModeEnum::Disable;
  }
  if (modeId == ANYDRIVE_MODE_ID_FREEZE) {
    return ModeEnum::Freeze;
  }
  if (modeId == ANYDRIVE_MODE_ID_GEAR_POS) {
    return ModeEnum::GearPosition;
  }
  if (modeId == ANYDRIVE_MODE_ID_GEAR_VEL) {
    return ModeEnum::GearVelocity;
  }
  if (modeId == ANYDRIVE_MODE_ID_JOINT_POS) {
    return ModeEnum::JointPosition;
  }
  if (modeId == ANYDRIVE_MODE_ID_JOINT_POS_VEL) {
    return ModeEnum::JointPositionVelocity;
  }
  if (modeId == ANYDRIVE_MODE_ID_JOINT_POS_VEL_TOR) {
    return ModeEnum::JointPositionVelocityTorque;
  }
  if (modeId == ANYDRIVE_MODE_ID_JOINT_POS_VEL_TOR_PID) {
    return ModeEnum::JointPositionVelocityTorquePidGains;
  }
  if (modeId == ANYDRIVE_MODE_ID_JOINT_TOR) {
    return ModeEnum::JointTorque;
  }
  if (modeId == ANYDRIVE_MODE_ID_JOINT_VEL) {
    return ModeEnum::JointVelocity;
  }
  if (modeId == ANYDRIVE_MODE_ID_MOTOR_POS) {
    return ModeEnum::MotorPosition;
  }
  if (modeId == ANYDRIVE_MODE_ID_MOTOR_VEL) {
    return ModeEnum::MotorVelocity;
  }
  return ModeEnum::NA;
}

std::string modeEnumToName(const ModeEnum modeEnum) {
  if (modeEnum == ModeEnum::Current) {
    return ANYDRIVE_MODE_NAME_CURRENT;
  }
  if (modeEnum == ModeEnum::DemoChangeGravity) {
    return ANYDRIVE_MODE_NAME_DEMO_CHANGE_GRAVITY;
  }
  if (modeEnum == ModeEnum::DemoSafeJointVelocity) {
    return ANYDRIVE_MODE_NAME_DEMO_SAFE_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::Disable) {
    return ANYDRIVE_MODE_NAME_DISABLE;
  }
  if (modeEnum == ModeEnum::Freeze) {
    return ANYDRIVE_MODE_NAME_FREEZE;
  }
  if (modeEnum == ModeEnum::GearPosition) {
    return ANYDRIVE_MODE_NAME_GEAR_POS;
  }
  if (modeEnum == ModeEnum::GearVelocity) {
    return ANYDRIVE_MODE_NAME_GEAR_VEL;
  }
  if (modeEnum == ModeEnum::JointPosition) {
    return ANYDRIVE_MODE_NAME_JOINT_POS;
  }
  if (modeEnum == ModeEnum::JointPositionVelocity) {
    return ANYDRIVE_MODE_NAME_JOINT_POS_VEL;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorque) {
    return ANYDRIVE_MODE_NAME_JOINT_POS_VEL_TOR;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorquePidGains) {
    return ANYDRIVE_MODE_NAME_JOINT_POS_VEL_TOR_PID;
  }
  if (modeEnum == ModeEnum::JointTorque) {
    return ANYDRIVE_MODE_NAME_JOINT_TOR;
  }
  if (modeEnum == ModeEnum::JointVelocity) {
    return ANYDRIVE_MODE_NAME_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::MotorPosition) {
    return ANYDRIVE_MODE_NAME_MOTOR_POS;
  }
  if (modeEnum == ModeEnum::MotorVelocity) {
    return ANYDRIVE_MODE_NAME_MOTOR_VEL;
  }
  return ANYDRIVE_MODE_NAME_NA;
}

std::string modeEnumToShortName(const ModeEnum modeEnum) {
  if (modeEnum == ModeEnum::Current) {
    return ANYDRIVE_MODE_NAME_SHORT_CURRENT;
  }
  if (modeEnum == ModeEnum::DemoChangeGravity) {
    return ANYDRIVE_MODE_NAME_SHORT_DEMO_CHANGE_GRAVITY;
  }
  if (modeEnum == ModeEnum::DemoSafeJointVelocity) {
    return ANYDRIVE_MODE_NAME_SHORT_DEMO_SAFE_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::Disable) {
    return ANYDRIVE_MODE_NAME_SHORT_DISABLE;
  }
  if (modeEnum == ModeEnum::Freeze) {
    return ANYDRIVE_MODE_NAME_SHORT_FREEZE;
  }
  if (modeEnum == ModeEnum::GearPosition) {
    return ANYDRIVE_MODE_NAME_SHORT_GEAR_POS;
  }
  if (modeEnum == ModeEnum::GearVelocity) {
    return ANYDRIVE_MODE_NAME_SHORT_GEAR_VEL;
  }
  if (modeEnum == ModeEnum::JointPosition) {
    return ANYDRIVE_MODE_NAME_SHORT_JOINT_POS;
  }
  if (modeEnum == ModeEnum::JointPositionVelocity) {
    return ANYDRIVE_MODE_NAME_SHORT_JOINT_POS_VEL;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorque) {
    return ANYDRIVE_MODE_NAME_SHORT_JOINT_POS_VEL_TOR;
  }
  if (modeEnum == ModeEnum::JointPositionVelocityTorquePidGains) {
    return ANYDRIVE_MODE_NAME_SHORT_JOINT_POS_VEL_TOR_PID;
  }
  if (modeEnum == ModeEnum::JointTorque) {
    return ANYDRIVE_MODE_NAME_SHORT_JOINT_TOR;
  }
  if (modeEnum == ModeEnum::JointVelocity) {
    return ANYDRIVE_MODE_NAME_SHORT_JOINT_VEL;
  }
  if (modeEnum == ModeEnum::MotorPosition) {
    return ANYDRIVE_MODE_NAME_SHORT_MOTOR_POS;
  }
  if (modeEnum == ModeEnum::MotorVelocity) {
    return ANYDRIVE_MODE_NAME_SHORT_MOTOR_VEL;
  }
  return ANYDRIVE_MODE_NAME_SHORT_NA;
}

ModeEnum modeNameToEnum(const std::string& string) {
  if (string == ANYDRIVE_MODE_NAME_CURRENT) {
    return ModeEnum::Current;
  }
  if (string == ANYDRIVE_MODE_NAME_DEMO_CHANGE_GRAVITY) {
    return ModeEnum::DemoChangeGravity;
  }
  if (string == ANYDRIVE_MODE_NAME_DEMO_SAFE_JOINT_VEL) {
    return ModeEnum::DemoSafeJointVelocity;
  }
  if (string == ANYDRIVE_MODE_NAME_DISABLE) {
    return ModeEnum::Disable;
  }
  if (string == ANYDRIVE_MODE_NAME_FREEZE) {
    return ModeEnum::Freeze;
  }
  if (string == ANYDRIVE_MODE_NAME_GEAR_POS) {
    return ModeEnum::GearPosition;
  }
  if (string == ANYDRIVE_MODE_NAME_GEAR_VEL) {
    return ModeEnum::GearVelocity;
  }
  if (string == ANYDRIVE_MODE_NAME_JOINT_POS) {
    return ModeEnum::JointPosition;
  }
  if (string == ANYDRIVE_MODE_NAME_JOINT_POS_VEL) {
    return ModeEnum::JointPositionVelocity;
  }
  if (string == ANYDRIVE_MODE_NAME_JOINT_POS_VEL_TOR) {
    return ModeEnum::JointPositionVelocityTorque;
  }
  if (string == ANYDRIVE_MODE_NAME_JOINT_POS_VEL_TOR_PID) {
    return ModeEnum::JointPositionVelocityTorquePidGains;
  }
  if (string == ANYDRIVE_MODE_NAME_JOINT_TOR) {
    return ModeEnum::JointTorque;
  }
  if (string == ANYDRIVE_MODE_NAME_JOINT_VEL) {
    return ModeEnum::JointVelocity;
  }
  if (string == ANYDRIVE_MODE_NAME_MOTOR_POS) {
    return ModeEnum::MotorPosition;
  }
  if (string == ANYDRIVE_MODE_NAME_MOTOR_VEL) {
    return ModeEnum::MotorVelocity;
  }
  return ModeEnum::NA;
}

std::ostream& operator<<(std::ostream& out, const ModeEnum modeEnum) {
  return out << modeEnumToName(modeEnum);
}

}  // namespace mode
}  // namespace anydrive
