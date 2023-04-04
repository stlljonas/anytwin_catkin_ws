#include "anydrive/fsm/Controlword.hpp"

namespace anydrive {
namespace fsm {

std::string controlwordIdToString(const uint16_t controlwordId) {
  if (controlwordId == ANYDRIVE_CW_ID_CALIBRATE_TO_CONFIGURE) {
    return ANYDRIVE_CW_NAME_CALIBRATE_TO_CONFIGURE;
  }
  if (controlwordId == ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_MOTOR_OP) {
    return ANYDRIVE_CW_NAME_CLEAR_ERRORS_TO_MOTOR_OP;
  }
  if (controlwordId == ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY) {
    return ANYDRIVE_CW_NAME_CLEAR_ERRORS_TO_STANDBY;
  }
  if (controlwordId == ANYDRIVE_CW_ID_CONFIGURE_TO_CALIBRATE) {
    return ANYDRIVE_CW_NAME_CONFIGURE_TO_CALIBRATE;
  }
  if (controlwordId == ANYDRIVE_CW_ID_CONFIGURE_TO_STANDBY) {
    return ANYDRIVE_CW_NAME_CONFIGURE_TO_STANDBY;
  }
  if (controlwordId == ANYDRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP) {
    return ANYDRIVE_CW_NAME_CONTROL_OP_TO_MOTOR_OP;
  }
  if (controlwordId == ANYDRIVE_CW_ID_CONTROL_OP_TO_STANDBY) {
    return ANYDRIVE_CW_NAME_CONTROL_OP_TO_STANDBY;
  }
  if (controlwordId == ANYDRIVE_CW_ID_MOTOR_OP_TO_CONTROL_OP) {
    return ANYDRIVE_CW_NAME_MOTOR_OP_TO_CONTROL_OP;
  }
  if (controlwordId == ANYDRIVE_CW_ID_MOTOR_OP_TO_STANDBY) {
    return ANYDRIVE_CW_NAME_MOTOR_OP_TO_STANDBY;
  }
  if (controlwordId == ANYDRIVE_CW_ID_STANDBY_TO_CONFIGURE) {
    return ANYDRIVE_CW_NAME_STANDBY_TO_CONFIGURE;
  }
  if (controlwordId == ANYDRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP) {
    return ANYDRIVE_CW_NAME_STANDBY_TO_MOTOR_PREOP;
  }
  if (controlwordId == ANYDRIVE_CW_ID_WARM_RESET) {
    return ANYDRIVE_CW_NAME_WARM_RESET;
  }
  return ANYDRIVE_CW_NAME_NA;
}

uint16_t controlwordStringToId(const std::string& controlwordString) {
  if (controlwordString == ANYDRIVE_CW_NAME_CALIBRATE_TO_CONFIGURE) {
    return ANYDRIVE_CW_ID_CALIBRATE_TO_CONFIGURE;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_CLEAR_ERRORS_TO_MOTOR_OP) {
    return ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_MOTOR_OP;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_CLEAR_ERRORS_TO_STANDBY) {
    return ANYDRIVE_CW_ID_CLEAR_ERRORS_TO_STANDBY;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_CONFIGURE_TO_CALIBRATE) {
    return ANYDRIVE_CW_ID_CONFIGURE_TO_CALIBRATE;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_CONFIGURE_TO_STANDBY) {
    return ANYDRIVE_CW_ID_CONFIGURE_TO_STANDBY;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_CONTROL_OP_TO_MOTOR_OP) {
    return ANYDRIVE_CW_ID_CONTROL_OP_TO_MOTOR_OP;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_CONTROL_OP_TO_STANDBY) {
    return ANYDRIVE_CW_ID_CONTROL_OP_TO_STANDBY;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_MOTOR_OP_TO_CONTROL_OP) {
    return ANYDRIVE_CW_ID_MOTOR_OP_TO_CONTROL_OP;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_MOTOR_OP_TO_STANDBY) {
    return ANYDRIVE_CW_ID_MOTOR_OP_TO_STANDBY;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_STANDBY_TO_CONFIGURE) {
    return ANYDRIVE_CW_ID_STANDBY_TO_CONFIGURE;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_STANDBY_TO_MOTOR_PREOP) {
    return ANYDRIVE_CW_ID_STANDBY_TO_MOTOR_PREOP;
  }
  if (controlwordString == ANYDRIVE_CW_NAME_WARM_RESET) {
    return ANYDRIVE_CW_ID_WARM_RESET;
  }
  return ANYDRIVE_CW_ID_NA;
}

}  // namespace fsm
}  // namespace anydrive
