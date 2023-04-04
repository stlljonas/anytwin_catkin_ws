#include <any_measurements_ros/ConvertRosMessages.hpp>
#include <param_io/get_param.hpp>

#include "anydrive_ros/conversions.hpp"

namespace anydrive_ros {

anydrive::calibration::CalibrationTypeEnum calibrationTypeMsgToEnum(anydrive_msgs::CalibrationType calibrationTypeMsg) {
  if (calibrationTypeMsg.calibration_type == anydrive_msgs::CalibrationType::Custom) {
    return anydrive::calibration::CalibrationTypeEnum::Custom;
  }
  if (calibrationTypeMsg.calibration_type == anydrive_msgs::CalibrationType::Factory) {
    return anydrive::calibration::CalibrationTypeEnum::Factory;
  }
  return anydrive::calibration::CalibrationTypeEnum::NA;
}

anydrive_msgs::CalibrationType calibrationTypeEnumToMsg(anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum) {
  anydrive_msgs::CalibrationType calibrationTypeMsg;
  calibrationTypeMsg.calibration_type = anydrive_msgs::CalibrationType::NA;
  if (calibrationTypeEnum == anydrive::calibration::CalibrationTypeEnum::Custom) {
    calibrationTypeMsg.calibration_type = anydrive_msgs::CalibrationType::Custom;
  }
  if (calibrationTypeEnum == anydrive::calibration::CalibrationTypeEnum::Factory) {
    calibrationTypeMsg.calibration_type = anydrive_msgs::CalibrationType::Factory;
  }
  return calibrationTypeMsg;
}

anydrive::calibration::CalibrationModeEnum calibrationModeMsgToEnum(anydrive_msgs::CalibrationMode calibrationModeMsg) {
  if (calibrationModeMsg.calibration_mode == anydrive_msgs::CalibrationMode::GearJointEncoderOffset) {
    return anydrive::calibration::CalibrationModeEnum::GearJointEncoderOffset;
  }
  if (calibrationModeMsg.calibration_mode == anydrive_msgs::CalibrationMode::FrictionEstimation) {
    return anydrive::calibration::CalibrationModeEnum::FrictionEstimation;
  }
  if (calibrationModeMsg.calibration_mode == anydrive_msgs::CalibrationMode::GravityCompensation) {
    return anydrive::calibration::CalibrationModeEnum::GravityCompensation;
  }
  if (calibrationModeMsg.calibration_mode == anydrive_msgs::CalibrationMode::GearAndJointEncoderHoming) {
    return anydrive::calibration::CalibrationModeEnum::GearAndJointEncoderHoming;
  }
  if (calibrationModeMsg.calibration_mode == anydrive_msgs::CalibrationMode::MotorEncoderOffset) {
    return anydrive::calibration::CalibrationModeEnum::MotorEncoderOffset;
  }
  if (calibrationModeMsg.calibration_mode == anydrive_msgs::CalibrationMode::SafeJointVelocity) {
    return anydrive::calibration::CalibrationModeEnum::SafeJointVelocity;
  }
  if (calibrationModeMsg.calibration_mode == anydrive_msgs::CalibrationMode::MotorEncoderParameters) {
    return anydrive::calibration::CalibrationModeEnum::MotorEncoderParameters;
  }
  if (calibrationModeMsg.calibration_mode == anydrive_msgs::CalibrationMode::ImuGyroscopeDcBias) {
    return anydrive::calibration::CalibrationModeEnum::ImuGyroscopeDcBias;
  }
  return anydrive::calibration::CalibrationModeEnum::NA;
}

anydrive_msgs::CalibrationMode calibrationModeEnumToMsg(anydrive::calibration::CalibrationModeEnum calibrationModeEnum) {
  anydrive_msgs::CalibrationMode calibrationModeMsg;
  calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::NA;
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::GearJointEncoderOffset) {
    calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::GearJointEncoderOffset;
  }
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::FrictionEstimation) {
    calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::FrictionEstimation;
  }
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::GravityCompensation) {
    calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::GravityCompensation;
  }
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::GearAndJointEncoderHoming) {
    calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::GearAndJointEncoderHoming;
  }
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::MotorEncoderOffset) {
    calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::MotorEncoderOffset;
  }
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::SafeJointVelocity) {
    calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::SafeJointVelocity;
  }
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::MotorEncoderParameters) {
    calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::MotorEncoderParameters;
  }
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::ImuGyroscopeDcBias) {
    calibrationModeMsg.calibration_mode = anydrive_msgs::CalibrationMode::ImuGyroscopeDcBias;
  }
  return calibrationModeMsg;
}

anydrive::mode::ModeEnum modeMsgToEnum(anydrive_msgs::Mode modeMsg) {
  if (modeMsg.mode == anydrive_msgs::Mode::Current) {
    return anydrive::mode::ModeEnum::Current;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::Disable) {
    return anydrive::mode::ModeEnum::Disable;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::Freeze) {
    return anydrive::mode::ModeEnum::Freeze;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::GearPosition) {
    return anydrive::mode::ModeEnum::GearPosition;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::GearVelocity) {
    return anydrive::mode::ModeEnum::GearVelocity;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::MotorPosition) {
    return anydrive::mode::ModeEnum::MotorPosition;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::MotorVelocity) {
    return anydrive::mode::ModeEnum::MotorVelocity;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::JointPosition) {
    return anydrive::mode::ModeEnum::JointPosition;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::JointVelocity) {
    return anydrive::mode::ModeEnum::JointVelocity;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::JointTorque) {
    return anydrive::mode::ModeEnum::JointTorque;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::JointPositionVelocity) {
    return anydrive::mode::ModeEnum::JointPositionVelocity;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::JointPositionVelocityTorque) {
    return anydrive::mode::ModeEnum::JointPositionVelocityTorque;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::JointPositionVelocityTorquePidGains) {
    return anydrive::mode::ModeEnum::JointPositionVelocityTorquePidGains;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::DemoChangeGravity) {
    return anydrive::mode::ModeEnum::DemoChangeGravity;
  }
  if (modeMsg.mode == anydrive_msgs::Mode::DemoSafeJointVelocity) {
    return anydrive::mode::ModeEnum::DemoSafeJointVelocity;
  }
  return anydrive::mode::ModeEnum::NA;
}

anydrive_msgs::Mode modeEnumToMsg(anydrive::mode::ModeEnum modeEnum) {
  anydrive_msgs::Mode modeMsg;
  modeMsg.mode = anydrive_msgs::Mode::NA;
  if (modeEnum == anydrive::mode::ModeEnum::Current) {
    modeMsg.mode = anydrive_msgs::Mode::Current;
  }
  if (modeEnum == anydrive::mode::ModeEnum::Disable) {
    modeMsg.mode = anydrive_msgs::Mode::Disable;
  }
  if (modeEnum == anydrive::mode::ModeEnum::Freeze) {
    modeMsg.mode = anydrive_msgs::Mode::Freeze;
  }
  if (modeEnum == anydrive::mode::ModeEnum::GearPosition) {
    modeMsg.mode = anydrive_msgs::Mode::GearPosition;
  }
  if (modeEnum == anydrive::mode::ModeEnum::GearVelocity) {
    modeMsg.mode = anydrive_msgs::Mode::GearVelocity;
  }
  if (modeEnum == anydrive::mode::ModeEnum::MotorPosition) {
    modeMsg.mode = anydrive_msgs::Mode::MotorPosition;
  }
  if (modeEnum == anydrive::mode::ModeEnum::MotorVelocity) {
    modeMsg.mode = anydrive_msgs::Mode::MotorVelocity;
  }
  if (modeEnum == anydrive::mode::ModeEnum::JointPosition) {
    modeMsg.mode = anydrive_msgs::Mode::JointPosition;
  }
  if (modeEnum == anydrive::mode::ModeEnum::JointVelocity) {
    modeMsg.mode = anydrive_msgs::Mode::JointVelocity;
  }
  if (modeEnum == anydrive::mode::ModeEnum::JointTorque) {
    modeMsg.mode = anydrive_msgs::Mode::JointTorque;
  }
  if (modeEnum == anydrive::mode::ModeEnum::JointPositionVelocity) {
    modeMsg.mode = anydrive_msgs::Mode::JointPositionVelocity;
  }
  if (modeEnum == anydrive::mode::ModeEnum::JointPositionVelocityTorque) {
    modeMsg.mode = anydrive_msgs::Mode::JointPositionVelocityTorque;
  }
  if (modeEnum == anydrive::mode::ModeEnum::JointPositionVelocityTorquePidGains) {
    modeMsg.mode = anydrive_msgs::Mode::JointPositionVelocityTorquePidGains;
  }
  if (modeEnum == anydrive::mode::ModeEnum::DemoChangeGravity) {
    modeMsg.mode = anydrive_msgs::Mode::DemoChangeGravity;
  }
  if (modeEnum == anydrive::mode::ModeEnum::DemoSafeJointVelocity) {
    modeMsg.mode = anydrive_msgs::Mode::DemoSafeJointVelocity;
  }
  return modeMsg;
}

anydrive::fsm::StateEnum stateMsgToEnum(anydrive_msgs::FsmState fsmStateMsg) {
  if (fsmStateMsg.state == anydrive_msgs::FsmState::Calibrate) {
    return anydrive::fsm::StateEnum::Calibrate;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::ColdStart) {
    return anydrive::fsm::StateEnum::ColdStart;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::Configure) {
    return anydrive::fsm::StateEnum::Configure;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::ControlOp) {
    return anydrive::fsm::StateEnum::ControlOp;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::DeviceMissing) {
    return anydrive::fsm::StateEnum::DeviceMissing;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::Error) {
    return anydrive::fsm::StateEnum::Error;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::Fatal) {
    return anydrive::fsm::StateEnum::Fatal;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::MotorOp) {
    return anydrive::fsm::StateEnum::MotorOp;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::Standby) {
    return anydrive::fsm::StateEnum::Standby;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::MotorPreOp) {
    return anydrive::fsm::StateEnum::MotorPreOp;
  }
  if (fsmStateMsg.state == anydrive_msgs::FsmState::WarmStart) {
    return anydrive::fsm::StateEnum::WarmStart;
  }
  return anydrive::fsm::StateEnum::NA;
}

anydrive_msgs::FsmState stateEnumToMsg(anydrive::fsm::StateEnum stateEnum) {
  anydrive_msgs::FsmState stateMsg;
  stateMsg.state = anydrive_msgs::FsmState::NA;
  if (stateEnum == anydrive::fsm::StateEnum::Calibrate) {
    stateMsg.state = anydrive_msgs::FsmState::Calibrate;
  }
  if (stateEnum == anydrive::fsm::StateEnum::ColdStart) {
    stateMsg.state = anydrive_msgs::FsmState::ColdStart;
  }
  if (stateEnum == anydrive::fsm::StateEnum::Configure) {
    stateMsg.state = anydrive_msgs::FsmState::Configure;
  }
  if (stateEnum == anydrive::fsm::StateEnum::ControlOp) {
    stateMsg.state = anydrive_msgs::FsmState::ControlOp;
  }
  if (stateEnum == anydrive::fsm::StateEnum::DeviceMissing) {
    stateMsg.state = anydrive_msgs::FsmState::DeviceMissing;
  }
  if (stateEnum == anydrive::fsm::StateEnum::Error) {
    stateMsg.state = anydrive_msgs::FsmState::Error;
  }
  if (stateEnum == anydrive::fsm::StateEnum::Fatal) {
    stateMsg.state = anydrive_msgs::FsmState::Fatal;
  }
  if (stateEnum == anydrive::fsm::StateEnum::MotorOp) {
    stateMsg.state = anydrive_msgs::FsmState::MotorOp;
  }
  if (stateEnum == anydrive::fsm::StateEnum::Standby) {
    stateMsg.state = anydrive_msgs::FsmState::Standby;
  }
  if (stateEnum == anydrive::fsm::StateEnum::MotorPreOp) {
    stateMsg.state = anydrive_msgs::FsmState::MotorPreOp;
  }
  if (stateEnum == anydrive::fsm::StateEnum::WarmStart) {
    stateMsg.state = anydrive_msgs::FsmState::WarmStart;
  }
  return stateMsg;
}

void writeToMessage(anydrive_msgs::Command& message, const anydrive::Command& command) {
  message.header.stamp = any_measurements_ros::toRos(command.getStamp());
  message.mode = modeEnumToMsg(command.getModeEnum());
  message.current = command.getCurrent();
  message.motor_position = command.getMotorPosition();
  message.motor_velocity = command.getMotorVelocity();
  message.gear_position = command.getGearPosition();
  message.gear_velocity = command.getGearVelocity();
  message.joint_position = command.getJointPosition();
  message.joint_velocity = command.getJointVelocity();
  message.joint_torque = command.getJointTorque();
  message.pid_gains_p = command.getPidGains().getP();
  message.pid_gains_i = command.getPidGains().getI();
  message.pid_gains_d = command.getPidGains().getD();
}

void readFromMessage(anydrive::Command& command, const anydrive_msgs::Command& message) {
  command.setStamp(any_measurements_ros::fromRos(message.header.stamp));
  command.setModeEnum(modeMsgToEnum(message.mode));
  command.setCurrent(message.current);
  command.setMotorPosition(message.motor_position);
  command.setMotorVelocity(message.motor_velocity);
  command.setGearPosition(message.gear_position);
  command.setGearVelocity(message.gear_velocity);
  command.setJointPosition(message.joint_position);
  command.setJointVelocity(message.joint_velocity);
  command.setJointTorque(message.joint_torque);
  command.getPidGains().setP(message.pid_gains_p);
  command.getPidGains().setI(message.pid_gains_i);
  command.getPidGains().setD(message.pid_gains_d);
}

void writeToMessage(anydrive_msgs::State& message, const anydrive::State& state) {
  message.header.stamp = any_measurements_ros::toRos(state.getStamp());
  message.statusword = state.getStatusword().getData();
  message.current = state.getCurrent();
  message.gear_position = state.getGearPosition();
  message.gear_velocity = state.getGearVelocity();
  message.joint_position = state.getJointPosition();
  message.joint_velocity = state.getJointVelocity();
  message.joint_acceleration = state.getJointAcceleration();
  message.joint_torque = state.getJointTorque();
  message.imu.linear_acceleration.x = state.getImu().linearAcceleration_.x();
  message.imu.linear_acceleration.y = state.getImu().linearAcceleration_.y();
  message.imu.linear_acceleration.z = state.getImu().linearAcceleration_.z();
  message.imu.angular_velocity.x = state.getImu().angularVelocity_.x();
  message.imu.angular_velocity.y = state.getImu().angularVelocity_.y();
  message.imu.angular_velocity.z = state.getImu().angularVelocity_.z();
}

void readFromMessage(anydrive::State& state, const anydrive_msgs::State& message) {
  state.setStamp(any_measurements_ros::fromRos(message.header.stamp));
  state.setStatusword(anydrive::Statusword(message.statusword));
  state.setCurrent(message.current);
  state.setGearPosition(message.gear_position);
  state.setGearVelocity(message.gear_velocity);
  state.setJointPosition(message.joint_position);
  state.setJointVelocity(message.joint_velocity);
  state.setJointAcceleration(message.joint_acceleration);
  state.setJointTorque(message.joint_torque);
  any_measurements::Imu imu;
  imu.linearAcceleration_.x() = message.imu.linear_acceleration.x;
  imu.linearAcceleration_.y() = message.imu.linear_acceleration.y;
  imu.linearAcceleration_.z() = message.imu.linear_acceleration.z;
  imu.angularVelocity_.x() = message.imu.angular_velocity.x;
  imu.angularVelocity_.y() = message.imu.angular_velocity.y;
  imu.angularVelocity_.z() = message.imu.angular_velocity.z;
  state.setImu(imu);
}

void writeToMessage(anydrive_msgs::StateExtended& message, const anydrive::StateExtended& state) {
  message.header.stamp = any_measurements_ros::toRos(state.getStamp());
  message.statusword = state.getStatusword().getData();
  message.current = state.getCurrent();
  message.motor_position = state.getMotorPosition();
  message.motor_velocity = state.getMotorVelocity();
  message.gear_position = state.getGearPosition();
  message.gear_velocity = state.getGearVelocity();
  message.joint_position = state.getJointPosition();
  message.joint_velocity = state.getJointVelocity();
  message.joint_acceleration = state.getJointAcceleration();
  message.joint_torque = state.getJointTorque();
  message.imu.linear_acceleration.x = state.getImu().linearAcceleration_.x();
  message.imu.linear_acceleration.y = state.getImu().linearAcceleration_.y();
  message.imu.linear_acceleration.z = state.getImu().linearAcceleration_.z();
  message.imu.angular_velocity.x = state.getImu().angularVelocity_.x();
  message.imu.angular_velocity.y = state.getImu().angularVelocity_.y();
  message.imu.angular_velocity.z = state.getImu().angularVelocity_.z();
  message.gear_position_ticks = state.getGearPositionTicks();
  message.joint_position_ticks = state.getJointPositionTicks();
  message.temperature = state.getTemperature();
  message.voltage = state.getVoltage();
  message.timestamp = state.getTimestamp();
  message.desired_current_d = state.getDesiredCurrentD();
  message.measured_current_d = state.getMeasuredCurrentD();
  message.desired_current_q = state.getDesiredCurrentQ();
  message.measured_current_q = state.getMeasuredCurrentQ();
  message.alpha = state.getAlpha();
  message.beta = state.getBeta();
  message.measured_current_phase_u = state.getMeasuredCurrentPhaseU();
  message.measured_current_phase_v = state.getMeasuredCurrentPhaseV();
  message.measured_current_phase_w = state.getMeasuredCurrentPhaseW();
  message.measured_voltage_phase_u = state.getMeasuredVoltagePhaseU();
  message.measured_voltage_phase_v = state.getMeasuredVoltagePhaseV();
  message.measured_voltage_phase_w = state.getMeasuredVoltagePhaseW();
  message.duty_cycle_u = state.getDutyCycleU();
  message.duty_cycle_v = state.getDutyCycleV();
  message.duty_cycle_w = state.getDutyCycleW();
}

void readFromMessage(anydrive::StateExtended& state, const anydrive_msgs::StateExtended& message) {
  state.setStamp(any_measurements_ros::fromRos(message.header.stamp));
  state.setStatusword(anydrive::Statusword(message.statusword));
  state.setCurrent(message.current);
  state.setMotorPosition(message.motor_position);
  state.setMotorVelocity(message.motor_velocity);
  state.setGearPosition(message.gear_position);
  state.setGearVelocity(message.gear_velocity);
  state.setJointPosition(message.joint_position);
  state.setJointVelocity(message.joint_velocity);
  state.setJointAcceleration(message.joint_acceleration);
  state.setJointTorque(message.joint_torque);
  any_measurements::Imu imu;
  imu.linearAcceleration_.x() = message.imu.linear_acceleration.x;
  imu.linearAcceleration_.y() = message.imu.linear_acceleration.y;
  imu.linearAcceleration_.z() = message.imu.linear_acceleration.z;
  imu.angularVelocity_.x() = message.imu.angular_velocity.x;
  imu.angularVelocity_.y() = message.imu.angular_velocity.y;
  imu.angularVelocity_.z() = message.imu.angular_velocity.z;
  state.setImu(imu);
  state.setGearPositionTicks(message.gear_position_ticks);
  state.setJointPositionTicks(message.joint_position_ticks);
  state.setTemperature(message.temperature);
  state.setVoltage(message.voltage);
  state.setDesiredCurrentD(message.desired_current_d);
  state.setMeasuredCurrentD(message.measured_current_d);
  state.setDesiredCurrentQ(message.desired_current_q);
  state.setMeasuredCurrentQ(message.measured_current_q);
  state.setAlpha(message.alpha);
  state.setBeta(message.beta);
  state.setMeasuredCurrentPhaseU(message.measured_current_phase_u);
  state.setMeasuredCurrentPhaseV(message.measured_current_phase_v);
  state.setMeasuredCurrentPhaseW(message.measured_current_phase_w);
  state.setMeasuredVoltagePhaseU(message.measured_voltage_phase_u);
  state.setMeasuredVoltagePhaseV(message.measured_voltage_phase_v);
  state.setMeasuredVoltagePhaseW(message.measured_voltage_phase_w);
  state.setDutyCycleU(message.duty_cycle_u);
  state.setDutyCycleV(message.duty_cycle_v);
  state.setDutyCycleW(message.duty_cycle_w);
}

void writeToMessage(anydrive_msgs::Reading& message, const anydrive::Reading& reading) {
  writeToMessage(message.commanded, reading.getCommanded());
  writeToMessage(message.state, reading.getState());
}

void readFromMessage(anydrive::Reading& reading, const anydrive_msgs::Reading& message) {
  readFromMessage(reading.getCommanded(), message.commanded);
  readFromMessage(reading.getState(), message.state);
}

void writeToMessage(anydrive_msgs::ReadingExtended& message, const anydrive::ReadingExtended& reading) {
  writeToMessage(message.commanded, reading.getCommanded());
  writeToMessage(message.state, reading.getState());
}

void readFromMessage(anydrive::ReadingExtended& reading, const anydrive_msgs::ReadingExtended& message) {
  readFromMessage(reading.getCommanded(), message.commanded);
  readFromMessage(reading.getState(), message.state);
}

bool readConfigurationParameters(XmlRpc::XmlRpcValue& params, anydrive::configuration::Configuration& configuration) {
  if (params.hasMember("max_command_age")) {
    configuration.setMaxCommandAge(param_io::getMember<double>(params, "max_command_age"));
  }
  if (params.hasMember("auto_stage_last_command")) {
    configuration.setAutoStageLastCommand(param_io::getMember<bool>(params, "auto_stage_last_command"));
  }
  if (params.hasMember("set_reading_to_nan_on_disconnect")) {
    configuration.setSetReadingToNanOnDisconnect(param_io::getMember<bool>(params, "set_reading_to_nan_on_disconnect"));
  }
  if (params.hasMember("error_state_behavior")) {
    configuration.setErrorStateBehavior(param_io::getMember<int>(params, "error_state_behavior"));
  }
  if (params.hasMember("max_current")) {
    configuration.setMaxCurrent(param_io::getMember<double>(params, "max_current"));
  }
  if (params.hasMember("max_freeze_current")) {
    configuration.setMaxFreezeCurrent(param_io::getMember<double>(params, "max_freeze_current"));
  }
  if (params.hasMember("max_motor_velocity")) {
    configuration.setMaxMotorVelocity(param_io::getMember<double>(params, "max_motor_velocity"));
  }
  if (params.hasMember("max_joint_torque")) {
    configuration.setMaxJointTorque(param_io::getMember<double>(params, "max_joint_torque"));
  }
  if (params.hasMember("current_integrator_saturation")) {
    configuration.setCurrentIntegratorSaturation(param_io::getMember<double>(params, "current_integrator_saturation"));
  }
  if (params.hasMember("joint_torque_integrator_saturation")) {
    configuration.setJointTorqueIntegratorSaturation(param_io::getMember<double>(params, "joint_torque_integrator_saturation"));
  }
  if (params.hasMember("direction")) {
    configuration.setDirection(param_io::getMember<int>(params, "direction"));
  }
  if (params.hasMember("joint_position_limits")) {
    XmlRpc::XmlRpcValue pLimits = param_io::getMember<XmlRpc::XmlRpcValue>(params, "joint_position_limits");
    if (pLimits.hasMember("sdk")) {
      XmlRpc::XmlRpcValue p = param_io::getMember<XmlRpc::XmlRpcValue>(pLimits, "sdk");
      if (p.hasMember("min") && p.hasMember("max")) {
        configuration.setJointPositionLimitsSdk({param_io::getMember<double>(p, "min"), param_io::getMember<double>(p, "max")});
      }
    }
    if (pLimits.hasMember("soft")) {
      XmlRpc::XmlRpcValue p = param_io::getMember<XmlRpc::XmlRpcValue>(pLimits, "soft");
      if (p.hasMember("min") && p.hasMember("max")) {
        configuration.setJointPositionLimitsSoft({param_io::getMember<double>(p, "min"), param_io::getMember<double>(p, "max")});
      }
    }
    if (pLimits.hasMember("hard")) {
      XmlRpc::XmlRpcValue p = param_io::getMember<XmlRpc::XmlRpcValue>(pLimits, "hard");
      if (p.hasMember("min") && p.hasMember("max")) {
        configuration.setJointPositionLimitsHard({param_io::getMember<double>(p, "min"), param_io::getMember<double>(p, "max")});
      }
    }
  }
  if (params.hasMember("joint_position_configurations")) {
    XmlRpc::XmlRpcValue jointPosConfigParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "joint_position_configurations");
    for (auto& param : jointPosConfigParams) {
      if (param.second.hasMember("name") && param.second.hasMember("value")) {
        configuration.addJointPositionConfiguration(param_io::getMember<std::string>(param.second, "name"),
                                                    param_io::getMember<double>(param.second, "value"));
      }
    }
  }
  if (params.hasMember("imu")) {
    XmlRpc::XmlRpcValue imuParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "imu");
    if (imuParams.hasMember("enable")) {
      configuration.setImuEnable(param_io::getMember<bool>(imuParams, "enable"));
    }
    if (imuParams.hasMember("accelerometer_range")) {
      configuration.setImuAccelerometerRange(param_io::getMember<int>(imuParams, "accelerometer_range"));
    }
    if (imuParams.hasMember("gyroscope_range")) {
      configuration.setImuGyroscopeRange(param_io::getMember<int>(imuParams, "gyroscope_range"));
    }
  }
  if (params.hasMember("fan")) {
    XmlRpc::XmlRpcValue fanParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "fan");
    if (fanParams.hasMember("mode")) {
      configuration.setFanMode(param_io::getMember<int>(fanParams, "mode"));
    }
    if (fanParams.hasMember("intensity")) {
      configuration.setFanIntensity(param_io::getMember<int>(fanParams, "intensity"));
    }
    if (fanParams.hasMember("lower_temperature")) {
      configuration.setFanLowerTemperature(param_io::getMember<double>(fanParams, "lower_temperature"));
    }
    if (fanParams.hasMember("upper_temperature")) {
      configuration.setFanUpperTemperature(param_io::getMember<double>(fanParams, "upper_temperature"));
    }
  }
  if (params.hasMember("modes")) {
    XmlRpc::XmlRpcValue modesParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "modes");
    for (auto& param : modesParams) {
      if (param.second.hasMember("name") && param.second.hasMember("gains")) {
        std::string modeName = param_io::getMember<std::string>(param.second, "name");
        XmlRpc::XmlRpcValue p = param_io::getMember<XmlRpc::XmlRpcValue>(param.second, "gains");
        if (p.hasMember("p") && p.hasMember("i") && p.hasMember("d")) {
          anydrive::mode::ModeBasePtr mode = configuration.getMode(anydrive::mode::modeNameToEnum(modeName));
          if (!mode) {
            ANYDRIVE_ERROR("Mode name '" << modeName << "' does not exist, cannot set gains.")
            continue;
          }
          mode->setPidGains(anydrive::mode::PidGainsF(param_io::getMember<double>(p, "p"), param_io::getMember<double>(p, "i"),
                                                      param_io::getMember<double>(p, "d")));
        }
      }
    }
  }
  if (params.hasMember("goal_states")) {
    XmlRpc::XmlRpcValue goalStateParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "goal_states");
    if (goalStateParams.hasMember("startup")) {
      configuration.setGoalStateEnumStartup(anydrive::fsm::stateNameToEnum(param_io::getMember<std::string>(goalStateParams, "startup")));
    }
    if (goalStateParams.hasMember("shutdown")) {
      configuration.setGoalStateEnumShutdown(anydrive::fsm::stateNameToEnum(param_io::getMember<std::string>(goalStateParams, "shutdown")));
    }
  }
  if (params.hasMember("gear_joint_velocity_filter")) {
    XmlRpc::XmlRpcValue filterParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "gear_joint_velocity_filter");
    if (filterParams.hasMember("type")) {
      configuration.setGearJointVelocityFilterType(param_io::getMember<int>(filterParams, "type"));
    }
    if (filterParams.hasMember("kf_noise_variance")) {
      configuration.setGearJointVelocityKfNoiseVariance(param_io::getMember<double>(filterParams, "kf_noise_variance"));
    }
    if (filterParams.hasMember("kf_lambda_2")) {
      configuration.setGearJointVelocityKfLambda2(param_io::getMember<double>(filterParams, "kf_lambda_2"));
    }
    if (filterParams.hasMember("kf_gamma")) {
      configuration.setGearJointVelocityKfGamma(param_io::getMember<double>(filterParams, "kf_gamma"));
    }
    if (filterParams.hasMember("ema_alpha")) {
      configuration.setGearJointVelocityEmaAlpha(param_io::getMember<double>(filterParams, "ema_alpha"));
    }
  }
  if (params.hasMember("joint_velocity_filter_for_acceleration")) {
    XmlRpc::XmlRpcValue filterParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "joint_velocity_filter_for_acceleration");
    if (filterParams.hasMember("type")) {
      configuration.setJointVelocityForAccelerationFilterType(param_io::getMember<int>(filterParams, "type"));
    }
    if (filterParams.hasMember("kf_noise_variance")) {
      configuration.setJointVelocityForAccelerationKfNoiseVariance(param_io::getMember<double>(filterParams, "kf_noise_variance"));
    }
    if (filterParams.hasMember("kf_lambda_2")) {
      configuration.setJointVelocityForAccelerationKfLambda2(param_io::getMember<double>(filterParams, "kf_lambda_2"));
    }
    if (filterParams.hasMember("kf_gamma")) {
      configuration.setJointVelocityForAccelerationKfGamma(param_io::getMember<double>(filterParams, "kf_gamma"));
    }
    if (filterParams.hasMember("ema_alpha")) {
      configuration.setJointVelocityForAccelerationEmaAlpha(param_io::getMember<double>(filterParams, "ema_alpha"));
    }
  }
  if (params.hasMember("joint_acceleration_filter")) {
    XmlRpc::XmlRpcValue filterParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "joint_acceleration_filter");
    if (filterParams.hasMember("type")) {
      configuration.setJointAccelerationFilterType(param_io::getMember<int>(filterParams, "type"));
    }
    if (filterParams.hasMember("kf_noise_variance")) {
      configuration.setJointAccelerationKfNoiseVariance(param_io::getMember<double>(filterParams, "kf_noise_variance"));
    }
    if (filterParams.hasMember("kf_lambda_2")) {
      configuration.setJointAccelerationKfLambda2(param_io::getMember<double>(filterParams, "kf_lambda_2"));
    }
    if (filterParams.hasMember("kf_gamma")) {
      configuration.setJointAccelerationKfGamma(param_io::getMember<double>(filterParams, "kf_gamma"));
    }
    if (filterParams.hasMember("ema_alpha")) {
      configuration.setJointAccelerationEmaAlpha(param_io::getMember<double>(filterParams, "ema_alpha"));
    }
  }
  return true;
}

bool readAnydriveSetupParameters(XmlRpc::XmlRpcValue& params, anydrive::setup::AnydrivePtr& anydrivePtr) {
  return readConfigurationParameters(params, anydrivePtr->configuration_);
}

}  // namespace anydrive_ros
