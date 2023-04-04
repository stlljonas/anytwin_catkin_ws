// series elastic actuator anydrive
#include "series_elastic_actuator_anydrive/conversions.hpp"


namespace series_elastic_actuator_anydrive {


void anydriveToSeActuator(SeActuatorMode& seActuatorMode, const AnydriveMode anydriveMode) {
  switch (anydriveMode) {
    case AnydriveMode::Current: seActuatorMode = SeActuatorMode::MODE_CURRENT; return;
    case AnydriveMode::Disable: seActuatorMode = SeActuatorMode::MODE_DISABLE; return;
    case AnydriveMode::Freeze: seActuatorMode = SeActuatorMode::MODE_FREEZE; return;
    case AnydriveMode::GearPosition: seActuatorMode = SeActuatorMode::MODE_GEAR_POSITION; return;
    case AnydriveMode::GearVelocity: seActuatorMode = SeActuatorMode::MODE_GEAR_VELOCITY; return;
    case AnydriveMode::JointPosition: seActuatorMode = SeActuatorMode::MODE_JOINT_POSITION; return;
    case AnydriveMode::JointPositionVelocity: seActuatorMode = SeActuatorMode::MODE_JOINT_POSITION_VELOCITY; return;
    case AnydriveMode::JointPositionVelocityTorque: seActuatorMode = SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE; return;
    case AnydriveMode::JointPositionVelocityTorquePidGains: seActuatorMode = SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS; return;
    case AnydriveMode::JointTorque: seActuatorMode = SeActuatorMode::MODE_JOINT_TORQUE; return;
    case AnydriveMode::JointVelocity: seActuatorMode = SeActuatorMode::MODE_JOINT_VELOCITY; return;
    case AnydriveMode::MotorPosition: seActuatorMode = SeActuatorMode::MODE_MOTOR_POSITION; return;
    case AnydriveMode::MotorVelocity: seActuatorMode = SeActuatorMode::MODE_MOTOR_VELOCITY; return;
    default: seActuatorMode = SeActuatorMode::MODE_FREEZE; return;
  }
}

void seActuatorToAnydrive(AnydriveMode& anydriveMode, const SeActuatorMode seActuatorMode) {
  switch (seActuatorMode) {
    case SeActuatorMode::MODE_CURRENT: anydriveMode = AnydriveMode::Current; return;
    case SeActuatorMode::MODE_DISABLE: anydriveMode = AnydriveMode::Disable; return;
    case SeActuatorMode::MODE_FREEZE: anydriveMode = AnydriveMode::Freeze; return;
    case SeActuatorMode::MODE_GEAR_POSITION: anydriveMode = AnydriveMode::GearPosition; return;
    case SeActuatorMode::MODE_GEAR_VELOCITY: anydriveMode = AnydriveMode::GearVelocity; return;
    case SeActuatorMode::MODE_JOINT_POSITION: anydriveMode = AnydriveMode::JointPosition; return;
    case SeActuatorMode::MODE_JOINT_POSITION_VELOCITY: anydriveMode = AnydriveMode::JointPositionVelocity; return;
    case SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE: anydriveMode = AnydriveMode::JointPositionVelocityTorque; return;
    case SeActuatorMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS: anydriveMode = AnydriveMode::JointPositionVelocityTorquePidGains; return;
    case SeActuatorMode::MODE_JOINT_TORQUE: anydriveMode = AnydriveMode::JointTorque; return;
    case SeActuatorMode::MODE_JOINT_VELOCITY: anydriveMode = AnydriveMode::JointVelocity; return;
    case SeActuatorMode::MODE_MOTOR_POSITION: anydriveMode = AnydriveMode::MotorPosition; return;
    case SeActuatorMode::MODE_MOTOR_VELOCITY: anydriveMode = AnydriveMode::MotorVelocity; return;
    default: anydriveMode = AnydriveMode::Freeze; return;
  }
}

void anydriveToSeActuator(SeActuatorCommand& seActuatorCommand, const AnydriveCommand& anydriveCommand) {
  seActuatorCommand.setStamp(anydriveCommand.getStamp());
  SeActuatorMode modeEnum = SeActuatorMode::MODE_FREEZE;
  anydriveToSeActuator(modeEnum, anydriveCommand.getModeEnum());
  seActuatorCommand.setModeEnum(modeEnum);
  seActuatorCommand.setCurrent(anydriveCommand.getCurrent());
  seActuatorCommand.setMotorPosition(anydriveCommand.getMotorPosition());
  seActuatorCommand.setMotorVelocity(anydriveCommand.getMotorVelocity());
  seActuatorCommand.setGearPosition(anydriveCommand.getGearPosition());
  seActuatorCommand.setGearVelocity(anydriveCommand.getGearVelocity());
  seActuatorCommand.setJointPosition(anydriveCommand.getJointPosition());
  seActuatorCommand.setJointVelocity(anydriveCommand.getJointVelocity());
  seActuatorCommand.setJointTorque(anydriveCommand.getJointTorque());
  seActuatorCommand.setPidGainsP(anydriveCommand.getPidGains().getP());
  seActuatorCommand.setPidGainsI(anydriveCommand.getPidGains().getI());
  seActuatorCommand.setPidGainsD(anydriveCommand.getPidGains().getD());
}

void seActuatorToAnydrive(AnydriveCommand& anydriveCommand, const SeActuatorCommand& seActuatorCommand) {
  anydriveCommand.setStamp(seActuatorCommand.getStamp());
  AnydriveMode modeEnum = AnydriveMode::Freeze;
  seActuatorToAnydrive(modeEnum, seActuatorCommand.getModeEnum());
  anydriveCommand.setModeEnum(modeEnum);
  anydriveCommand.setCurrent(seActuatorCommand.getCurrent());
  anydriveCommand.setMotorPosition(seActuatorCommand.getMotorPosition());
  anydriveCommand.setMotorVelocity(seActuatorCommand.getMotorVelocity());
  anydriveCommand.setGearPosition(seActuatorCommand.getGearPosition());
  anydriveCommand.setGearVelocity(seActuatorCommand.getGearVelocity());
  anydriveCommand.setJointPosition(seActuatorCommand.getJointPosition());
  anydriveCommand.setJointVelocity(seActuatorCommand.getJointVelocity());
  anydriveCommand.setJointTorque(seActuatorCommand.getJointTorque());
  anydriveCommand.getPidGains().setP(seActuatorCommand.getPidGainsP());
  anydriveCommand.getPidGains().setI(seActuatorCommand.getPidGainsI());
  anydriveCommand.getPidGains().setD(seActuatorCommand.getPidGainsD());
}

void anydriveToSeActuator(SeActuatorState& seActuatorState, const AnydriveState& anydriveState) {
  seActuatorState.setStamp(anydriveState.getStamp());
  seActuatorState.setStatusword(anydriveState.getStatusword().getData());
  seActuatorState.setCurrent(anydriveState.getCurrent());
  seActuatorState.setGearPosition(anydriveState.getGearPosition());
  seActuatorState.setGearVelocity(anydriveState.getGearVelocity());
  seActuatorState.setJointPosition(anydriveState.getJointPosition());
  seActuatorState.setJointVelocity(anydriveState.getJointVelocity());
  seActuatorState.setJointAcceleration(anydriveState.getJointAcceleration());
  seActuatorState.setJointTorque(anydriveState.getJointTorque());
  seActuatorState.setImu(anydriveState.getImu());
}

void seActuatorToAnydrive(AnydriveState& anydriveState, const SeActuatorState& seActuatorState) {
  anydriveState.setStamp(seActuatorState.getStamp());
  anydriveState.setStatusword(anydrive::Statusword(seActuatorState.getStatusword()));
  anydriveState.setCurrent(seActuatorState.getCurrent());
  anydriveState.setGearPosition(seActuatorState.getGearPosition());
  anydriveState.setGearVelocity(seActuatorState.getGearVelocity());
  anydriveState.setJointPosition(seActuatorState.getJointPosition());
  anydriveState.setJointVelocity(seActuatorState.getJointVelocity());
  anydriveState.setJointAcceleration(seActuatorState.getJointAcceleration());
  anydriveState.setJointTorque(seActuatorState.getJointTorque());
}

void anydriveToSeActuator(SeActuatorStateExtended& seActuatorStateExtended, const AnydriveStateExtended& anydriveStateExtended) {
  anydriveToSeActuator(static_cast<SeActuatorState&>(seActuatorStateExtended), static_cast<const AnydriveState&>(anydriveStateExtended));
  seActuatorStateExtended.setMotorPosition(anydriveStateExtended.getMotorPosition());
  seActuatorStateExtended.setMotorVelocity(anydriveStateExtended.getMotorVelocity());
  seActuatorStateExtended.setGearPositionTicks(anydriveStateExtended.getGearPositionTicks());
  seActuatorStateExtended.setJointPositionTicks(anydriveStateExtended.getJointPositionTicks());
  seActuatorStateExtended.setTemperature(anydriveStateExtended.getTemperature());
  seActuatorStateExtended.setVoltage(anydriveStateExtended.getVoltage());
  seActuatorStateExtended.setTimestamp(anydriveStateExtended.getTimestamp());
  seActuatorStateExtended.setDesiredCurrentD(anydriveStateExtended.getDesiredCurrentD());
  seActuatorStateExtended.setMeasuredCurrentD(anydriveStateExtended.getMeasuredCurrentD());
  seActuatorStateExtended.setDesiredCurrentQ(anydriveStateExtended.getDesiredCurrentQ());
  seActuatorStateExtended.setMeasuredCurrentQ(anydriveStateExtended.getMeasuredCurrentQ());
  seActuatorStateExtended.setMeasuredCurrentPhaseU(anydriveStateExtended.getMeasuredCurrentPhaseU());
  seActuatorStateExtended.setMeasuredVoltagePhaseU(anydriveStateExtended.getMeasuredVoltagePhaseU());
  seActuatorStateExtended.setMeasuredCurrentPhaseV(anydriveStateExtended.getMeasuredCurrentPhaseV());
  seActuatorStateExtended.setMeasuredVoltagePhaseV(anydriveStateExtended.getMeasuredVoltagePhaseV());
  seActuatorStateExtended.setMeasuredCurrentPhaseW(anydriveStateExtended.getMeasuredCurrentPhaseW());
  seActuatorStateExtended.setMeasuredVoltagePhaseW(anydriveStateExtended.getMeasuredVoltagePhaseW());
}

void seActuatorToAnydrive(AnydriveStateExtended& anydriveStateExtended, const SeActuatorStateExtended& seActuatorStateExtended) {
  seActuatorToAnydrive(static_cast<AnydriveState&>(anydriveStateExtended), static_cast<const SeActuatorState&>(seActuatorStateExtended));
  anydriveStateExtended.setMotorPosition(seActuatorStateExtended.getMotorPosition());
  anydriveStateExtended.setMotorVelocity(seActuatorStateExtended.getMotorVelocity());
  anydriveStateExtended.setGearPositionTicks(seActuatorStateExtended.getGearPositionTicks());
  anydriveStateExtended.setJointPositionTicks(seActuatorStateExtended.getJointPositionTicks());
  anydriveStateExtended.setTemperature(seActuatorStateExtended.getTemperature());
  anydriveStateExtended.setVoltage(seActuatorStateExtended.getVoltage());
  anydriveStateExtended.setTimestamp(seActuatorStateExtended.getTimestamp());
  anydriveStateExtended.setDesiredCurrentD(seActuatorStateExtended.getDesiredCurrentD());
  anydriveStateExtended.setMeasuredCurrentD(seActuatorStateExtended.getMeasuredCurrentD());
  anydriveStateExtended.setDesiredCurrentQ(seActuatorStateExtended.getDesiredCurrentQ());
  anydriveStateExtended.setMeasuredCurrentQ(seActuatorStateExtended.getMeasuredCurrentQ());
  anydriveStateExtended.setMeasuredCurrentPhaseU(seActuatorStateExtended.getMeasuredCurrentPhaseU());
  anydriveStateExtended.setMeasuredVoltagePhaseU(seActuatorStateExtended.getMeasuredVoltagePhaseU());
  anydriveStateExtended.setMeasuredCurrentPhaseV(seActuatorStateExtended.getMeasuredCurrentPhaseV());
  anydriveStateExtended.setMeasuredVoltagePhaseV(seActuatorStateExtended.getMeasuredVoltagePhaseV());
  anydriveStateExtended.setMeasuredCurrentPhaseW(seActuatorStateExtended.getMeasuredCurrentPhaseW());
  anydriveStateExtended.setMeasuredVoltagePhaseW(seActuatorStateExtended.getMeasuredVoltagePhaseW());
}

void anydriveToSeActuator(SeActuatorReading& seActuatorReading, const AnydriveReading& anydriveReading) {
  anydriveToSeActuator(seActuatorReading.getCommanded(), anydriveReading.getCommanded());
  anydriveToSeActuator(seActuatorReading.getState(), anydriveReading.getState());
}

void seActuatorToAnydrive(AnydriveReading& anydriveReading, const SeActuatorReading& seActuatorReading) {
  seActuatorToAnydrive(anydriveReading.getCommanded(), seActuatorReading.getCommanded());
  seActuatorToAnydrive(anydriveReading.getState(), seActuatorReading.getState());
}

void anydriveToSeActuator(SeActuatorReadingExtended& seActuatorReadingExtended, const AnydriveReadingExtended& anydriveReadingExtended) {
  anydriveToSeActuator(seActuatorReadingExtended.getCommanded(), anydriveReadingExtended.getCommanded());
  anydriveToSeActuator(seActuatorReadingExtended.getState(), anydriveReadingExtended.getState());
}

void seActuatorToAnydrive(AnydriveReadingExtended& anydriveReadingExtended, const SeActuatorReadingExtended& seActuatorReadingExtended) {
  seActuatorToAnydrive(anydriveReadingExtended.getCommanded(), seActuatorReadingExtended.getCommanded());
  seActuatorToAnydrive(anydriveReadingExtended.getState(), seActuatorReadingExtended.getState());
}


} // series_elastic_actuator_anydrive
