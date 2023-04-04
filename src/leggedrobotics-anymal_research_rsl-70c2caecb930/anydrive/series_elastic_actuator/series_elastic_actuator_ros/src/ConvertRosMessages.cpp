/*!
 * @file    ConvertRosMessages.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */


// any measurements ros
#include <any_measurements_ros/ConvertRosMessages.hpp>

// series elastic actuator ros
#include "series_elastic_actuator_ros/ConvertRosMessages.hpp"


namespace series_elastic_actuator_ros {


void ConvertRosMessages::writeToMessage(series_elastic_actuator_msgs::SeActuatorCommand& message, const series_elastic_actuator::SeActuatorCommand& command) {
  message.header.stamp = any_measurements_ros::toRos(command.getStamp());
  message.mode = command.getMode();
  message.current = command.getCurrent();
  switch (command.getModeEnum()) {
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_MOTOR_POSITION:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_MOTOR_VELOCITY:
      message.position = command.getMotorPosition();
      message.velocity = command.getMotorVelocity();
      break;
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_GEAR_POSITION:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_GEAR_VELOCITY:
      message.position = command.getGearPosition();
      message.velocity = command.getGearVelocity();
      break;
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_POSITION:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_VELOCITY:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_POSITION_VELOCITY:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_POSITION_VELOCITY_TORQUE:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_TORQUE:
      message.position = command.getJointPosition();
      message.velocity = command.getJointVelocity();
      break;
    default:
      break;
  }
  message.joint_torque = command.getJointTorque();
  message.pid_gains_p = command.getPidGainsP();
  message.pid_gains_i = command.getPidGainsI();
  message.pid_gains_d = command.getPidGainsD();
}

void ConvertRosMessages::readFromMessage(series_elastic_actuator::SeActuatorCommand& command, const series_elastic_actuator_msgs::SeActuatorCommand& message) {
  command.setStamp(any_measurements_ros::fromRos(message.header.stamp));
  command.setMode(message.mode);
  command.setCurrent(message.current);
  switch (command.getModeEnum()) {
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_MOTOR_POSITION:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_MOTOR_VELOCITY:
      command.setMotorPosition(message.position);
      command.setMotorVelocity(message.velocity);
      break;
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_GEAR_POSITION:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_GEAR_VELOCITY:
      command.setGearPosition(message.position);
      command.setGearVelocity(message.velocity);
      break;
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_POSITION:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_VELOCITY:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_POSITION_VELOCITY:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_POSITION_VELOCITY_TORQUE:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS:
    case series_elastic_actuator_msgs::SeActuatorCommand::MODE_JOINT_TORQUE:
      command.setJointPosition(message.position);
      command.setJointVelocity(message.velocity);
      break;
    default:
      break;
  }
  command.setJointTorque(message.joint_torque);
  command.setPidGainsP(message.pid_gains_p);
  command.setPidGainsI(message.pid_gains_i);
  command.setPidGainsD(message.pid_gains_d);
}

void ConvertRosMessages::writeToMessage(series_elastic_actuator_msgs::SeActuatorState& message, const series_elastic_actuator::SeActuatorState& state) {
  message.header.stamp = any_measurements_ros::toRos(state.getStamp());
  message.statusword = state.getStatusword();
  message.current = state.getCurrent();
  message.gear_position = state.getGearPosition();
  message.gear_velocity = state.getGearVelocity();
  message.joint_position = state.getJointPosition();
  message.joint_velocity = state.getJointVelocity();
  message.joint_acceleration = state.getJointAcceleration();
  message.joint_torque = state.getJointTorque();
  message.imu = any_measurements_ros::toRos(state.getImu());
}

void ConvertRosMessages::readFromMessage(series_elastic_actuator::SeActuatorState& state, const series_elastic_actuator_msgs::SeActuatorState& message) {
  state.setStamp(any_measurements_ros::fromRos(message.header.stamp));
  state.setStatusword(message.statusword);
  state.setCurrent(message.current);
  state.setGearPosition(message.gear_position);
  state.setGearVelocity(message.gear_velocity);
  state.setJointPosition(message.joint_position);
  state.setJointVelocity(message.joint_velocity);
  state.setJointAcceleration(message.joint_acceleration);
  state.setJointTorque(message.joint_torque);
  state.setImu(any_measurements_ros::fromRos(message.imu));
}

void ConvertRosMessages::writeToMessage(series_elastic_actuator_msgs::SeActuatorReading& message, const series_elastic_actuator::SeActuatorReading& reading) {
  message.header.stamp = any_measurements_ros::toRos(reading.getState().getStamp());
  writeToMessage(message.commanded, reading.getCommanded());
  writeToMessage(message.state, reading.getState());
}

void ConvertRosMessages::readFromMessage(series_elastic_actuator::SeActuatorReading& reading, const series_elastic_actuator_msgs::SeActuatorReading& message) {
  readFromMessage(reading.getCommanded(), message.commanded);
  readFromMessage(reading.getState(), message.state);
}

void ConvertRosMessages::writeToMessage(series_elastic_actuator_msgs::SeActuatorStateExtended& message, const series_elastic_actuator::SeActuatorStateExtended& state) {
  message.header.stamp = any_measurements_ros::toRos(state.getStamp());
  message.statusword = state.getStatusword();
  message.current = state.getCurrent();
  message.motor_position = state.getMotorPosition();
  message.motor_velocity = state.getMotorVelocity();
  message.gear_position = state.getGearPosition();
  message.gear_velocity = state.getGearVelocity();
  message.joint_position = state.getJointPosition();
  message.joint_velocity = state.getJointVelocity();
  message.joint_acceleration = state.getJointAcceleration();
  message.joint_torque = state.getJointTorque();
  message.imu = any_measurements_ros::toRos(state.getImu());
  message.gear_position_ticks = state.getGearPositionTicks();
  message.joint_position_ticks = state.getJointPositionTicks();
  message.temperature = state.getTemperature();
  message.voltage = state.getVoltage();
  message.timestamp = state.getTimestamp();
  message.desired_current_d = state.getDesiredCurrentD();
  message.measured_current_d = state.getMeasuredCurrentD();
  message.desired_current_q = state.getDesiredCurrentQ();
  message.measured_current_q = state.getMeasuredCurrentQ();
  message.measured_current_phase_u = state.getMeasuredCurrentPhaseU();
  message.measured_current_phase_v = state.getMeasuredCurrentPhaseV();
  message.measured_current_phase_w = state.getMeasuredCurrentPhaseW();
  message.measured_voltage_phase_u = state.getMeasuredVoltagePhaseU();
  message.measured_voltage_phase_v = state.getMeasuredVoltagePhaseV();
  message.measured_voltage_phase_w = state.getMeasuredVoltagePhaseW();
}

void ConvertRosMessages::readFromMessage(series_elastic_actuator::SeActuatorStateExtended& state, const series_elastic_actuator_msgs::SeActuatorStateExtended& message) {
  state.setStamp(any_measurements_ros::fromRos(message.header.stamp));
  state.setStatusword(message.statusword);
  state.setCurrent(message.current);
  state.setMotorPosition(message.motor_position);
  state.setMotorVelocity(message.motor_velocity);
  state.setGearPosition(message.gear_position);
  state.setGearVelocity(message.gear_velocity);
  state.setJointPosition(message.joint_position);
  state.setJointVelocity(message.joint_velocity);
  state.setJointAcceleration(message.joint_acceleration);
  state.setJointTorque(message.joint_torque);
  state.setImu(any_measurements_ros::fromRos(message.imu));
  state.setGearPositionTicks(message.gear_position_ticks);
  state.setJointPositionTicks(message.joint_position_ticks);
  state.setTemperature(message.temperature);
  state.setVoltage(message.voltage);
  state.setDesiredCurrentD(message.desired_current_d);
  state.setMeasuredCurrentD(message.measured_current_d);
  state.setDesiredCurrentQ(message.desired_current_q);
  state.setMeasuredCurrentQ(message.measured_current_q);
  state.setMeasuredCurrentPhaseU(message.measured_current_phase_u);
  state.setMeasuredCurrentPhaseV(message.measured_current_phase_v);
  state.setMeasuredCurrentPhaseW(message.measured_current_phase_w);
  state.setMeasuredVoltagePhaseU(message.measured_voltage_phase_u);
  state.setMeasuredVoltagePhaseV(message.measured_voltage_phase_v);
  state.setMeasuredVoltagePhaseW(message.measured_voltage_phase_w);
}

void ConvertRosMessages::writeToMessage(series_elastic_actuator_msgs::SeActuatorReadingExtended& message, const series_elastic_actuator::SeActuatorReadingExtended& reading) {
  writeToMessage(message.commanded, reading.getCommanded());
  writeToMessage(message.state, reading.getState());
}

void ConvertRosMessages::readFromMessage(series_elastic_actuator::SeActuatorReadingExtended& reading, const series_elastic_actuator_msgs::SeActuatorReadingExtended& message) {
  readFromMessage(reading.getCommanded(), message.commanded);
  readFromMessage(reading.getState(), message.state);
}


} // series_elastic_actuator_ros
