/*!
 * @file    SeabotGazeboPlugin.tpp
 * @author  Markus Staeuble
 * @date    Dec, 2017
 */

#include <seabot_gazebo_plugin/SeabotGazeboPlugin.hpp>

namespace gazebo {

using namespace param_io;

template <typename ConcreteDescription_>
SeabotGazeboPlugin<ConcreteDescription_>::SeabotGazeboPlugin(const std::string& robotName)
    : AnybotGazeboPlugin<ConcreteDescription_>(robotName), receiveMaxLockTime_{std::chrono::microseconds{200}} {}

template <typename ConcreteDescription_>
void SeabotGazeboPlugin<ConcreteDescription_>::Reset() {
  // Reset the base class.
  AnybotGazeboPlugin<ConcreteDescription_>::Reset();

  // Reset the joint structures.
  resetJointStructures();
}

template <typename ConcreteDescription_>
void SeabotGazeboPlugin<ConcreteDescription_>::initJointStructures() {
  // Init the joint structures.
  for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorName = actuatorKey.getName();

    auto joint = this->robotDescriptionUrdfModel_.getJoint(actuatorName);
    if (!joint) {
      ROS_ERROR_STREAM_NAMED("gazebo_ros_control", "Joint named '" << actuatorName << "' does not exist in the URDF model.");
      return;
    }
    auto simJoint = this->model_->GetJoint(actuatorName);
    if (!simJoint) {
      ROS_ERROR_STREAM_NAMED("gazebo_ros_control", "Joint named '" << actuatorName << "' does not exist in Gazebo.");
      return;
    }

    // Initialize first order low-pass filters for joint velocity estimation
    this->jointVelocityFilter_[actuatorEnum] = basic_filters::FirstOrderFilterD(1.0, this->publishingTimeStep_ / 100.0, 1.0);

    // Set joint position to default initial position
#if GAZEBO_MAJOR_VERSION >= 7
    simJoint->SetPosition(0, this->jointPositionsDefault_[actuatorEnum]);
#else
    simJoint->SetAngle(0, this->jointPositionsDefault_[actuatorEnum]);
#endif

    this->simJoints_[actuatorEnum] = simJoint;
    this->jointTypes_[actuatorEnum] = joint->type;
    this->jointPositionsReset_[actuatorEnum] = this->jointPositionsDefault_[actuatorEnum];
    this->jointPositionLimitsLow_[actuatorEnum] = joint->limits->lower;
    this->jointPositionLimitsHigh_[actuatorEnum] = joint->limits->upper;
    this->jointTorqueLimits_[actuatorEnum] = joint->limits->effort;
    this->jointVelocityLimits_[actuatorEnum] = joint->limits->velocity;
  }

  // Initialize actuator models.
  romo_std::fillContainer<ConcreteDescription>(actuatorModel_);
  setActuatorGains();

  // Initialize the default position actuator commands.
  romo_std::fillContainer<ConcreteDescription>(actuatorDefaultPositionCommands_);
  for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();

    auto& command = actuatorDefaultPositionCommands_[actuatorEnum];
    command.setMode(series_elastic_actuator::SeActuatorCommand::MODE_FREEZE);
    command.setJointPosition(this->jointPositionsDefault_[actuatorEnum]);
  }

  resetJointStructures();
}

template <typename ConcreteDescription_>
void SeabotGazeboPlugin<ConcreteDescription_>::resetJointStructures() {
  // Reset readings and commands.
  romo_std::fillContainer<ConcreteDescription>(actuatorReadings_);
  actuatorCommands_ = actuatorDefaultPositionCommands_;

  // Reset models.
  for (const auto actuatorKey : ConcreteDescription::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();

    actuatorReadings_[actuatorEnum].getState().setJointPosition(actuatorCommands_[actuatorEnum].getJointPosition());
    actuatorReadings_[actuatorEnum].getCommanded() = actuatorCommands_[actuatorEnum];
    actuatorModel_[actuatorEnum].initialize(actuatorCommands_[actuatorEnum], actuatorReadings_[actuatorEnum].getState(), 0.0);
  }
}

template <typename ConcreteDescription_>
void SeabotGazeboPlugin<ConcreteDescription_>::setActuatorGains() {
  for (auto& item : actuatorModel_) {
    const double jointPositionPGain = param(*this->nodeHandle_, "joint_states/gains/jointPositionPGain", 100.0);
    const double jointPositionDGain = param(*this->nodeHandle_, "joint_states/gains/jointPositionDGain", 2.0);
    const double jointPositionVelocityTorquePGain = param(*this->nodeHandle_, "joint_states/gains/jointPositionVelocityTorquePGain", 30.0);
    const double jointPositionVelocityTorqueIGain = param(*this->nodeHandle_, "joint_states/gains/jointPositionVelocityTorqueIGain", 0.0);
    const double jointPositionVelocityTorqueDGain = param(*this->nodeHandle_, "joint_states/gains/jointPositionVelocityTorqueDGain", 0.0);

    item.getControllerFreeze().setJointPositionPGain(jointPositionPGain);
    item.getControllerFreeze().setJointPositionDGain(jointPositionDGain);
    item.getControllerJointPosition().setJointPositionPGain(jointPositionPGain);
    item.getControllerJointPosition().setJointPositionDGain(jointPositionDGain);
    item.getControllerJointPositionVelocity().setJointPositionPGain(jointPositionPGain);
    item.getControllerJointPositionVelocity().setJointPositionDGain(jointPositionDGain);
    item.getControllerJointPositionVelocityTorque().setJointPositionPGain(jointPositionVelocityTorquePGain);
    item.getControllerJointPositionVelocityTorque().setJointPositionIGain(jointPositionVelocityTorqueIGain);
    item.getControllerJointPositionVelocityTorque().setJointPositionDGain(jointPositionVelocityTorqueDGain);
  }
}

template <typename ConcreteDescription_>
void SeabotGazeboPlugin<ConcreteDescription_>::publishActuatorReadings() {
  actuatorReadingsPublisher_->publish(actuatorReadings_, std::chrono::microseconds(200));
  this->publishActuatorReadingsDerived();
}

template <typename ConcreteDescription_>
double SeabotGazeboPlugin<ConcreteDescription_>::computeActuatorCommand(const ActuatorEnum& actuatorEnum) {
  // Generate control references for each joint.
  if (!actuatorModel_[actuatorEnum].advance(actuatorReadings_[actuatorEnum], actuatorCommands_[actuatorEnum], this->updateSimTimeStep_)) {
    MELO_WARN_STREAM("[" << this->robotName_ << "GazeboPlugin::writeSimulation] Could not advance actuator "
                         << ConcreteDescription::mapKeyEnumToKeyName(actuatorEnum));
  }

  // Update commands in readings.
  actuatorReadings_[actuatorEnum].setCommanded(actuatorCommands_[actuatorEnum]);
  return actuatorReadings_[actuatorEnum].getState().getJointTorque();
}

template <typename ConcreteDescription_>
void SeabotGazeboPlugin<ConcreteDescription_>::updateActuatorReading(const ActuatorEnum& actuatorEnum, const double jointPosition,
                                                                     const double jointVelocity, const double jointAcceleration,
                                                                     const double jointTorque) {
  series_elastic_actuator::SeActuatorState state;
  state.setJointPosition(jointPosition);
  state.setJointVelocity(jointVelocity);
  state.setJointAcceleration(jointAcceleration);
  state.setJointTorque(jointTorque);
  state.setStamp(this->getTime());
  actuatorReadings_[actuatorEnum].setState(state);
}

template <typename ConcreteDescription_>
void SeabotGazeboPlugin<ConcreteDescription_>::actuatorCommandsCallback(const ActuatorCommandsShm& msg) {
  actuatorCommands_ = msg;
}

template <typename ConcreteDescription_>
void SeabotGazeboPlugin<ConcreteDescription_>::receiveMessages() {
  actuatorCommandsSubscriber_->receive(receiveMaxLockTime_);
}

}  // namespace gazebo