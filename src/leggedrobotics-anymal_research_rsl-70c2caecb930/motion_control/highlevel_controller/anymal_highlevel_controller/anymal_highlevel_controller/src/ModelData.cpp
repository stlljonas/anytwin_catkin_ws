/*!
 * @file    ModelData.cpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Oct, 2014
 */

#include "anymal_highlevel_controller/ModelData.hpp"

#include <any_measurements_ros/ConversionTraits.hpp>

// anymal model
#include <anymal_model/LimitsAnymal.hpp>

#include <message_logger/message_logger.hpp>
#include <signal_logger/signal_logger.hpp>

namespace anymal_highlevel_controller {

ModelData::ModelData():
  anymalModelMeasured_(),
  anymalModelDesired_(),
  state_(new anymal_roco::RocoState()),
  command_(new anymal_roco::RocoCommand()),
  mutexState_(new boost::shared_mutex()),
  mutexCommand_(new boost::shared_mutex())
{

}

std::shared_ptr<anymal_roco::RocoState>& ModelData::getState() {
  return state_;
}

const std::shared_ptr<anymal_roco::RocoState>& ModelData::getState() const {
  return state_;
}

std::shared_ptr<boost::shared_mutex>& ModelData::getStateMutex() {
  return mutexState_;
}

std::shared_ptr<anymal_roco::RocoCommand>& ModelData::getCommand() {
  return command_;
}

const std::shared_ptr<anymal_roco::RocoCommand>& ModelData::getCommand() const {
  return command_;
}

std::shared_ptr<boost::shared_mutex>& ModelData::getCommandMutex() {
  return mutexCommand_;
}

void ModelData::init(double dt, bool isRealRobot, const std::string& urdfDescription) {
  anymalModelMeasured_.reset(new anymal_model::AnymalModel(dt));
  anymalModelDesired_.reset(new anymal_model::AnymalModel(dt));

  /* initialize model from URDF decription */
  if(!anymalModelMeasured_->initializeFromUrdf(urdfDescription)) {
    MELO_ERROR("[Model::initializeForController] Could not initialize anymal model from urdf description!");
  }

  /* initialize the desired model from URDF decription */
  if(!anymalModelDesired_->initializeFromUrdf(urdfDescription)) {
    MELO_ERROR("[Model::initializeForController] Could not initialize desired anymal model from urdf description!");
  }

  boost::unique_lock<boost::shared_mutex> lock(*mutexState_);

  state_->setAnymalModelPtr(anymalModelMeasured_.get());
  state_->setDesiredAnymalModelPtr(anymalModelDesired_.get());

  anymal_roco::initializeState(*state_);
  anymal_roco::initializeCommand(*command_, *anymalModelMeasured_->getLimitsAnymal(), anymalModelMeasured_->getLegConfigurations());

  anymalModelMeasured_->setIsRealRobot(isRealRobot);
}

void ModelData::addVariablesToLog() {
  command_->addVariablesToLog(true);
  state_->addVariablesToLog(true);
  anymalModelMeasured_->addVariablesToLog(true, std::string{"/model/meas/"});
  anymalModelDesired_->addVariablesToLog(true, std::string{"/model/des/"});
}

void ModelData::switchLegConfigurationsTo(const anymal_model::LegConfigurations& legConfigAnymal) {
  anymalModelMeasured_->setLegConfigurations(legConfigAnymal);
  anymalModelDesired_->setLegConfigurations(legConfigAnymal);
  anymal_roco::initializeCommand(*command_, *anymalModelMeasured_->getLimitsAnymal(), legConfigAnymal);
}

void ModelData::setJoystickCommands(const sensor_msgs::Joy& joy) {
  boost::unique_lock<boost::shared_mutex> lock(*mutexState_);
  for (int i=0; i<joy.axes.size(); ++i) {
    state_->getJoystickPtr()->setAxis(i+1, joy.axes[i]);
  }
  for (int i=0; i<joy.buttons.size(); ++i) {
    state_->getJoystickPtr()->setButton(i+1, joy.buttons[i]);
  }
}

void ModelData::setCommandVelocity(const any_measurements::Twist& msg) {
  boost::unique_lock<boost::shared_mutex> lock(*mutexState_);
  state_->getDesiredRobotVelocityPtr()->getRotationalVelocity() = msg.twist_.getRotationalVelocity();
  state_->getDesiredRobotVelocityPtr()->getTranslationalVelocity() = msg.twist_.getTranslationalVelocity();
}

void ModelData::setCommandPose(const any_measurements::Pose& msg) {
  boost::unique_lock<boost::shared_mutex> lock(*mutexState_);
  state_->getDesiredRobotPosePtr()->getPosition() = msg.pose_.getPosition();
  state_->getDesiredRobotPosePtr()->getRotation() = msg.pose_.getRotation();
}


void ModelData::setActuatorReadings(const anymal_model::ActuatorReadingRobotContainer& msg) {
  boost::unique_lock<boost::shared_mutex> lock(*mutexState_);
  anymal_model::JointTorques jointTorques;
  anymal_model::JointAccelerations jointAccelerations;
  for(const auto actuatorKey : AD::getActuatorKeys()) {
    const auto actuatorEnum = actuatorKey.getEnum();
    const auto actuatorId = actuatorKey.getId();
    jointTorques(actuatorId) = msg[actuatorEnum].getState().getJointTorque();
    jointAccelerations(actuatorId) = msg[actuatorEnum].getState().getJointAcceleration();
    state_->getActuatorReadings()[actuatorEnum].setState(msg[actuatorEnum].getState());
  }
  anymalModelMeasured_->setJointTorques(jointTorques);
  anymalModelMeasured_->setJointAccelerations(jointAccelerations);
}

void ModelData::getActuatorCommands(anymal_model::ActuatorCommandRobotContainer& commands) const {
  boost::shared_lock<boost::shared_mutex> lock(*mutexCommand_);
  commands = command_->getActuatorCommands();
}

void ModelData::setAnymalState(const anymal_model::ExtendedAnymalState& state) {
  boost::unique_lock<boost::shared_mutex> lock(*mutexState_);

  anymalModelMeasured_->setState(state.anymalState_, true, true, false);
  state_->setStatus(static_cast<anymal_roco::RocoState::StateStatus>(state.status_));

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    anymalModelMeasured_->getContactContainer()[contactEnum]->setForce(state.contacts_[contactEnum].wrench_.wrench_.getForce(), AD::CoordinateFrameEnum::WORLD);
    anymalModelMeasured_->getContactContainer()[contactEnum]->setNormal(state.contacts_[contactEnum].normal_, AD::CoordinateFrameEnum::WORLD);
    anymalModelMeasured_->getContactContainer()[contactEnum]->setState(static_cast<AD::ContactStateEnum>(state.contacts_[contactEnum].state_));
  }
}

void ModelData::setLocalization(const LocalizationMsg& localization) {
  boost::unique_lock<boost::shared_mutex> lock(*mutexState_);
  state_->setLocalization(any_measurements_ros::fromRos<any_measurements::PoseWithCovariance, geometry_msgs::PoseWithCovarianceStamped>(localization));
}

void ModelData::setImuMeasurements(const any_measurements::Imu& imu) {
  boost::unique_lock<boost::shared_mutex> lock(*mutexState_);
  state_->setImu(imu);
}

} /* namespace model */
