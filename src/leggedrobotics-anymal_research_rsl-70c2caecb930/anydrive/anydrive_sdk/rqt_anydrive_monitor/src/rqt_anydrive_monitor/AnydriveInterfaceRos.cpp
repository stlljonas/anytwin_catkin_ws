#include <anydrive/Command.hpp>

#include <anydrive_ros/AnydriveRos.hpp>

#include <anydrive_msgs/Command.h>
#include <anydrive_msgs/SendControlword.h>
#include <anydrive_msgs/SetFsmGoalState.h>

#include "rqt_anydrive_monitor/AnydriveInterfaceRos.h"

namespace rqt_anydrive_monitor {

/* ========================================================================== */
/* Constructor/Destructor                                                     */
/* ========================================================================== */

AnydriveInterfaceRos::AnydriveInterfaceRos(ros::NodeHandle nh, std::string deviceName, std::string rosPrefix, bool enableSubscriber,
                                           QWidget* parent)
    : anydrive_monitor::AnydriveInterface(parent),
      nh_(nh),
      deviceName_(std::move(deviceName)),
      rosPrefix_(std::move(rosPrefix)),
      enableSubscriber_(enableSubscriber),
      readingReceived_(false) {
  setName(QString::fromStdString(deviceName_));

  connect(this, SIGNAL(sigStateChanged(const QString&)), this, SLOT(setGoalState(const QString&)));
  connect(this, SIGNAL(sigSendControlword()), this, SLOT(sendControlword()));
  connect(this, SIGNAL(sigResetCommand()), this, SLOT(resetCommand()));
  connect(this, SIGNAL(sigSendCommand()), this, SLOT(sendCommand()));
  connect(this, SIGNAL(sigSendCommandDisable()), this, SLOT(sendCommandDisable()));
  connect(this, SIGNAL(sigSendCommandFreeze()), this, SLOT(sendCommandFreeze()));
}

/* ========================================================================== */
/* Accessors                                                                  */
/* ========================================================================== */

void AnydriveInterfaceRos::startup() {
  if (enableSubscriber_) {
    readingSubscriber_ = nh_.subscribe("/" + rosPrefix_ + "/" + deviceName_ + "/reading_extended_throttled", 1,
                                       &AnydriveInterfaceRos::readingCallback, this);
  }

  setGoalStateClient_ = nh_.serviceClient<anydrive_msgs::SetFsmGoalState>("/" + rosPrefix_ + "/set_goal_state", false);

  sendControlwordClient_ = nh_.serviceClient<anydrive_msgs::SendControlword>("/" + rosPrefix_ + "/send_controlword", false);

  commandPublisher_ = nh_.advertise<anydrive_msgs::Command>("/" + rosPrefix_ + "/" + deviceName_ + "/command", 1, false);
}

void AnydriveInterfaceRos::shutdown() {
  if (enableSubscriber_) {
    readingSubscriber_.shutdown();
  }
  setGoalStateClient_.shutdown();
  sendControlwordClient_.shutdown();
  commandPublisher_.shutdown();
}

void AnydriveInterfaceRos::updateAnydrive() {
  if (!readingReceived_) {
    return;
  }

  // Use a copy of the reading to update the GUI.
  anydrive::ReadingExtended reading;
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    reading = reading_;
  }
  readingReceived_ = false;

  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  anydrive::Statusword statusword(reading.getState().getStatusword());
  std::vector<std::string> infos;
  std::vector<std::string> warnings;
  std::vector<std::string> errors;
  std::vector<std::string> fatals;
  statusword.getMessages(infos, warnings, errors, fatals);
  updateAnydriveWidget(statusword.getStateEnum(), statusword.getModeEnum(), infos, warnings, errors, fatals,
                       reading.getState().getJointPosition(), reading.getState().getJointVelocity(), reading.getState().getJointTorque(),
                       reading.getState().getCurrent(), reading.getState().getVoltage(), reading.getState().getTemperature());
}

/* ========================================================================== */
/* Callbacks                                                                  */
/* ========================================================================== */

void AnydriveInterfaceRos::readingCallback(const anydrive_msgs::ReadingExtended& msg) {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  anydrive_ros::readFromMessage(reading_, msg);
  readingReceived_ = true;
}

/* ========================================================================== */
/* Slots                                                                      */
/* ========================================================================== */

void AnydriveInterfaceRos::setGoalState(const QString& stateCommand) {
  // Get the selected goal state.
  anydrive::fsm::StateEnum goalState = anydrive::fsm::stateNameToEnum(stateCommand.toStdString());

  // Set the goal state.
  anydrive_msgs::SetFsmGoalState setGoalState;
  setGoalState.request.device_name = deviceName_;
  setGoalState.request.fsm_state = anydrive_ros::stateEnumToMsg(goalState);
  if (!setGoalStateClient_.exists()) {
    ANYDRIVE_ERROR("Service server '" << setGoalStateClient_.getService() << "' does not exist.");
    return;
  }
  if (!setGoalStateClient_.call(setGoalState)) {
    ANYDRIVE_ERROR("Service call '" << setGoalStateClient_.getService() << "' returned false.");
    return;
  }
}

void AnydriveInterfaceRos::sendControlword() {
  if (!sendControlwordClient_.exists()) {
    ANYDRIVE_ERROR("Service server '" << sendControlwordClient_.getService() << "' does not exist.");
    return;
  }

  anydrive_msgs::SendControlword sendControlword;
  sendControlword.request.device_name = deviceName_;
  sendControlword.request.controlword = anydrive::fsm::controlwordStringToId(getControlword().toStdString());
  if (!sendControlwordClient_.call(sendControlword)) {
    ANYDRIVE_ERROR("Service call '" << sendControlwordClient_.getService() << "' returned false.");
    return;
  }
}

void AnydriveInterfaceRos::sendCommand() {
  anydrive::mode::ModeEnum mode = anydrive::mode::ModeEnum::NA;
  double position = 0.0;
  double velocity = 0.0;
  double jointTorque = 0.0;
  double current = 0.0;
  double pidGainsP = 0.0;
  double pidGainsI = 0.0;
  double pidGainsD = 0.0;
  getCommand(mode, position, velocity, jointTorque, current, pidGainsP, pidGainsI, pidGainsD);

  // Send the command.
  anydrive::Command command;
  command.setModeEnum(mode);
  command.setStamp(any_measurements::Time::NowWallClock());
  command.setMotorPosition(position);
  command.setMotorVelocity(velocity);
  command.setGearPosition(position);
  command.setGearVelocity(velocity);
  command.setJointPosition(position);
  command.setJointVelocity(velocity);
  command.setJointTorque(jointTorque);
  command.setCurrent(current);
  command.getPidGains().setP(pidGainsP);
  command.getPidGains().setI(pidGainsI);
  command.getPidGains().setD(pidGainsD);
  anydrive_msgs::Command commandMsg;
  anydrive_ros::writeToMessage(commandMsg, command);
  commandPublisher_.publish(commandMsg);
}

void AnydriveInterfaceRos::resetCommand() {
  double position = 0.0;
  {
    std::lock_guard<std::recursive_mutex> lock(readingMutex_);
    position = reading_.getState().getJointPosition();
  }
  setCommand(position, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void AnydriveInterfaceRos::sendCommandDisable() {
  // Send the command.
  anydrive::Command command;
  command.setModeEnum(anydrive::mode::ModeEnum::Disable);
  command.setStamp(any_measurements::Time::NowWallClock());
  anydrive_msgs::Command commandMsg;
  anydrive_ros::writeToMessage(commandMsg, command);
  commandPublisher_.publish(commandMsg);
}

void AnydriveInterfaceRos::sendCommandFreeze() {
  // Send the command.
  anydrive::Command command;
  command.setModeEnum(anydrive::mode::ModeEnum::Freeze);
  command.setStamp(any_measurements::Time::NowWallClock());
  anydrive_msgs::Command commandMsg;
  anydrive_ros::writeToMessage(commandMsg, command);
  commandPublisher_.publish(commandMsg);
}

}  // namespace rqt_anydrive_monitor
