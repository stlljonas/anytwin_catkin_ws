/*!
* @file     JoyRpsm.cpp
* @author   Christian Gehring
* @date     Feb, 2017
* @brief
*/

#include <pluginlib/class_list_macros.h>
#include <any_msgs/Shutdown.h>
#include "joy_rpsm/JoyRpsm.hpp"


namespace joy_manager {


JoyRpsm::JoyRpsm() {
  isProcessingService_ = false;
}


JoyRpsm::~JoyRpsm() {
}


void JoyRpsm::init(const ros::NodeHandle& nh,
                   joy_manager::JoyManager* joyManager, 
                   const std::string& name) {
  ModuleBase::init(nh, joyManager, name);
  ROS_INFO_STREAM("[" << name_ << "] init()");

  isProcessingService_ = false;
  rpsmClient_ = nh_.serviceClient<any_msgs::Shutdown>("/rpsm_lpc/shutdown");
}


void JoyRpsm::cleanup() {
  ModuleBase::cleanup();
  if (serviceThread_.joinable())
    serviceThread_.join();
  rpsmClient_.shutdown();
}


void JoyRpsm::processCommand(const std::string& command, std::string* answer) {
  ROS_INFO_STREAM("[" << name_ << "] received command: " << command);

  if (isProcessingService_) {
    *answer = "still processing";
    return;
  }
  
  if (serviceThread_.joinable()) {
    serviceThread_.join();
  }
  serviceThread_ = std::thread(std::bind(&JoyRpsm::runServiceThread, this, command));
  *answer = "started";
}


void JoyRpsm::runServiceThread(const std::string& command) {
  isProcessingService_ = true;
  if (command == "smart_full_shutdown") {
    any_msgs::Shutdown srv;
    srv.request.type = any_msgs::Shutdown::Request::SMART_FULL_SHUTDOWN;
    rpsmClient_.call(srv);
  }
  else if (command == "full_shutdown") {
    any_msgs::Shutdown srv;
    srv.request.type = any_msgs::Shutdown::Request::FULL_SHUTDOWN;
    rpsmClient_.call(srv);
  }
  else if (command == "smart_motor_shutdown") {
    any_msgs::Shutdown srv;
    srv.request.type = any_msgs::Shutdown::Request::SMART_MOTORS_POWER_SHUTDOWN;
    rpsmClient_.call(srv);
  }
  else if (command == "motor_shutdown") {
    any_msgs::Shutdown srv;
    srv.request.type = any_msgs::Shutdown::Request::MOTORS_POWER_SHUTDOWN;
    rpsmClient_.call(srv);
  }
  isProcessingService_ = false;
}


} // namespace joy_manager

//Declare the lowlevel controller module as a PluginBase class
PLUGINLIB_EXPORT_CLASS(joy_manager::JoyRpsm, joy_manager::ModuleBase)
