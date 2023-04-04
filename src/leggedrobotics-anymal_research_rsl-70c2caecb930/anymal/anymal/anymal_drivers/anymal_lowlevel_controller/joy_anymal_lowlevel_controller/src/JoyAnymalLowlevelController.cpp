/*!
* @file     JoyAnymalLowlevelController.cpp
* @author   Linus Isler
* @date     June, 2016
* @brief
*/

#include <pluginlib/class_list_macros.h>
#include <anymal_lowlevel_controller_common/state_machine/StateEnum.hpp>
#include <anymal_msgs/AnymalLowLevelControllerGoToState.h>
#include "joy_anymal_lowlevel_controller/JoyAnymalLowlevelController.hpp"


namespace joy_manager {


JoyAnymalLowlevelController::JoyAnymalLowlevelController() {
}


JoyAnymalLowlevelController::~JoyAnymalLowlevelController() {
}


void JoyAnymalLowlevelController::init(const ros::NodeHandle& nh, 
                                       joy_manager::JoyManager* joyManager, 
                                       const std::string& name) {
  ModuleBase::init(nh, joyManager, name);
  ROS_INFO_STREAM("[" << name_ << "] init()");

  std::string ns = "/anymal_lowlevel_controller";
  switchLowLevelControllerClient_ = nh_.serviceClient<anymal_msgs::AnymalLowLevelControllerGoToState>(ns + "/go_to_state");
}


void JoyAnymalLowlevelController::cleanup() {
  ModuleBase::cleanup();
}


void JoyAnymalLowlevelController::processCommand(const std::string& command, std::string* answer) {
  ROS_INFO_STREAM("[" << name_ << "] received command: " << command);

  setGoalState(command, answer);
}


void JoyAnymalLowlevelController::setGoalState(const std::string& command, std::string* answer) {
  if (switchLowLevelControllerClient_.exists()) {
    anymal_msgs::AnymalLowLevelControllerGoToState goToState;
    goToState.request.state = anymal_lowlevel_controller_common::state_machine::stateEnumToMsg(
        anymal_lowlevel_controller_common::state_machine::stateNameToEnum(command));
    if (switchLowLevelControllerClient_.call(goToState)) {
      *answer = "success";
      return;
    }
  }
  ROS_WARN_STREAM("[" << name_ << "] Could not switch low-level controller!");
  *answer = "error";
}


} // namespace joy_manager

//Declare the lowlevel controller module as a PluginBase class
PLUGINLIB_EXPORT_CLASS(joy_manager::JoyAnymalLowlevelController, joy_manager::ModuleBase)
