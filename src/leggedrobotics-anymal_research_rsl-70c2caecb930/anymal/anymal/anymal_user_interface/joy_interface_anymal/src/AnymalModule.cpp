/*!
* @file     AnymalModule.cpp
* @author   Linus Isler
* @date     October 13th, 2016
* @brief
*/

#include "joy_interface_anymal/AnymalModule.hpp"

#include <pluginlib/class_list_macros.h>




namespace joy_interface {

AnymalModule::AnymalModule() {}


AnymalModule::~AnymalModule() {

}


void AnymalModule::init(const ros::NodeHandle& nh, const std::string& name) {
  InterfaceModuleBase::init(nh, name);
  ROS_DEBUG_STREAM("[" << name_ << "] init()");
}


void AnymalModule::cleanup() {
  InterfaceModuleBase::cleanup();
}


void AnymalModule::processJoy(const sensor_msgs::Joy::ConstPtr& joy,
                              std::vector<std::string>* modules,
                              std::vector<std::string>* commands) {
  // B-button: go to torso_control
  if (joy->buttons[1] == 1) {
    modules->push_back("JoyMotionControl");
    commands->push_back("go_to torso_control");
    return;
  }

  // X-button: go to walk/trot mode in active controller
  if (joy->buttons[2] == 1) {
    modules->push_back("JoyMotionControl");
    commands->push_back("go_to walk");
    return;
  }

}

} // namespace joy_interface

//Declare the localization module plugin as a JoyInterface class
PLUGINLIB_EXPORT_CLASS(joy_interface::AnymalModule, joy_interface::InterfaceModuleBase)
