/*!
* @file     Joystick.cpp
* @author   Linus Isler
* @date     May, 2016
* @brief
*/

#include <joy_manager/Joystick.hpp>



namespace joy_manager {


Joystick::Joystick(const ros::NodeHandle& nh, joy_manager::JoyManager* joyManager)
:
  nh_(nh),
  joyManager_(joyManager),
  priority_(0),
  name_("")
{}


Joystick::~Joystick() {
}


void Joystick::setName(const std::string& name) {
  name_ = name;
}


const std::string Joystick::getName() const {
  return name_;
}


void Joystick::setPriority(const int& priority) {
  priority_ = priority;
}


int Joystick::getPriority() const {
  return priority_;
}


void Joystick::setTopic(const std::string topic) {
  anyJoySubscriber_ = nh_.subscribe(topic, 10, &Joystick::callback, this);
}


const std::string Joystick::getTopic() const {
  return anyJoySubscriber_.getTopic();
}

int Joystick::getNumberOfPublishers() const {
  return anyJoySubscriber_.getNumPublishers();
}


void Joystick::callback(const joy_manager_msgs::AnyJoy::ConstPtr& msg) {
  joyManager_->checkPriority(msg, priority_, isNonZero(msg));
  lastJoyMsg_ = msg->joy;
}

bool Joystick::isNonZero(const joy_manager_msgs::AnyJoy::ConstPtr& msg) {
  for (size_t i = 0; i < msg->joy.axes.size(); i++) {
    if (msg->joy.axes[i] != 0.0) {
      return true;
    }
  }
  if (!lastJoyMsg_.buttons.empty()) {
    if (msg->joy.buttons != lastJoyMsg_.buttons) {
      return true;
    }
  }
  if (!msg->modules.empty() && msg->modules[0] != "") {
    return true;
  }
  return false;
}


} // namespace joy_manager
