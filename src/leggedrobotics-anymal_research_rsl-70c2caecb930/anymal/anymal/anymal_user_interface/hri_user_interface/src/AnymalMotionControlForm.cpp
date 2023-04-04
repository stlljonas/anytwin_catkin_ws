/*
 * AnymalMotionControlForm.cpp
 *
 *  Created on: Feb 2020
 *      Author: lisler
 */

#include "hri_user_interface/AnymalMotionControlForm.h"

// param io
#include <param_io/get_param.hpp>

namespace hri_user_interface {

AnymalMotionControlForm::AnymalMotionControlForm(HRIUserInterface* userInteface)
: hriUserInterface_(userInteface)
{
  LineWidgetPtr tmpPtr;

  // configure the Lines
  tmpPtr.reset(new TextLineWidget("Motion Control"));
  boost::static_pointer_cast<TextLineWidget>(tmpPtr)->setUpdateFun(boost::bind(&AnymalMotionControlForm::updateMci, this, _1));
  lines_.push_back(tmpPtr);

  tmpPtr.reset(new TextLineWidget());
  boost::static_pointer_cast<TextLineWidget>(tmpPtr)->setUpdateFun(boost::bind(&AnymalMotionControlForm::updateMci, this, _1));
  boost::static_pointer_cast<TextLineWidget>(tmpPtr)->setSelectFun(boost::bind(&AnymalMotionControlForm::selectMotion, this, _1));
  lines_.push_back(tmpPtr);
}

bool AnymalMotionControlForm::init(const ros::NodeHandle& nh) {
  bool success = Form::init(nh);

  success = success && param_io::getParam(nh, "desired_motion_states", desiredMotionStates_);
  boost::static_pointer_cast<TextLineWidget>(lines_[1])->setLines(desiredMotionStates_);

  currentMotionStateSubscriber_ =
      nh_.subscribe("/anymal_highlevel_controller/current_motion_state", 1,&AnymalMotionControlForm::currentMotionStateCallback, this);

  lines_[0]->update();
  return success;
}

void AnymalMotionControlForm::updateMci(TextLineWidget& widget) {
  if (!currentMotionState_.empty() && !boost::static_pointer_cast<TextLineWidget>(lines_[1])->setCurrentText(currentMotionState_)) {
    ROS_DEBUG_STREAM("Couldn't set current motion state: " << currentMotionState_);
    return;
  }
}

void AnymalMotionControlForm::selectMotion(TextLineWidget& widget) {
  // Set the currently selected motion.
  hriUserInterface_->setCommand("JoyMotionControl", "go_to " + widget.getCurrentText());
}

void AnymalMotionControlForm::button4() {
  hriUserInterface_->setCommand("JoyMotionControl", "go_to walk");
  ros::Duration(1.0).sleep();
  lines_[1]->update();
  hriUserInterface_->updateJoystickScreen();
}

void AnymalMotionControlForm::button2() {
  hriUserInterface_->setCommand("JoyMotionControl", "go_to torso_control");
  ros::Duration(1.0).sleep();
  lines_[1]->update();
  hriUserInterface_->updateJoystickScreen();
}

void AnymalMotionControlForm::currentMotionStateCallback(const motion_transitioner_msgs::MotionStateConstPtr& msg) {
  currentMotionState_ = msg->name;
  // If the currently active motion state is not part of the desired motion state list the list is temporarily updated to show it anyway.
  if (std::find(desiredMotionStates_.begin(), desiredMotionStates_.end(), currentMotionState_) != desiredMotionStates_.end()) {
    std::vector<std::string> tempMotionStates = desiredMotionStates_;
    tempMotionStates.push_back(currentMotionState_);
    boost::static_pointer_cast<TextLineWidget>(lines_[1])->setLines(tempMotionStates);
  }
  // Else we reset the desired motion states.
  else{
    boost::static_pointer_cast<TextLineWidget>(lines_[1])->setLines(desiredMotionStates_);
  }
  lines_[1]->update();
  hriUserInterface_->updateJoystickScreen();
}

} // namespace hri_user_interface
