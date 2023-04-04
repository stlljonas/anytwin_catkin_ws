/*
 * ServiceForm.cpp
 *
 *  Created on: 11 March 2020
 *      Author: lisler
 */

#include "hri_user_interface/ServiceForm.h"

// param io
#include <param_io/get_param.hpp>

using namespace anymal_lowlevel_controller_common::state_machine;

namespace hri_user_interface {

ServiceForm::ServiceForm(HRIUserInterface* hriUserInterface)
: hriUserInterface_(hriUserInterface)
{
  currentGoalState_ = stateEnumToName(static_cast<StateEnum>(static_cast<unsigned int>(StateEnum::NA) + 1)); // TODO: Why not simply NA? Where is currentGoalState_ updated?
  activeState_ = stateEnumToName(StateEnum::NA);

  LineWidgetPtr tmpPtr;

  // configure the Lines
  tmpPtr.reset(new TextLineWidget("Services"));
  lines_.push_back(tmpPtr);

  tmpPtr.reset(new TextLineWidget());
  boost::static_pointer_cast<TextLineWidget>(tmpPtr)->setUpdateFun(boost::bind(&ServiceForm::updateGoalState, this, _1));
  boost::static_pointer_cast<TextLineWidget>(tmpPtr)->setSelectFun(boost::bind(&ServiceForm::selectGoalState, this, _1));
  boost::static_pointer_cast<TextLineWidget>(tmpPtr)->setFocusFun(boost::bind(&ServiceForm::focusGoalState, this, _1));
  lines_.push_back(tmpPtr);

  tmpPtr.reset(new StatusLineWidget());
  boost::static_pointer_cast<StatusLineWidget>(tmpPtr)->setUpdateFun(boost::bind(&ServiceForm::updateActiveState, this, _1));
  lines_.push_back(tmpPtr);
}

bool ServiceForm::init(const ros::NodeHandle& nh) {
  bool success = Form::init(nh);

  // Load parameters.
  success = success && param_io::getParam(nh, "lowlevel_controller_namespace", lowlevelControllerNamespace_);
  success = success && param_io::getParam(nh, "joy_lowlevel_controller_name", joyLowlevelControllerName_);

  activeStateSubscriber_ = nh_.subscribe(lowlevelControllerNamespace_ + "/active_state", 1, &ServiceForm::activeStateCb, this);
  return success;
}

void ServiceForm::updateGoalState(TextLineWidget& widget) {
  std::vector<std::string> lines({
      "SE reset",
      "SE reset_here",
      "Go default",
      "Go rest",
      stateEnumToCommandName(StateEnum::ActionActuatorsClearErrors),
      stateEnumToCommandName(StateEnum::ActionActuatorsDisable),
      stateEnumToCommandName(StateEnum::ActionActuatorsEnable),
      stateEnumToCommandName(StateEnum::ActionActuatorsWarmReset),
      stateEnumToCommandName(StateEnum::StateFatal),
      stateEnumToCommandName(StateEnum::StateOperational),
      stateEnumToCommandName(StateEnum::StateIdle),
      stateEnumToCommandName(StateEnum::StateZeroJointTorque)
  });
  widget.setLines(lines);
  lines_[2]->update();
}

void ServiceForm::selectGoalState(TextLineWidget& widget) {
  if (widget.getCurrentText().rfind("SE", 0) == 0) {
    hriUserInterface_->setCommand("JoyStateEstimation", widget.getCurrentText().substr(3));
  }
  else if (widget.getCurrentText().rfind("Go", 0) == 0) {
    hriUserInterface_->setCommand("JoyMotionControl", "execute " + widget.getCurrentText().substr(3));
  }
  else {
    hriUserInterface_->setCommand(joyLowlevelControllerName_, stateEnumToName(stateCommandNameToEnum(widget.getCurrentText())));
  }
}

void ServiceForm::focusGoalState(TextLineWidget& widget) {
  if (currentGoalState_ != widget.getCurrentText()) {
    if (!widget.setCurrentText(currentGoalState_)) {
      ROS_WARN("Could not adjust current state!\n");
    }
  }
  lines_[2]->update();
}

void ServiceForm::updateActiveState(StatusLineWidget& widget) {
  std::lock_guard<std::mutex> lock(activeStateMutex_);
  if (activeState_ != widget.getCurrentText()) {
    if (!widget.setCurrentText(activeState_)) {
      ROS_WARN("Could not update current state!\n");
    }
  }
}

void ServiceForm::activeStateCb(const anymal_msgs::AnymalLowLevelControllerStateConstPtr& msg) {
  std::lock_guard<std::mutex> lock(activeStateMutex_);
  activeState_ = stateEnumToName(stateMsgToEnum(*msg));
}


} // namespace hri_user_interface
