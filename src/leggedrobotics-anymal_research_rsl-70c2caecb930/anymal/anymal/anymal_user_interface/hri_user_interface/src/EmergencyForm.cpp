/*
 * EmergencyForm.cpp
 *
 *  Created on: 24. July 2018
 *      Author: linusisler
 */


#include "hri_user_interface/EmergencyForm.h"


namespace hri_user_interface {

EmergencyForm::EmergencyForm(HRIUserInterface* userInteface)
: hriUserInterface_(userInteface)
{
  LineWidgetPtr tmpPtr;

  // configure the Lines
  tmpPtr.reset(new TextLineWidget("******ESTOP!******"));
  lines_.push_back(tmpPtr);

  tmpPtr.reset(new TextLineWidget(""));
  lines_.push_back(tmpPtr);

  tmpPtr.reset(new TextLineWidget("press (1) to clear"));
  lines_.push_back(tmpPtr);

}


bool EmergencyForm::init(const ros::NodeHandle& nh) {
  return Form::init(nh);
}


int EmergencyForm::handleButtons(int button, int steps){
  if (button == ButtonConfig::BUTTON_1) {
    hriUserInterface_->setCommand("JoyMotionControl", "p_stop clear");
  }
  return -1;
}


} // namespace hri_user_interface
