/*
 * AnymalMotionControlForm.h
 *
 *  Created on: Feb 2020
 *      Author: lisler
 */

#ifndef HRI_USER_INTERFACE_LOCOMOTION_CONTROLLER_FORM_H
#define HRI_USER_INTERFACE_LOCOMOTION_CONTROLLER_FORM_H

#include <ros/ros.h>

#include <std_msgs/String.h>

#include <hri_user_interface/Form.h>
#include "hri_user_interface/HRIUserInterface.h"
#include <hri_user_interface/TextLineWidget.h>

#include <motion_transitioner_msgs/MotionState.h>


namespace hri_user_interface {

class AnymalMotionControlForm : public Form
{
 public:
  AnymalMotionControlForm(HRIUserInterface* userInteface);
  ~AnymalMotionControlForm() override = default;

  bool init(const ros::NodeHandle& nh) override;

  void updateMci(TextLineWidget& widget);
  void selectMotion(TextLineWidget& widget);

  void button4() override;
  void button2() override;

  void currentMotionStateCallback(const motion_transitioner_msgs::MotionStateConstPtr& msg);


 private:
  HRIUserInterface* hriUserInterface_;

  ros::Subscriber currentMotionStateSubscriber_;

  std::vector<std::string> desiredMotionStates_;
  std::string currentMotionState_ = "";

};

} // namespace hri_user_interface

#endif /* HRI_USER_INTERFACE_LOCOMOTION_CONTROLLER_FORM_H */
