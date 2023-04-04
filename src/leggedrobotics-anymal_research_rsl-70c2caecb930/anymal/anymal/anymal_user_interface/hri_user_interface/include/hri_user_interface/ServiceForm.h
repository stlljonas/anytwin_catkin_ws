/*
 * ServiceForm.h
 *
 *  Created on: 11 March 2020
 *      Author: lisler
 */

#ifndef HRI_USER_INTERFACE_LOW_LEVEL_CONTROLLER_FORM_H
#define HRI_USER_INTERFACE_LOW_LEVEL_CONTROLLER_FORM_H

#include <string>
#include <mutex>

#include <ros/ros.h>

#include <hri_user_interface/Form.h>
#include <hri_user_interface/TextLineWidget.h>
#include <hri_user_interface/StatusLineWidget.h>
#include <hri_user_interface/HRIUserInterface.h>

#include <anymal_lowlevel_controller_common/state_machine/StateEnum.hpp>

#include <anymal_msgs/GetAvailableControllers.h>

#include <hri_safety_sense/EmergencyStop.h>
#include <hri_safety_sense/KeyString.h>
#include <hri_safety_sense/KeyValue.h>
#include <hri_safety_sense/VehicleMessages.h>
#include <hri_safety_sense/KeyValueResp.h>


namespace hri_user_interface {

class ServiceForm : public Form
{
  ros::Subscriber activeStateSubscriber_;

  std::string currentGoalState_;
  std::mutex activeStateMutex_;
  std::string activeState_;
  HRIUserInterface* hriUserInterface_;

 public:
  ServiceForm(HRIUserInterface* hriUserInterface);
  ~ServiceForm() override = default;

  bool init(const ros::NodeHandle& nh) override;

  void updateGoalState(TextLineWidget& widget);
  void selectGoalState(TextLineWidget& widget);
  void focusGoalState(TextLineWidget& widget);

  void updateActiveState(StatusLineWidget& widget);

 protected:
  void activeStateCb(const anymal_msgs::AnymalLowLevelControllerStateConstPtr& msg);

  std::string lowlevelControllerNamespace_;
  std::string joyLowlevelControllerName_;
};

} // namespace hri_user_interface

#endif /* HRI_USER_INTERFACE_LOW_LEVEL_CONTROLLER_FORM_H */
