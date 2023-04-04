/*
 * EmergencyForm.h
 *
 *  Created on: 24. July 2018
 *      Author: linusisler
 */

#pragma once

#include <std_srvs/Trigger.h>

#include <hri_user_interface/Form.h>
#include "hri_user_interface/HRIUserInterface.h"
#include <hri_user_interface/TextLineWidget.h>


namespace hri_user_interface {

class EmergencyForm : public Form
{
 public:
  EmergencyForm(HRIUserInterface* userInteface);
  ~EmergencyForm() override = default;

  bool init(const ros::NodeHandle& nh) override;

  int handleButtons(int button, int steps) override;


 private:
  HRIUserInterface* hriUserInterface_;
};

} // namespace hri_user_interface

