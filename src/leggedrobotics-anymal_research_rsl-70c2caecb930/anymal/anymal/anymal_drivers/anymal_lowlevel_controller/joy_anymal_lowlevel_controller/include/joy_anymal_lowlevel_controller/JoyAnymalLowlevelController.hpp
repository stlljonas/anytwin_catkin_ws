/*!
* @file     JoyAnymalLowlevelController.hpp
* @author   Linus Isler
* @date     June, 2016
* @brief
*/

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <std_srvs/Trigger.h>
#include <joy_manager/ModuleBase.hpp>



namespace joy_manager {

class JoyAnymalLowlevelController : public joy_manager::ModuleBase
{
 public:
  JoyAnymalLowlevelController();
  virtual ~JoyAnymalLowlevelController();
  void init(const ros::NodeHandle& nh, joy_manager::JoyManager* joyManager, const std::string& name);
  void cleanup();
  void processCommand(const std::string& command, std::string* answer);

 protected:
  void setGoalState(const std::string& command, std::string* answer);

  ros::ServiceClient switchLowLevelControllerClient_;

};

} // namespace joy_manager
