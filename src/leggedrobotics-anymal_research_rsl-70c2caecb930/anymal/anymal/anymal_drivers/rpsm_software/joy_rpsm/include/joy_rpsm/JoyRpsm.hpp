/*!
* @file     JoyRpsm.hpp
* @author   Christian Gehring
* @date     Feb, 2017
* @brief
*/

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <atomic>
#include <thread>
#include <any_msgs/SetUInt32.h>
#include <std_srvs/Trigger.h>
#include <joy_manager/ModuleBase.hpp>



namespace joy_manager {

class JoyRpsm : public joy_manager::ModuleBase
{
 public:
  JoyRpsm();
  virtual ~JoyRpsm();
  void init(const ros::NodeHandle& nh, joy_manager::JoyManager* joyManager, const std::string& name);
  void cleanup();
  void processCommand(const std::string& command, std::string* answer);

 protected:
  void runServiceThread(const std::string& command);

  std::thread serviceThread_;

  std::atomic<bool> isProcessingService_;

  ros::ServiceClient rpsmClient_;


};

} // namespace joy_manager
