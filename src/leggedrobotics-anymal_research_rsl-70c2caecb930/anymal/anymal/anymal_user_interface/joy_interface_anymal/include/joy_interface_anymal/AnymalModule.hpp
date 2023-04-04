/*!
* @file     AnymalModule.hpp
* @author   Linus Isler
* @date     October 13th, 2016
* @brief
*/

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

#include "joy_interface/InterfaceModuleBase.hpp"


namespace joy_interface {

class AnymalModule : public joy_interface::InterfaceModuleBase
{
 public:
  AnymalModule();
  virtual ~AnymalModule();
  void init(const ros::NodeHandle& nh, const std::string& name);
  void cleanup();
  void processJoy(const sensor_msgs::Joy::ConstPtr& joy,
                  std::vector<std::string>* modules,
                  std::vector<std::string>* commands);
};

} // namespace joy_interface
