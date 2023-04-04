#pragma once

// anymal model
#include <anymal_model/AnymalFramesGenerator.hpp>
#include <anymal_model/AnymalModel.hpp>
#include <anymal_model/ExtendedAnymalState.hpp>

// seabot gazebo plugin
#include <seabot_gazebo_plugin/SeabotGazeboPlugin.hpp>

// anymal msgs
#include <anymal_msgs/AnymalState.h>

// cosmo ros
#include <cosmo_ros/cosmo_ros.hpp>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// anydrive
#include <anydrive/ReadingExtended.hpp>
#include <anydrive/StateExtended.hpp>

// anydrive msgs
#include <anydrive_msgs/ReadingsExtended.h>

namespace gazebo {

using SeaGazeboPluginBase = SeabotGazeboPlugin<anymal_description::AnymalDescription>;
using AnymalGazeboPluginBase = AnybotGazeboPlugin<anymal_description::AnymalDescription>;

// The AnymalGazeboPlugin class interfaces ANYmal with the Gazebo simulator.
class AnymalGazeboPlugin : public SeaGazeboPluginBase {
 public:
  using AD = anymal_description::AnymalDescription;
  using AnymalStateShm = anymal_model::ExtendedAnymalState;
  using AnymalStateRos = anymal_msgs::AnymalState;

  // Constructor.
  AnymalGazeboPlugin();

  void Reset() override;

 protected:
  // Reads parameters from the parameter server.
  void readParameters() override;

  void initSubscribers() override;
  // Inits the ROS publishers.
  void initPublishers() override;
  // Inits the ROS services.
  void initServices() override;

  void publishActuatorReadingsDerived() override;

  // Publishes robot state over ROS.
  void publishRobotState() override;

  void sendRos() override;

  void setActuatorGains() override;

  WrenchShm createContactWrenchMessage(const ContactEnum& contactEnum) override;

  // Anymal model.
  anymal_model::AnymalModel anymalModel_;

  // Frames generator.
  anymal_model::AnymalFramesGenerator<anymal_description::ConcreteAnymalDescription, anymal_model::AnymalState> framesGenerator_;

  // Anymal specific frames.
  std::string frameFootprint_;
  std::string frameFeetcenter_;

  cosmo_ros::PublisherRosPtr<AnymalStateShm, AnymalStateRos, anymal_model_ros::conversion_traits::ConversionTraits> anymalStatePublisher_;

  ros::Publisher anymalStateThrottledPublisher_;
  unsigned int robotStateThrottledCounter_ = 0;
  unsigned int robotStateThrottledDecimation_ = 0;

  AnymalStateShm anymalMsgShm_;
  AnymalStateRos anymalMsgRos_;

  // ROS throttled extended actuator readings publisher.
  ros::Publisher actuatorReadingsExtendedThrottledPublisher_;
  unsigned int actuatorReadingsExtendedThrottledCounter_ = 0;
  unsigned int actuatorReadingsExtendedThrottledDecimation_ = 0;
  anydrive_msgs::ReadingsExtended readingsExtendedMsg_;
};

}  // namespace gazebo
