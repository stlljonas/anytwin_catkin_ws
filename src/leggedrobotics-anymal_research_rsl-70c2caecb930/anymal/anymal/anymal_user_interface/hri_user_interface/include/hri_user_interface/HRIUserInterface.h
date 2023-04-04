/*
 * HRIUserInterface.h
 *
 *  Created on: 21 May 2015
 *      Author: markuszahner
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>

#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include <any_node/any_node.hpp>
#include <anydrive_msgs/ReadingsExtended.h>
#include <std_msgs/UInt32.h>
#include <sensor_msgs/Joy.h>
#include <joy_manager_msgs/AnyJoy.h>
#include <notification/NotificationSubscriber.hpp>
#include <hri_safety_sense/EmergencyStop.h>
#include <hri_safety_sense/EstopStatus.h>
#include <hri_safety_sense/KeyString.h>
#include <hri_safety_sense/KeyValue.h>
#include <hri_safety_sense/VehicleMessages.h>
#include <hri_safety_sense/KeyValueResp.h>
#include <anymal_msgs/AnymalState.h>
#include <sensor_msgs/BatteryState.h>

#include <hri_user_interface/Form.h>
#include <hri_user_interface/Buttons.h>

// rocoma controller manager state to read estop_cleared value
#include <rocoma_msgs/ControllerManagerState.h>


// The successful operation of this node
// depends on the order of these includes!

static constexpr int kNumberScreenLines = 3;
static constexpr int kNumberOfAxes = 6;
static constexpr int kNumberOfButtons = 8;
static constexpr double kStatusLineUpdateFrequency = 2.0;
static constexpr double kNotificationLifespan = 2.0;


namespace hri_user_interface {

class HRIUserInterface: public any_node::Node {
public:
  HRIUserInterface() = delete;
  HRIUserInterface(any_node::Node::NodeHandlePtr nh);
  ~HRIUserInterface() override = default;
  bool init() override;
  void cleanup() override;

  /*!
   * @brief Set a command for the AnyJoy message. Called in the forms.
   * @param module Module to call in the JoyManager
   * @param command Command to send to the module
   */
  void setCommand(const std::string& module, const std::string& command);
  void setRemoteKey(int key,int value);
  void setRemoteString(int key,std::string str);
  void updateJoystickScreen();


protected:
  void notificationCallback(const notification::Notification & msg);
  void hriJoystickCallback(const sensor_msgs::Joy::ConstPtr & msg);
  void anydriveCallback(const anydrive_msgs::ReadingsExtended::ConstPtr & msg);
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr & msg);
  void anymalStateCallback(const anymal_msgs::AnymalState::ConstPtr & msg);
  void hriEmergencyStopCallback(const hri_safety_sense::EstopStatus::ConstPtr & msg);
  void controllerManagerStateCallback(const rocoma_msgs::ControllerManagerStateConstPtr& msg);
  void handleJoystickButtonPressed(int button,int action);
  bool loadParameters();
  void setJoystickKeys(std::vector<std::string> keys);
  bool publishJoystick(const any_worker::WorkerEvent& timerEvent);
  /*!
   * The top line in the HRI display shows the incomming notifications, the maximal drive temperature,
   * the StateEstimator state and the battery voltage.
   * 01234567890123456789
   * 48.3V  52.0°  SE :(
   * success
   * 48.0V  61.3°  SE :)
   * @return True if the update succeeded.
   */
  void updateStatusLine(const ros::TimerEvent& event);

  ros::Subscriber hriJoySubscriber_;
  sensor_msgs::Joy hriJoy_;
  boost::shared_mutex hriJoyMutex_;
  ros::Subscriber hriEmergencyStopSubscriber_;
  ros::Subscriber controllerManagerStateSubscriber_;
  ros::Subscriber anydriveReadingsSubscriber_;
  ros::Subscriber batteryStateSubscriber_;
  ros::Subscriber anymalStateSubscriber_;
  boost::shared_ptr<notification::NotificationSubscriber> notificationSubscriberPtr_;

  ros::ServiceClient joySetTextOutputClient_;
  ros::ServiceClient joySetKeyValueClient_;

  ros::Publisher anyJoyPublisher_;
  joy_manager_msgs::AnyJoy anyJoyMsg_;

  Buttons buttons_;
  double sliderLeft_;
  double sliderRight_;
  bool firstRemoteConnection_;
  bool isReceivingJoyMessages_;
  std::mutex connectionMutex_;
  bool hardEmergencyStop_;
  int notificationLevel_;          // when do i react
  int notificationVibration_;      // when to vibrate the joystick
  ros::Duration timeOut_;
  std::unordered_map<std::string, std::string> currentCommands_;
  std::mutex currentCommandsMutex_;

  // GUI states
  std::vector<FormPtr> forms_;
  FormPtr emergencyForm_;
  FormPtr currentForm_;
  std::vector<FormPtr>::iterator lastFormIt_;

  // Status infos
  ros::Timer statusLineTimer_;
  double batteryVoltage_ = 0.0;
  int batteryChargingState_ = 0;
  double maxDriveTemperature_ = 0.0;
  int stateEstimatorState_ = -1;
  std::string notification_;
  std::string statusLine_;
  ros::Duration notificationAge_;
};

} // namespace hri_user_interface
