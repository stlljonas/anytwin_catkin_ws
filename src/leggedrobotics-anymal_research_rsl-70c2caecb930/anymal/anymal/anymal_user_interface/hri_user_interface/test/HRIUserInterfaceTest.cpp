/*!
* @file     HRIUserInterfaceTest.cpp
* @author   Linus Isler
* @date     June, 2016
* @brief
*/

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <hri_user_interface/HRIUserInterface.h>
#include <joy_manager_msgs/AnyJoy.h>
#include <hri_safety_sense/EstopStatus.h>
#include <notification_msgs/Notification.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>



using namespace hri_user_interface;

class AnyJoyCallback {
public:
  AnyJoyCallback()
  : firstEntry_(0.0) {
  };
  void callback(const joy_manager_msgs::AnyJoyConstPtr& msg) {
    if (!msg->joy.axes.empty() && msg->joy.axes[0] != 0.0) {
      firstEntry_ = msg->joy.axes[0];
    }
  }
  double getEntry() {
    return firstEntry_;
  }
private:
  double firstEntry_;
};


/* Test the hri user-interface by sending a joy message
 * and test if it is forwarded as an AnyJoy message.
 */
TEST(HRIUserInterface, Message)
{
  ros::NodeHandle nh("~");
  ros::Duration(2.0).sleep();

  // initialize publisher
  std::string subTopic, pubTopic;
  nh.getParam("/hri_user_interface/subscribers/hri_joy/topic", subTopic);
  nh.getParam("/hri_user_interface/publishers/anyJoy/topic", pubTopic);
  if (subTopic.empty() || pubTopic.empty())
    FAIL();
  ros::Publisher joyPublisher = nh.advertise<sensor_msgs::Joy>(subTopic, 1);
  sensor_msgs::Joy joyMsg;
  joyMsg.header.stamp = ros::Time::now();
  double value = rand() % 1024;
  joyMsg.axes.assign(6, value);
  joyMsg.buttons.assign(8, 0);
  ros::Time twoSecs = ros::Time::now() + ros::Duration(2.0);
  AnyJoyCallback anyJoyCallback;
    ros::Subscriber anyJoySub = nh.subscribe(pubTopic, 10, &AnyJoyCallback::callback, &anyJoyCallback);
  while(ros::Time::now() < twoSecs) {
    joyPublisher.publish(joyMsg);
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }
  EXPECT_EQ(anyJoyCallback.getEntry(), -value/1024);
}


class AnyJoyCommandCallback {
public:
  AnyJoyCommandCallback(const std::string& command)
  : command_(command),
    hasCommand_(false) {
  };
  void callback(const joy_manager_msgs::AnyJoyConstPtr& msg) {
    for (std::string command : msg->commands) {
      if (command == command_) {
        hasCommand_ = true;
        break;
      }
    }
  }
  bool hasCommandArrived() {
    return hasCommand_;
  }
private:
  std::string command_;
  bool hasCommand_;
};


/* Test the hri user-interface by sending a joy message
 * containing the button assigned for a soft emergency stop
 * and test if it is forwarded as a command in the AnyJoy message.
 */
TEST(HRIUserInterface, SoftEmergencyStop)
{
  ros::NodeHandle nh("~");

  // initialize publisher
  std::string subTopic, pubTopic;
  nh.getParam("/hri_user_interface/subscribers/hri_joy/topic", subTopic);
  nh.getParam("/hri_user_interface/publishers/anyJoy/topic", pubTopic);
  if (subTopic.empty() || pubTopic.empty())
    FAIL();
  ros::Publisher joyPublisher = nh.advertise<sensor_msgs::Joy>(subTopic, 1);
  sensor_msgs::Joy joyMsg;
  joyMsg.header.stamp = ros::Time::now();
  joyMsg.axes.assign(6, 0.0);
  joyMsg.buttons = {0, 0, 0, 0, 0, 0, 1, 0};
  AnyJoyCommandCallback anyJoyCommandCallback("soft_emcy_stop");
  ros::Subscriber anyJoySub = nh.subscribe(pubTopic, 10, &AnyJoyCommandCallback::callback, &anyJoyCommandCallback);
  ros::Time twoSecs = ros::Time::now() + ros::Duration(2.0);
  while(ros::Time::now() < twoSecs) {
    joyPublisher.publish(joyMsg);
    ros::spinOnce();
  }
  EXPECT_TRUE(anyJoyCommandCallback.hasCommandArrived());
}


/* Test the hri user-interface by sending a hri_safety_sense/emergency_stop message
 * for a hard emergency stop and test if it is forwarded as a command in the AnyJoy message.
 */
//TEST(HRIUserInterface, HardEmergencyStop) TODO: Enable again when false positives have been fixed!
//{
//  ros::NodeHandle nh("~");
//
//  // initialize publisher
//  std::string subTopic, pubTopic;
//  nh.getParam("/hri_user_interface/subscribers/hri_emergency_stop/topic", subTopic);
//  nh.getParam("/hri_user_interface/publishers/anyJoy/topic", pubTopic);
//  if (subTopic.empty() || pubTopic.empty())
//    FAIL();
//  ros::Publisher emcyPublisher = nh.advertise<hri_safety_sense::EstopStatus>(subTopic, 1);
//  hri_safety_sense::EstopStatus emergencyMsg;
//  emergencyMsg.EstopStatus = 1;
//  AnyJoyCommandCallback anyJoyCommandCallback("hard_emcy_stop");
//  ros::Subscriber anyJoySub = nh.subscribe(pubTopic, 10, &AnyJoyCommandCallback::callback, &anyJoyCommandCallback);
//  ros::Time twoSecs = ros::Time::now() + ros::Duration(2.0);
//  while(ros::Time::now() < twoSecs) {
//    emcyPublisher.publish(emergencyMsg);
//    ros::spinOnce();
//  }
//  EXPECT_TRUE(anyJoyCommandCallback.hasCommandArrived());
//}


