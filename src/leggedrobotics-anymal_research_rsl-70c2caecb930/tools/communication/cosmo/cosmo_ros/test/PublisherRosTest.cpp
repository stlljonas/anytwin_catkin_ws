/*
 * PublisherRosTest.cpp
 *
 *  Created on: Jun 29, 2017
 *      Author: gehrinch
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cosmo_ros/cosmo_ros.hpp>

#include "UnitTestShmMsg.hpp"

namespace cosmo_ros {

class TestObject {
 public:
  void callback(const UnitTestShmMsg& msg) {}
};

}  // namespace cosmo_ros

TEST(PublisherRosTest, helper_methods) {  // NOLINT
  ros::NodeHandle nodeHandle("~");

  using MsgShm = cosmo_ros::UnitTestShmMsg;
  using MsgRos = cosmo_ros::UnitTestRosMsg;

  auto pub = cosmo_ros::advertiseShmRos<MsgShm, MsgRos, cosmo_ros::ConversionTraits>(nodeHandle, "test", "/test");

  auto options = std::make_shared<cosmo_ros::PublisherRosOptions>("test", nodeHandle);
  auto optionsPub = cosmo_ros::advertiseShmRos<MsgShm, MsgRos, cosmo_ros::ConversionTraits>(options);

  auto defaultOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("test", nodeHandle);
  defaultOptions->topic_ = "haha";
  defaultOptions->rosLatch_ = false;
  defaultOptions->rosQueueSize_ = 1;
  auto defaultPub = cosmo_ros::advertiseShmRos<MsgShm, MsgRos, cosmo_ros::ConversionTraits>("mypublisher", defaultOptions);

  ASSERT_EQ(std::string{"/test_cosmo_ros_node/my_topic"}, defaultOptions->topic_);
  ASSERT_EQ(true, defaultOptions->rosLatch_);
  ASSERT_EQ(99u, defaultOptions->rosQueueSize_);
}

TEST(SubscriberRosTest, helper_methods) {  // NOLINT
  ros::NodeHandle nodeHandle("~");

  using MsgShm = cosmo_ros::UnitTestShmMsg;
  using MsgRos = cosmo_ros::UnitTestRosMsg;

  cosmo_ros::TestObject testObj;

  auto defaultOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<MsgShm>>(
      "/test", std::bind(&cosmo_ros::TestObject::callback, &testObj, std::placeholders::_1), nodeHandle);
  defaultOptions->topic_ = "haha";
  defaultOptions->rosQueueSize_ = 1;

  auto defaultPub = cosmo_ros::subscribeShmRos<MsgShm, MsgRos, cosmo_ros::ConversionTraits>("mysubscriber", defaultOptions);

  ASSERT_EQ(std::string{"/test_cosmo_ros_node/my_topic"}, defaultOptions->topic_);
  ASSERT_EQ(77u, defaultOptions->rosQueueSize_);
}
