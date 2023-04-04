#pragma once

#include <mutex>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <param_io/get_param.hpp>

#include <any_node/ThreadedPublisher.hpp>

#include <anydrive/Anydrive.hpp>

#include <anydrive_msgs/Command.h>
#include <anydrive_msgs/Reading.h>
#include <anydrive_msgs/ReadingExtended.h>

#include "anydrive_ros/conversions.hpp"

namespace anydrive_ros {

class AnydriveRos : public anydrive::Anydrive {
 protected:
  ros::NodeHandle& nh_;

  std::string rosPrefix_;

  bool runCommandSubscriber_ = true;

  bool runReadingPublisher_ = true;

  bool runReadingExtendedThrottledPublisher_ = true;
  unsigned int readingExtendedThrottledPublisherCounter_ = 0;
  unsigned int readingExtendedThrottledPublisherDecimation_ = 0;

  ros::Subscriber commandSubscriber_;
  any_node::ThreadedPublisherPtr<anydrive_msgs::Reading> readingPublisher_;
  any_node::ThreadedPublisherPtr<anydrive_msgs::ReadingExtended> readingExtendedThrottledPublisher_;

  std::recursive_mutex readingMsgMutex_;
  std::atomic<bool> readingMsgUpdated_;
  anydrive_msgs::Reading readingMsg_;
  std::recursive_mutex readingExtendedMsgMutex_;
  std::atomic<bool> readingExtendedMsgUpdated_;
  anydrive_msgs::ReadingExtended readingExtendedMsg_;

  //! Publish options.
  const unsigned int maxPublishMessageBufferSize_ = 0;

 public:
  AnydriveRos(ros::NodeHandle& nh, const unsigned int maxPublishMessageBufferSize);
  ~AnydriveRos() override = default;

  anydrive_msgs::Reading getReadingMsg();
  anydrive_msgs::ReadingExtended getReadingExtendedMsg();

  void startupWithoutCommunication() override;
  void updateProcessReading() override;
  void shutdownWithoutCommunication() override;

  void sendRos();

 protected:
  void commandCb(const anydrive_msgs::CommandConstPtr& commandMsg);
};

using AnydriveRosPtr = std::shared_ptr<AnydriveRos>;

}  // namespace anydrive_ros
