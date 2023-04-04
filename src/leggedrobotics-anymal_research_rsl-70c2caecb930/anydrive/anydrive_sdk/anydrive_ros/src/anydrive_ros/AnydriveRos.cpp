#include "anydrive_ros/AnydriveRos.hpp"

namespace anydrive_ros {

AnydriveRos::AnydriveRos(ros::NodeHandle& nh, const unsigned int maxPublishMessageBufferSize)
    : Anydrive(),
      nh_(nh),
      readingMsgUpdated_(false),
      readingExtendedMsgUpdated_(false),
      maxPublishMessageBufferSize_(maxPublishMessageBufferSize) {}

anydrive_msgs::Reading AnydriveRos::getReadingMsg() {
  std::lock_guard<std::recursive_mutex> lock(readingMsgMutex_);
  if (!readingMsgUpdated_) {
    anydrive::Reading reading;
    const anydrive::ReadingExtended readingExtended = getReading();
    reading.setCommanded(readingExtended.getCommanded());
    reading.setState(static_cast<anydrive::State>(readingExtended.getState()));
    writeToMessage(readingMsg_, reading);
    readingMsgUpdated_ = true;
  }
  return readingMsg_;
}

anydrive_msgs::ReadingExtended AnydriveRos::getReadingExtendedMsg() {
  std::lock_guard<std::recursive_mutex> lock(readingExtendedMsgMutex_);
  if (!readingExtendedMsgUpdated_) {
    writeToMessage(readingExtendedMsg_, getReading());
    readingExtendedMsgUpdated_ = true;
  }
  return readingExtendedMsg_;
}

void AnydriveRos::startupWithoutCommunication() {
  ANYDRIVE_NAMED_INFO("Starting up ANYdrive ROS interface ...");

  param_io::getParam(nh_, "ros_prefix", rosPrefix_);
  param_io::getParam(nh_, "subscribers/command/enable", runCommandSubscriber_);
  param_io::getParam(nh_, "publishers/reading/enable", runReadingPublisher_);
  param_io::getParam(nh_, "publishers/reading_extended_throttled/enable", runReadingExtendedThrottledPublisher_);
  param_io::getParam(nh_, "publishers/reading_extended_throttled/decimation", readingExtendedThrottledPublisherDecimation_);

  if (runCommandSubscriber_) {
    commandSubscriber_ =
        nh_.subscribe(rosPrefix_ + "/" + getName() + "/" + param_io::param<std::string>(nh_, "subscribers/command/topic", "command"),
                      param_io::param<uint32_t>(nh_, "subscribers/command/queue_size", 1), &AnydriveRos::commandCb, this);
  }

  if (runReadingPublisher_) {
    readingPublisher_.reset(new any_node::ThreadedPublisher<anydrive_msgs::Reading>(
        nh_.advertise<anydrive_msgs::Reading>(
            rosPrefix_ + "/" + getName() + "/" + param_io::param<std::string>(nh_, "publishers/reading/topic", "reading"),
            param_io::param<uint32_t>(nh_, "publishers/reading/queue_size", 1),
            param_io::param<bool>(nh_, "publishers/reading/latch", false)),
        maxPublishMessageBufferSize_, false));
  }

  if (runReadingExtendedThrottledPublisher_) {
    readingExtendedThrottledPublisher_.reset(new any_node::ThreadedPublisher<anydrive_msgs::ReadingExtended>(
        nh_.advertise<anydrive_msgs::ReadingExtended>(
            rosPrefix_ + "/" + getName() + "/" +
                param_io::param<std::string>(nh_, "publishers/reading_extended_throttled/topic", "reading_extended_throttled"),
            param_io::param<uint32_t>(nh_, "publishers/reading_extended_throttled/queue_size", 1),
            param_io::param<bool>(nh_, "publishers/reading_extended_throttled/latch", false)),
        maxPublishMessageBufferSize_, false));
  }

  Anydrive::startupWithoutCommunication();
}

void AnydriveRos::updateProcessReading() {
  Anydrive::updateProcessReading();

  // Reset the messages.
  readingMsgUpdated_ = false;
  readingExtendedMsgUpdated_ = false;

  // Publish reading.
  if (runReadingPublisher_) {
    readingPublisher_->publish(getReadingMsg());
  }

  // Publish extended reading.
  if (runReadingExtendedThrottledPublisher_ && readingExtendedThrottledPublisherCounter_ == readingExtendedThrottledPublisherDecimation_) {
    readingExtendedThrottledPublisher_->publish(getReadingExtendedMsg());
    readingExtendedThrottledPublisherCounter_ = 0;
  }
  readingExtendedThrottledPublisherCounter_++;
}

void AnydriveRos::shutdownWithoutCommunication() {
  Anydrive::shutdownWithoutCommunication();

  ANYDRIVE_NAMED_INFO("Shutting down ANYdrive ROS interface ...");

  if (runCommandSubscriber_) {
    commandSubscriber_.shutdown();
  }
  if (runReadingPublisher_) {
    readingPublisher_->shutdown();
  }
  if (runReadingExtendedThrottledPublisher_) {
    readingExtendedThrottledPublisher_->shutdown();
  }
}

void AnydriveRos::sendRos() {
  if (runReadingPublisher_) {
    readingPublisher_->sendRos();
  }

  if (runReadingExtendedThrottledPublisher_) {
    readingExtendedThrottledPublisher_->sendRos();
  }
}

void AnydriveRos::commandCb(const anydrive_msgs::CommandConstPtr& commandMsg) {
  anydrive::Command command;
  readFromMessage(command, *commandMsg);
  stageCommand(command);
}

}  // namespace anydrive_ros
