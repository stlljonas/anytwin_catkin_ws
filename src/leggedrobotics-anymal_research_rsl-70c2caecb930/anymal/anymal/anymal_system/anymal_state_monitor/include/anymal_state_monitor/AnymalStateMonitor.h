#pragma once

#include <ros/ros.h>

#include <notification/NotificationPublisher.hpp>
#include <notification/NotificationSubscriber.hpp>

#include <param_io/get_param.hpp>

#include <std_srvs/Trigger.h>

#include <string>

namespace anymal_state_monitor {

class AnymalStateMonitor {
  const std::string TAG = "anymal_state_monitor";
public:
  AnymalStateMonitor(ros::NodeHandle nh);
  ~AnymalStateMonitor();

private:
  ros::NodeHandle nh_;

  std::shared_ptr<notification::NotificationSubscriber> notificationSubscriber_;
  std::shared_ptr<notification::NotificationPublisher> notificationPublisher_;

  std::vector<notification::Notification> states_;

  void notificationCallback(const notification::Notification &notification);
};

} // namespace


