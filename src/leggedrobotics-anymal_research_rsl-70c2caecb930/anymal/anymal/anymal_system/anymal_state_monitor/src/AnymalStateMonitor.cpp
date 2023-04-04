#include "anymal_state_monitor/AnymalStateMonitor.h"

namespace anymal_state_monitor {

AnymalStateMonitor::AnymalStateMonitor(ros::NodeHandle nh) : nh_(nh) {
  // get params
  std::string topicGetRobotState;
  param_io::getParam(nh_, "services/servers/get_robot_state/service", topicGetRobotState);

  std::string topicGetCurrentStates;
  param_io::getParam(nh_, "services/clients/get_current_states/service", topicGetCurrentStates);

  std::vector<std::string> components_;
  param_io::getParam(nh_, "components", components_);

  // init notification subscriber
  notificationSubscriber_ = std::make_shared<notification::NotificationSubscriber>("operator_screen", nh_,
                                             boost::bind(&AnymalStateMonitor::notificationCallback, this, _1));
  // init notification publisher
  notificationPublisher_ = std::make_shared<notification::NotificationPublisher>("robot_visualizer", nh_, true);

  // initialize states with already known objects
  for (auto item : components_) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    notification::Notification state(header, notification::Level::LEVEL_DEBUG, 0, "", "", {}, item);
    states_.push_back(state);
  }

  // wait to give the notification time to initialize
  ros::Duration(0.5).sleep();
}

AnymalStateMonitor::~AnymalStateMonitor() {
  notificationSubscriber_->shutdown();
}

void AnymalStateMonitor::notificationCallback(const notification::Notification &notification) {
  notification::Notification* newNotification = nullptr;
  bool isNewObject = true;
  bool isNewNotification = false;
  for (auto &item : states_) {
    if (item.id_.compare(notification.id_) == 0) {
      if (item.getLevel() != notification.getLevel() ||
          item.name_.compare(notification.name_) != 0 ||
          item.description_.compare(notification.description_) != 0 ||
          item.code_ != notification.code_) {
        item = notification;
        newNotification = &item;
        isNewNotification = true;
      }
      isNewObject = false;
      break;
    }
  }
  if (isNewObject) {
    states_.push_back(notification);
    newNotification = &states_.back();
    isNewNotification = true;
  }
  if (isNewNotification) {
    if (newNotification != nullptr) {
      notificationPublisher_->notify(newNotification->getLevel(), newNotification->name_,
                                     newNotification->description_, newNotification->id_);
    }
  }
}

} // namespace
