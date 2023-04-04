#pragma once

// std
#include <map>
#include <vector>

// ros
#include <ros/ros.h>

// signal relay msgs
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

namespace signal_relay {

/**
 * Signal Relay
 *  This class implements a ROS node which is intended to route reset signals between nodes that work together
 *  and in synch, as part of ANYmal's software stack.
 *
 *  Functionality:
 *  - Route reset signals received via topics to service calls.
 *  - Compute the probability of a point being dynamic.
 *  - Detect node status and send resets autonomously. (Planned)
 *
 *  Threading model:
 *    The node runs on ROS threads, which are spawned for every reset message received.
 */
class SignalRelay {
 public:
  explicit SignalRelay(ros::NodeHandle& nodeHandle);
  ~SignalRelay() = default;

  /**
   * @brief Read parameters from ROS server.
   *
   * @return true   If successful.
   */
  bool readParameters();

  /**
   * @brief Callback for reset messages.
   *
   * @param msg     Incoming reset message.
   * @param topic   Topic ID.
   */
  void resetMessageCb(const std_msgs::Empty::ConstPtr& msg, std::string topic);

 private:
  //! Queue size of reset subscribers.
  const int kQueueSize_ = 5;

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Collection of publishers, service clients and routes.
  std::map<std::string, ros::Publisher> publishers_;
  std::map<std::string, ros::ServiceClient> serviceClients_;
  std::map<std::string, std::vector<std::string>> topicRoutes_;
  std::map<std::string, std::vector<std::string>> serviceRoutes_;

  //! ROS subscribers.
  std::vector<ros::Subscriber> subscribers_;
};

}  // namespace signal_relay
