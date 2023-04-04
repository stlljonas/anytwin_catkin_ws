/*
 * ContactWrenchPublisher.hpp
 *
 *  Created on: Nov 14, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <boost/thread.hpp>
#include <chrono>
#include <cosmo_ros/cosmo_ros.hpp>
#include <memory>
#include <any_measurements/Wrench.hpp>
#include <any_measurements_ros/any_measurements_ros.hpp>

namespace anymal_state_estimator {

class ContactWrenchPublisher {
 public:
  using WrenchShm = any_measurements::Wrench;
  using WrenchRos = geometry_msgs::WrenchStamped;

  ContactWrenchPublisher() = delete;
  ContactWrenchPublisher(std::string name, ros::NodeHandle& nh) : name_(std::move(name)) {
    auto publishOptions = std::make_shared<cosmo_ros::PublisherRosOptions>(name_, nh);
    publishOptions->rosQueueSize_ = 10u;
    publishOptions->rosLatch_ = false;
    publishOptions->autoPublishRos_ = false;

    publisher_ =
        cosmo_ros::advertiseShmRos<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits>(name_, publishOptions);
  }

  virtual ~ContactWrenchPublisher() = default;

  ros::Time getRosTime() const {
    boost::unique_lock<boost::shared_mutex> lock(mutexWrench_);
    return any_measurements_ros::toRos(wrench_.time_);
  }

  any_measurements::Time getTime() const {
    boost::unique_lock<boost::shared_mutex> lock(mutexWrench_);
    return wrench_.time_;
  }

  WrenchShm getWrench() const {
    boost::unique_lock<boost::shared_mutex> lock(mutexWrench_);
    return wrench_;
  }
  void getWrench(WrenchShm& wrench) const {
    boost::unique_lock<boost::shared_mutex> lock(mutexWrench_);
    wrench = wrench_;
  }

  void setWrench(WrenchShm& wrench) {
    boost::unique_lock<boost::shared_mutex> lock(mutexWrench_);
    wrench_ = wrench;
  }

  bool publish(const std::chrono::microseconds& maxLockTime = std::chrono::microseconds{500}) {
    boost::unique_lock<boost::shared_mutex> lock(mutexWrench_);
    return publisher_->publish(wrench_, maxLockTime);
  }

  void sendRos() { publisher_->sendRos(); }

 protected:
  std::string name_;
  cosmo_ros::PublisherRosPtr<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits> publisher_;

  mutable boost::shared_mutex mutexWrench_;
  WrenchShm wrench_;
};

}  // namespace anymal_state_estimator
