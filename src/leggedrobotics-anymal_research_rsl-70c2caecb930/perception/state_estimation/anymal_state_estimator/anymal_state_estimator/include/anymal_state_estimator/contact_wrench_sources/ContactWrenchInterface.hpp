/*
 * ContactWrenchInterface.hpp
 *
 *  Created on: Nov 16, 2017
 *      Author: gehrinch
 */

#pragma once

#include <any_measurements/Wrench.hpp>
#include <any_measurements_ros/any_measurements_ros.hpp>

namespace anymal_state_estimator {

class ContactWrenchInterface {
 public:
  using WrenchShm = any_measurements::Wrench;
  using WrenchRos = geometry_msgs::WrenchStamped;

  ContactWrenchInterface() = delete;
  ContactWrenchInterface(std::string name, const double maxTimeout)
      : name_(std::move(name)), maxTimeout_(maxTimeout), hasWrench_(false) {}

  virtual ~ContactWrenchInterface() = default;

  virtual void init() = 0;
  virtual void update() = 0;

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

  virtual bool hasTimeout(const double lastTime) const {
    if (!hasWrench_) {
      return true;
    }
    else {
      return ((lastTime - wrench_.time_.toSeconds()) > maxTimeout_);
    }
  }

 protected:
  std::string name_;

  mutable boost::shared_mutex mutexWrench_;
  WrenchShm wrench_;
  const double maxTimeout_;
  bool hasWrench_;
};

}  // namespace anymal_state_estimator
