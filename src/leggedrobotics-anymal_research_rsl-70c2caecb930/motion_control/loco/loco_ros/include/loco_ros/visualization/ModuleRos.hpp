/*
 * ModuleRos.hpp
 *
 *  Created on: Oct 19, 2017
 *      Author: dbellicoso
 */

#pragma once

// stl
#include <vector>

// ros
#include <ros/ros.h>

class Publisher;

// TODO(scaron): move to loco_ros namespace

class ModuleRos {
 public:
  ModuleRos() {}
  virtual ~ModuleRos() = default;

  virtual bool initialize(ros::NodeHandle& /*nodeHandle*/, const std::string& /*topic*/) { return true; }

  virtual bool publish() = 0;

  virtual unsigned int getNumSubscribers() const {
    auto numSubs = 0;
    for (const auto& pub: publisherRefs_) {
      numSubs += pub.get().getNumSubscribers();
    }
    return numSubs;
  }

  virtual bool shutdown() {
    for (auto& pub : publisherRefs_) {
      pub.get().shutdown();
    }
    return true;
  }

 protected:
  bool isNotSubscribed() const { return getNumSubscribers() == 0u; }
  std::vector<std::reference_wrapper<ros::Publisher>> publisherRefs_;
};
