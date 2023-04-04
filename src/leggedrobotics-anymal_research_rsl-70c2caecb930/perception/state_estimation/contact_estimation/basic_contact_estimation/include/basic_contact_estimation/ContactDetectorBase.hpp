/*
 * ContactDetectorBase.hpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <kindr/Core>

#include <boost/thread.hpp>

#include <string>

namespace basic_contact_estimation {

class ContactDetectorBase {
 public:
  // WARNING: right now, modules defining their own ContactStateEnum (e.g. in a romo::RobotDescription) rely on the following enum to be
  // equivalent
  enum class ContactState : unsigned int { OPEN = 0, CLOSED, SLIPPING };

 public:
  explicit ContactDetectorBase(std::string name);
  virtual ~ContactDetectorBase();

  virtual bool initialize(double /*dt*/) { return true; };
  virtual bool advance(double /*dt*/) { return true; };

  const std::string& getName() const { return name_; }

  void setWrench(const kindr::WrenchD& wrench) {
    boost::unique_lock<boost::shared_mutex> lock{mutexWrench_};
    wrench_ = wrench;
  }

  void setForce(const kindr::Force3D& force) {
    boost::unique_lock<boost::shared_mutex> lock{mutexWrench_};
    wrench_.setForce(force);
  }

  void setTorque(const kindr::Torque3D& torque) {
    boost::unique_lock<boost::shared_mutex> lock{mutexWrench_};
    wrench_.setTorque(torque);
  }

  virtual ContactState getContactState() const { return state_; }

 protected:
  std::string name_;
  boost::atomic<ContactState> state_;
  kindr::WrenchD wrench_;
  boost::shared_mutex mutexWrench_;
};

}  // namespace basic_contact_estimation
