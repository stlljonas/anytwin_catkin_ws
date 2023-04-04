/*
 * ContactWrenchManager.hpp
 *
 *  Created on: Jun 28, 2017
 *      Author: gehrinch
 */

#pragma once

#include <anymal_state_estimator/contact_wrench_sources/ContactWrenchInterface.hpp>
#include <boost/thread.hpp>
#include <chrono>
#include <cosmo_ros/cosmo_ros.hpp>
#include <memory>

namespace anymal_state_estimator {

template <typename ConcreteDescription_, typename RobotState_>
class ContactWrenchReceiver : public ContactWrenchInterface {
 public:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;
  using BodyEnum = typename RD::BodyEnum;

  using WrenchShm = any_measurements::Wrench;
  using WrenchRos = geometry_msgs::WrenchStamped;

  ContactWrenchReceiver() = delete;
  ContactWrenchReceiver(const std::string& name, const BodyEnum bodyEnum, const double maxTimeout,
                        const RobotModel& robotModel, ros::NodeHandle& nh)
      : ContactWrenchInterface(name, maxTimeout), bodyEnum_(bodyEnum), robotModel_(robotModel) {
    auto subscribeOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<WrenchShm>>(
        name_, std::bind(&ContactWrenchReceiver::callback, this, std::placeholders::_1), nh);
    subscribeOptions->autoSubscribe_ = false;
    subscribeOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
    subscribeOptions->tryRosResubscribing_ = false;

    subscriber_ = cosmo_ros::subscribeShmRos<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits>(
        name_, subscribeOptions);
  }

  ~ContactWrenchReceiver() override = default;

  void callback(const WrenchShm& wrench) {
    boost::unique_lock<boost::shared_mutex> lock(mutexWrench_);
    if (!hasWrench_) {
      hasWrench_ = true;
    }

    kindr::RotationMatrixD rotWorldToBase(robotModel_.getOrientationWorldToBody(bodyEnum_));
    wrench_.time_ = wrench.time_;
    wrench_.wrench_.getForce() = rotWorldToBase.inverseRotate(wrench.wrench_.getForce());
    wrench_.wrench_.getTorque() = rotWorldToBase.inverseRotate(wrench.wrench_.getTorque());
  }

  void init() override { hasWrench_ = false; }

  void update() override { subscriber_->receive(); }

 protected:
  cosmo_ros::SubscriberRosPtr<WrenchShm, WrenchRos, any_measurements_ros::ConversionTraits> subscriber_;
  const BodyEnum bodyEnum_;
  const RobotModel& robotModel_;
};

}  // namespace anymal_state_estimator
