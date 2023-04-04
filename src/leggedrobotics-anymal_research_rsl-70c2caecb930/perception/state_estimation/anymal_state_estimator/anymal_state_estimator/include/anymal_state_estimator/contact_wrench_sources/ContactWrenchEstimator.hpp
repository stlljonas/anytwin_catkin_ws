/*
 * ContactWrenchManager.hpp
 *
 *  Created on: Jun 28, 2017
 *      Author: gehrinch
 */

#pragma once

#include <anymal_state_estimator/contact_wrench_sources/ContactWrenchInterface.hpp>
#include <memory>
#include <message_logger/message_logger.hpp>
#include <romo/ContactForceEstimation.hpp>


namespace anymal_state_estimator {

template <typename ConcreteDescription_, typename RobotState_>
class ContactWrenchEstimator : public ContactWrenchInterface {
 public:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;
  using WrenchShm = any_measurements::Wrench;
  using WrenchRos = geometry_msgs::WrenchStamped;
  using ContactEnum = typename RD::ContactEnum;

  using ContactForceEstimation = romo::ContactForceEstimation<ConcreteDescription_, RobotState_>;

  ContactWrenchEstimator() = delete;
  ContactWrenchEstimator(const std::string& name, const double maxTimeout, const ContactEnum contactEnum,
                         const RobotModel& model, std::shared_ptr<ContactForceEstimation> estimator,
                         bool useMeasuredAccelerations, bool updateEstimator = true)
      : ContactWrenchInterface(name, maxTimeout),
        robotModel_(model),
        estimator_(estimator),
        contactEnum_(contactEnum),
        useMeasuredAccelerations_(useMeasuredAccelerations),
        updateEstimator_(updateEstimator) {}

  ~ContactWrenchEstimator() override = default;

  void init() override {
    // do nothing
  }

  void update() override {
    if (updateEstimator_) {
      if (useMeasuredAccelerations_) {
        estimator_->estimate(robotModel_.getState().getJointAccelerations().toImplementation());
      } else {
        estimator_->estimate();
      }
    }
    if (!hasWrench_) {
      hasWrench_ = true;
    }

    wrench_.time_ = any_measurements_ros::fromRos(ros::Time::now());
    wrench_.wrench_.setForce(kindr::WrenchD::Force(robotModel_.getContactContainer()[contactEnum_]->getForce()));
  }

 protected:
  const RobotModel& robotModel_;
  std::shared_ptr<ContactForceEstimation> estimator_;
  const ContactEnum contactEnum_;
  const bool updateEstimator_;
  bool useMeasuredAccelerations_;
};

}  // namespace anymal_state_estimator
