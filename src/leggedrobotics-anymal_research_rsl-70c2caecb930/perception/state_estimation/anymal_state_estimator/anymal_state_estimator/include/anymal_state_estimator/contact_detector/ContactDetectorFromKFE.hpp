/*
 * ContactDetectorFromKFE.hpp
 *
 *  Created on: Nov 28, 2017
 *      Author: markusta
 */

#pragma once

#include <basic_contact_estimation/ContactDetectorBase.hpp>

#include <string>

namespace anymal_state_estimator {

template <typename ConcreteDescription_, typename RobotState_>
class ContactDetectorFromKFE : public basic_contact_estimation::ContactDetectorBase {
 public:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;

  ContactDetectorFromKFE(const std::string& name, const RobotModel& robotModel, const double lowerThreshold,
                         const double upperThreshold, const unsigned int legId);

  ~ContactDetectorFromKFE() override = default;
  bool initialize(double dt) override;
  bool advance(double dt) override;

 protected:
  void detectContact();
  double computeKfeTorque();

  const RobotModel& robotModel_;

  double lowerThreshold_;
  double upperThreshold_;

  size_t legId_;
  unsigned int kfeJointUInt_;
};

}  // namespace anymal_state_estimator

#include <anymal_state_estimator/contact_detector/ContactDetectorFromKFE.tpp>
