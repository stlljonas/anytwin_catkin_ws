

#pragma once

#include <basic_contact_estimation/ContactDetectorThresholding.hpp>

namespace anymal_state_estimator {

class ContactDetectorThresholdingRos : public basic_contact_estimation::ContactDetectorThresholding {
public:
  using Base = basic_contact_estimation::ContactDetectorThresholding;

  ContactDetectorThresholdingRos() = delete;
  ContactDetectorThresholdingRos(
    const ros::NodeHandle& nh,
    std::string name) :
      Base(std::move(name))
  {
    std::string prefix = name_;
    config_.isThresholdingForce_ = true;
    config_.isThresholdingTorque_ = false;

    std::string method = param_io::param<std::string>(nh, prefix + std::string{"/method_force"}, "zonly");

    if (method == std::string{"norm"}) {
      config_.forceThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::NORM;
    } else if (method == std::string{"zonly"}) {
      config_.forceThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::ZONLY;
    } else if (method == std::string{"xyonly"}) {
      config_.forceThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::XYONLY;
    } else if (method == std::string{"zclamped"}) {
      config_.forceThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::ZCLAMPED;
    } else if (method == std::string{"all"}) {
      config_.forceThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::ALL;
    } else if (method == std::string{"allclamped"}) {
      config_.forceThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::ALLCLAMPED;
    }

    method = param_io::param<std::string>(nh, prefix + std::string{"/method_torque"}, "zonly");
    if (method == std::string{"norm"}) {
      config_.torqueThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::NORM;
    } else if (method == std::string{"zonly"}) {
      config_.torqueThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::ZONLY;
    } else if (method == std::string{"zclamped"}) {
      config_.torqueThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::ZCLAMPED;
    } else if (method == std::string{"all"}) {
      config_.torqueThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::ALL;
    } else if (method == std::string{"allclamped"}) {
      config_.torqueThresholdingMethod_ =
          basic_contact_estimation::ContactDetectorThresholding::ThresholdingMethod::ALLCLAMPED;
    }

    config_.thresholdForceNorm_ =
        param_io::param(nh, prefix + std::string{"/threshold_force_norm"}, 999.0);
    config_.thresholdTorqueNorm_ =
        param_io::param(nh, prefix + std::string{"/threshold_torque_norm"}, 999.0);

    param_io::getParam<double>(nh, prefix + std::string{"/threshold_force/x"},
                               config_.thresholdForce_.x());
    param_io::getParam<double>(nh, prefix + std::string{"/threshold_force/y"},
                               config_.thresholdForce_.y());
    param_io::getParam<double>(nh, prefix + std::string{"/threshold_force/z"},
                               config_.thresholdForce_.z());

    param_io::getParam<double>(nh, prefix + std::string{"/upper_threshold_force/x"},
                               config_.upperThresholdForce_.x());
    param_io::getParam<double>(nh, prefix + std::string{"/upper_threshold_force/y"},
                               config_.upperThresholdForce_.y());
    param_io::getParam<double>(nh, prefix + std::string{"/upper_threshold_force/z"},
                               config_.upperThresholdForce_.z());

    param_io::getParam<double>(nh, prefix + std::string{"/threshold_torque/x"},
                               config_.thresholdTorque_.x());
    param_io::getParam<double>(nh, prefix + std::string{"/threshold_torque/y"},
                               config_.thresholdTorque_.y());
    param_io::getParam<double>(nh, prefix + std::string{"/threshold_torque/z"},
                               config_.thresholdTorque_.z());

    param_io::getParam<double>(nh, prefix + std::string{"/upper_threshold_torque/x"},
                               config_.upperThresholdTorque_.x());
    param_io::getParam<double>(nh, prefix + std::string{"/upper_threshold_torque/y"},
                               config_.upperThresholdTorque_.y());
    param_io::getParam<double>(nh, prefix + std::string{"/upper_threshold_torque/z"},
                               config_.upperThresholdTorque_.z());

    MELO_INFO_STREAM("Setting config for contact detector " << name_ << ":\n"
                                                                << config_);
  }

  ~ContactDetectorThresholdingRos() override = default;

};

}

