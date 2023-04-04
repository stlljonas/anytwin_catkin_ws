
/*!
 * @file    AngularVelocityUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <message_logger/message_logger.hpp>
#include <tsif/residuals/gyroscope_update.h>

namespace tsif {

template <int Y,           // innovation
          int B_OMEGA_IB,  // angular velocity base to odom in base
          int B_B_OMEGA>   // IMU angular velocity bias in base
using AngularVelocityUpdateBase = GyroscopeUpdate<Y, B_OMEGA_IB, B_B_OMEGA>;

template <int B_OMEGA_IB, int B_B_OMEGA>
class AngularVelocityUpdate : public AngularVelocityUpdateBase<0, B_OMEGA_IB, B_B_OMEGA> {
 public:
  using Base = AngularVelocityUpdateBase<0, B_OMEGA_IB, B_B_OMEGA>;

  using Current =  typename Base::Current;
  using Previous =  typename Base::Previous;
  using Output =  typename Base::Output;

  using Base::dt_;
  bool bias_estimation_enabled_{true};

  AngularVelocityUpdate() : Base() {};
  ~AngularVelocityUpdate() override = default;

  int Extrapolate(const typename Current::CRef pre, typename Current::Ref ext, const double& delta_t) {
    // forward the latest bias corrected angular velocity measurement
    ext.template Get<B_OMEGA_IB>() = pre.template Get<B_OMEGA_IB>();
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef /*pre*/, const typename Current::CRef cur) override {
    if(!bias_estimation_enabled_){
      J_cur.block<Output::Dim(), Current::template GetElementDim<B_B_OMEGA>()>(out.Start(0), cur.Start(B_B_OMEGA)).setZero();
    }

    // compute weight using huber loss function
    double weight = w_ * sqrt(dt_);
    const double norm = out.template Get<0>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[AngularVelocityUpdate] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if (norm > huber_threshold_) {
      weight *= sqrt(2 * huber_threshold_ * (norm - 0.5 * huber_threshold_)) / norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[AngularVelocityUpdate] Outlier detected!");
    }

    this->AddWeight(weight,out,J_pre,J_cur);
  }

  int LoadParameters(const ros::NodeHandle& handle, const std::string& id) {
    w_ = param_io::param<double>(handle, id + std::string{"/w"}, 1.);
    print_diagnostics_ = param_io::param<bool>(handle, id + std::string{"/print_diagnostics"}, false);
    huber_threshold_ = param_io::param<double>(handle, id + std::string{"/huber_threshold"}, std::numeric_limits<double>::max());
    return 0;
  }

 protected:
  using Base::w_;

  double max_norm_{0.};
  double huber_threshold_{std::numeric_limits<double>::max()};
  bool print_diagnostics_{false};
};
}  // namespace tsif
