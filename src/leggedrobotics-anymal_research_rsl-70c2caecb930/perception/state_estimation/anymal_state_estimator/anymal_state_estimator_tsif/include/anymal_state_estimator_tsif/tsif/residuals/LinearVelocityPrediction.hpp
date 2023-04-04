
/*!
 * @file    LinearVelocityPrediction.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <message_logger/message_logger.hpp>
#include <tsif/residuals/accelerometer_prediction.h>

namespace tsif {

template <int Y,           // innovation
          int PHI_IB,      // attitude base to odom
          int B_V_IM,      // linear velocity imu to odom in base
          int B_OMEGA_IB,  // angular velocity base to odom in base
          int B_B_F>       // imu linear acceleration bias in base
using LinearVelocityPredictionBase = AccelerometerPrediction<Y, B_V_IM, PHI_IB, B_OMEGA_IB, B_B_F>;

// residual implementing a prediction of the linear velocity from imu measurements
template <int PHI_IB, int B_V_IM, int B_OMEGA_IB, int B_B_F>
class LinearVelocityPrediction : public LinearVelocityPredictionBase<0, PHI_IB, B_V_IM, B_OMEGA_IB, B_B_F> {
 public:
  using  Base = LinearVelocityPredictionBase<0, PHI_IB, B_V_IM, B_OMEGA_IB, B_B_F>;
  using  Previous = typename Base::Previous;
  using  Current = typename Base::Current;
  using  Output = typename Base::Output;

  using Base::dt_;
  using Base::meas_;
  bool bias_estimation_enabled_{true};

  LinearVelocityPrediction() : Base() {};
  ~LinearVelocityPrediction() override = default;

  int Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    // euler forward integration of the linear velocity
    ext.template Get<B_V_IM>() =
        (Mat3::Identity() - SSM(delta_t * pre.template Get<B_OMEGA_IB>())) * pre.template Get<B_V_IM>() +
        delta_t * (meas_->GetAcc() - pre.template Get<B_B_F>() +
                   pre.template Get<PHI_IB>().inverse().toRotationMatrix() * g_);
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef /*cur*/) override {
    if(!bias_estimation_enabled_){
      J_pre.block<Output::Dim(), Previous::template GetElementDim<B_B_F>()>(out.Start(0), pre.Start(B_B_F)).setZero();
    }

    // compute weight using huber loss function and time scaling
    double weight = w_ / sqrt(dt_);
    const double norm = out.template Get<0>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[LinearVelocityPrediction] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if (norm > huber_threshold_) {
      weight *= sqrt(2 * huber_threshold_ * (norm - 0.5 * huber_threshold_)) / norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[LinearVelocityPrediction] Outlier detected!");
    }
    // scale the innovation and jacobians
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
  using Base::g_;

  double huber_threshold_{std::numeric_limits<double>::max()};
  bool print_diagnostics_{false};
  double max_norm_{0.};
};
}  // namespace tsif
