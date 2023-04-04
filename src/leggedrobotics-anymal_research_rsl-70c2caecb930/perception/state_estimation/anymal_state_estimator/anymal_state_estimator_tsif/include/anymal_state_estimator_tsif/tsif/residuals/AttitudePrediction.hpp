
/*!
 * @file    AttitudePrediction.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <tsif/residuals/attitude_findif.h>

namespace tsif {

template <int Y,           // innovation
          int PHI_IB,      // orientation base to odom
          int B_OMEGA_IB>  // angular velocity base to odom in base
using AttitudePredictionBase = AttitudeFindif<Y, PHI_IB, B_OMEGA_IB>;

// residual implementing a prediction of the attitude from angular velocity state
template <int PHI_IB, int B_OMEGA_IB>
class AttitudePrediction : public AttitudePredictionBase<0, PHI_IB, B_OMEGA_IB> {
 public:
  using Base = AttitudePredictionBase<0, PHI_IB, B_OMEGA_IB>;
  using Previous = typename Base::Previous;
  using Current = typename Base::Current;
  using Output = typename Base::Output;

  using Base::dt_;

  AttitudePrediction() : Base() {};
  ~AttitudePrediction() override = default;

  int Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    // euler forward integration of the orientation
    ext.template Get<PHI_IB>() = Exp(delta_t * pre.template Get<B_OMEGA_IB>()) * pre.template Get<PHI_IB>();
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef /*pre*/, const typename Current::CRef /*cur*/) override {
    // compute weight with time scaling
    double weight = w_ / sqrt(dt_);
    this->AddWeight(weight,out,J_pre,J_cur);
  }

  int LoadParameters(const ros::NodeHandle& handle, const std::string& id) {
    w_ = param_io::param<double>(handle, id + std::string{"/w"}, 1.);
    return 0;
  }

 protected:
  using Base::w_;
};
}  // namespace tsif
