
/*!
 * @file    OdomFrameCentricAttitudeUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    April, 2018
 */

#pragma once

#include <tsif/residuals/attitude_update.h>
#include <message_logger/message_logger.hpp>
#include <any_node/Param.hpp>

namespace tsif{

template<int PHI_IB,//orientation base to odom
         int PHI_IJ,//orientation ext to odom
         int PHI_BV,//orientation measurement frame to base
         int Y=0>//innovation
using OdomFrameCentricAttitudeUpdateBase = AttitudeUpdate<Y, PHI_IB, PHI_IJ, PHI_BV>;

//residual implementing an update of the base to odom and ext to odom orientations from a direct measurement of the orientation in an external frame
template<int PHI_IB, int PHI_IJ, int PHI_BV, int Y=0>
class OdomFrameCentricAttitudeUpdate: public OdomFrameCentricAttitudeUpdateBase<PHI_IB,PHI_IJ,PHI_BV,Y>{
 public:

  typedef OdomFrameCentricAttitudeUpdateBase<PHI_IB,PHI_IJ,PHI_BV,Y> Base;
  typedef typename Base::Output Output;
  typedef typename Base::Previous Previous;
  typedef typename Base::Current Current;

  using Base::meas_;

  OdomFrameCentricAttitudeUpdate(): Base() {}
  ~OdomFrameCentricAttitudeUpdate() override = default;

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //compute weight using huber loss function
    double weight = w_;
    const double norm = out.template Get<Y>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[OdomFrameCentricAttitudeUpdate] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if(norm > huber_threshold_) {
      weight *= sqrt(2*huber_threshold_ * (norm - 0.5 * huber_threshold_))/norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[OdomFrameCentricAttitudeUpdate] Outlier detected!");
    }
    //scale the innovation and jacobians
    this->AddWeight(weight,out,J_pre,J_cur);
  }

  int Extrapolate(const typename Current::CRef pre, typename Current::Ref ext, const double& delta_t){
    //nothing to do
    return 0;
  }

  int LoadParameters(const ros::NodeHandle& handle, const std::string& id){
    w_ = param_io::param<double>(handle, id+std::string{"/w"}, 1.);
    print_diagnostics_ = param_io::param<bool>(handle, id+std::string{"/print_diagnostics"}, false);
    huber_threshold_ = param_io::param<double>(handle, id+std::string{"/huber_threshold"}, std::numeric_limits<double>::max());
    return 0;
  }

 protected:
  using Base::w_;

  double max_norm_{0.};
  double huber_threshold_{std::numeric_limits<double>::max()};
  bool print_diagnostics_{false};
};

} // namespace tsif
