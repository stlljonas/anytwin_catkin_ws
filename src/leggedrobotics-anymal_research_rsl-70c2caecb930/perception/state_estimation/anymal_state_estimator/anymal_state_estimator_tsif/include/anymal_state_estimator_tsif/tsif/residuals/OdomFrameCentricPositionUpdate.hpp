
/*!
 * @file    OdomFrameCentricPositionUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    April, 2018
 */

#pragma once

#include <tsif/residuals/position_update.h>
#include <message_logger/message_logger.hpp>
#include <any_node/Param.hpp>

namespace tsif{

template<int I_R_IB,//position in odom to base in odom
         int PHI_IB,//orientation base to odom
         int I_R_IJ,//position odom to external frame in odom
         int PHI_IJ,//orientation external frame to odom
         int B_R_BV,//position base to measurement frame in base
         int Y=0>//innovation
using OdomFrameCentricPositionUpdateBase = PositionUpdate<Y, I_R_IB, PHI_IB, I_R_IJ, PHI_IJ, B_R_BV>;

//residual implementing an update of the position in odom and the ext to odom pose from a position measurement in the external frame
template<int I_R_IB, int PHI_IB, int I_R_IJ, int PHI_IJ, int B_R_BV, int Y=0>
class OdomFrameCentricPositionUpdate: public OdomFrameCentricPositionUpdateBase<I_R_IB,PHI_IB,I_R_IJ,PHI_IJ,B_R_BV,Y>{
 public:

  typedef OdomFrameCentricPositionUpdateBase<I_R_IB,PHI_IB,I_R_IJ,PHI_IJ,B_R_BV,Y> Base;
  typedef typename Base::Output Output;
  typedef typename Base::Previous Previous;
  typedef typename Base::Current Current;

  using Base::meas_;

  OdomFrameCentricPositionUpdate(): Base() {}
  ~OdomFrameCentricPositionUpdate() override = default;

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //compute weight using huber loss function
    double weight = w_;
    const double norm = out.template Get<Y>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[OdomFrameCentricPositionUpdate] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if(norm > huber_threshold_) {
      weight *= sqrt(2*huber_threshold_ * (norm - 0.5 * huber_threshold_))/norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[OdomFrameCentricPositionUpdate] Outlier detected!");
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
