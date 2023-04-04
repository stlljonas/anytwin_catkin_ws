
/*!
 * @file    LandmarkInOdomUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <message_logger/message_logger.hpp>
#include <tsif/residual.h>

namespace tsif {

// measurement class to hold base to landmark vector in base frame
class LandmarkInOdomMeasurement : public ElementVector<Element<Vec3, 0>> {
 public:
  LandmarkInOdomMeasurement() : ElementVector<Element<Vec3, 0>>(Vec3(.0, .0, .0)) {}
  LandmarkInOdomMeasurement(const Vec3& B_Ns) : ElementVector<Element<Vec3, 0>>(B_Ns) {}
  const Vec3& GetMeasurement() const { return Get<0>(); }
  Vec3& GetMeasurement() { return Get<0>(); }
};

template <int Y,       // contact point innovation
          int I_R_IB,  // position base to odom in odom
          int PHI_IB,  // orientation base to odom
          int I_P>     // landmark in odom
using LandmarkInOdomUpdateBase =
    Residual<ElementVector<Element<Vec3, Y>>, ElementVector<>,
             ElementVector<Element<Vec3, I_R_IB>, Element<Quat, PHI_IB>, Element<Vec3, I_P>>,
             LandmarkInOdomMeasurement>;

// landmark update residual comparing measured landmark points to contact points in the filter state
template <int I_R_IB, int PHI_IB, int I_P>
class LandmarkInOdomUpdate : public LandmarkInOdomUpdateBase<0, I_R_IB, PHI_IB, I_P> {
 public:
  using Base = LandmarkInOdomUpdateBase<0, I_R_IB, PHI_IB, I_P> ;
  using typename Base::Current;
  using typename Base::Output;
  using typename Base::Previous;

  using Base::dt_;
  using Base::meas_;

  LandmarkInOdomUpdate() : Base(false, false, false){};
  ~LandmarkInOdomUpdate() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef /*pre*/, const typename Current::CRef cur) override {
    // compare measured landmark points to filter state
    out.template Get<0>() = meas_->GetMeasurement() - cur.template Get<PHI_IB>().inverse().toRotationMatrix() *
                                                          (cur.template Get<I_P>() - cur.template Get<I_R_IB>());
    return 0;
  }

  int JacPre(MatRefX /*J*/, const typename Previous::CRef /*pre*/, const typename Current::CRef /*cur*/) override {
    // has dimension 0
    return 0;
  }

  int JacCur(MatRefX J, const typename Previous::CRef /*pre*/, const typename Current::CRef cur) override {
    J.setZero();
    const Mat3 C_BI = cur.template Get<PHI_IB>().inverse().toRotationMatrix();
    this->template SetJacCur<0, I_R_IB>(J, cur, C_BI);
    this->template SetJacCur<0, PHI_IB>(J, cur,
                                        -SSM(C_BI * (cur.template Get<I_P>() - cur.template Get<I_R_IB>())) * C_BI);
    this->template SetJacCur<0, I_P>(J, cur, -C_BI);
    return 0;
  }

  int Extrapolate(const typename Current::CRef pre, typename Current::Ref ext, const double& delta_t) {
    // nothing to do
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef /*pre*/, const typename Current::CRef /*cur*/) override {
    // compute weight using huber loss function
    double weight = w_;
    const double norm = out.template Get<0>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[LandmarkInOdomUpdate] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if (norm > huber_threshold_) {
      weight *= sqrt(2 * huber_threshold_ * (norm - 0.5 * huber_threshold_)) / norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[LandmarkInOdomUpdate] Outlier detected!");
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

  double huber_threshold_{std::numeric_limits<double>::max()};
  bool print_diagnostics_{false};
  double max_norm_{0.};
};
}  // namespace tsif
