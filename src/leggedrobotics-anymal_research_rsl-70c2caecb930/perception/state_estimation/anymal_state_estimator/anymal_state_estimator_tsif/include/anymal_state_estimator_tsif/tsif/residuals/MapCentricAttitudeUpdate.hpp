
/*!
 * @file    MapCentricAttitudeUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <message_logger/message_logger.hpp>
#include <tsif/residual.h>

namespace tsif {

// measurement class for for measurement frame to map orientation
class MeasAttMapCentric : public ElementVector<Element<Quat, 0>> {
 public:
  MeasAttMapCentric() : ElementVector<Element<Quat, 0>>(Quat(1, 0, 0, 0)) {}
  explicit MeasAttMapCentric(const Quat& q_JV) : ElementVector<Element<Quat, 0>>(q_JV) {}
  const Quat& GetAtt() const { return Get<0>(); }
  Quat& GetAtt() { return Get<0>(); }
};

template <int Y,       // innovation
          int PHI_JB,  // base to map orientation
          int PHI_BV>  // measurement frame to map orientation
using MapCentricAttitudeUpdateBase =
    Residual<ElementVector<Element<Vec3, Y>>, ElementVector<>,
             ElementVector<Element<Quat, PHI_JB>, Element<Quat, PHI_BV>>, MeasAttMapCentric>;

// residual implementing an update of the orientation in the map frame from a direct measurement
template <int PHI_JB, int PHI_BV>
class MapCentricAttitudeUpdate : public MapCentricAttitudeUpdateBase<0, PHI_JB, PHI_BV> {
 public:
  using Base = MapCentricAttitudeUpdateBase<0, PHI_JB, PHI_BV>;
  using Output = typename Base::Output;
  using Previous = typename Base::Previous;
  using Current = typename Base::Current;

  using Base::meas_;

  MapCentricAttitudeUpdate(): Base(false, false, false) {}
  ~MapCentricAttitudeUpdate() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    // compare transformed measurement to filter state
    out.template Get<0>() = Log(cur.template Get<PHI_JB>() * cur.template Get<PHI_BV>() * meas_->GetAtt().inverse());
    return 0;
  }

  int JacPre(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    // has dimension 0
    return 0;
  }

  int JacCur(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    J.setZero();
    const Vec3 y = Log(cur.template Get<PHI_JB>() * cur.template Get<PHI_BV>() * meas_->GetAtt().inverse());
    const Mat3 C_JB = cur.template Get<PHI_JB>().toRotationMatrix();
    const Mat3 G_inv = GammaMat(y).inverse();
    this->template SetJacCur<0, PHI_JB>(J, cur, G_inv);
    this->template SetJacCur<0, PHI_BV>(J, cur, G_inv * C_JB);
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    // compute weight using huber loss function
    double weight = w_;
    const double norm = out.template Get<0>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[MapCentricAttitudeUpdate] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if (norm > huber_threshold_) {
      weight *= sqrt(2 * huber_threshold_ * (norm - 0.5 * huber_threshold_)) / norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[MapCentricAttitudeUpdate] Outlier detected!");
    }
    // scale the innovation and jacobians
    this->AddWeight(weight,out,J_pre,J_cur);
  }

  int Extrapolate(const typename Current::CRef pre, typename Current::Ref ext, const double& delta_t) {
    // nothing to do
    return 0;
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
