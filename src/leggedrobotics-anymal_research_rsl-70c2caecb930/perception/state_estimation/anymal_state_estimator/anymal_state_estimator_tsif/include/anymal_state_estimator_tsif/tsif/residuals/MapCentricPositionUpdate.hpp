
/*!
 * @file    MapCentricPositionUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <message_logger/message_logger.hpp>
#include <tsif/residual.h>

namespace tsif {

// measurement class for the measurement frame position in map frame
class MeasPosMapCentric : public ElementVector<Element<Vec3, 0>> {
 public:
  MeasPosMapCentric() : ElementVector<Element<Vec3, 0>>(Vec3(0, 0, 0)) {}
  explicit MeasPosMapCentric(const Vec3& J_r_JV) : ElementVector<Element<Vec3, 0>>(J_r_JV) {}
  const Vec3& GetPos() const { return Get<0>(); }
  Vec3& GetPos() { return Get<0>(); }
};

template <int Y,       // innovation
          int J_R_JB,  // base to map position in map frame
          int PHI_JB,  // base to map orientation
          int B_R_BV>  // position measurement frame to base in base
using MapCentricPositionUpdateBase =
    Residual<ElementVector<Element<Vec3, Y>>, ElementVector<>,
             ElementVector<Element<Vec3, J_R_JB>, Element<Quat, PHI_JB>, Element<Vec3, B_R_BV>>, MeasPosMapCentric>;

// residual implementing an update of the position in the map frame from a direct measurement
template <int J_R_JB, int PHI_JB, int B_R_BV>
class MapCentricPositionUpdate : public MapCentricPositionUpdateBase<0, J_R_JB, PHI_JB, B_R_BV> {
 public:
  using Base = MapCentricPositionUpdateBase<0, J_R_JB, PHI_JB, B_R_BV>;
  using Output = typename Base::Output;
  using Previous = typename Base::Previous;
  using Current = typename Base::Current;

  using Base::meas_;

  MapCentricPositionUpdate(): Base(false, false, false) {}
  ~MapCentricPositionUpdate() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    // compare transformed measurement to filter state
    out.template Get<0>() = cur.template Get<J_R_JB>() +
                            cur.template Get<PHI_JB>().toRotationMatrix() * cur.template Get<B_R_BV>() -
                            meas_->GetPos();
    return 0;
  }

  int JacPre(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    // has dimension zero
    return 0;
  }

  int JacCur(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    const Mat3 C_JB = cur.template Get<PHI_JB>().toRotationMatrix();
    this->template SetJacCur<0, J_R_JB>(J, cur, Mat3::Identity());
    this->template SetJacCur<0, PHI_JB>(J, cur, -SSM(C_JB * cur.template Get<B_R_BV>()));
    this->template SetJacCur<0, B_R_BV>(J, cur, C_JB);
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    // compute weight using huber loss function
    double weight = w_;
    const double norm = out.template Get<0>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[MapCentricPositionUpdate] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if (norm > huber_threshold_) {
      weight *= sqrt(2 * huber_threshold_ * (norm - 0.5 * huber_threshold_)) / norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[MapCentricPositionUpdate] Outlier detected!");
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
