
/*!
 * @file    PositionPrediction.hpp
 * @author  Fabian Tresoldi
 * @date    June, 2017
 */

#pragma once

#include <any_node/Param.hpp>
#include <tsif/residual.h>

namespace tsif {

template <int I_R_IB,      // position base to odom in odom
          int PHI_IB,      // orientation base to odom
          int B_V_IM,      // imu to odom velocity in base
          int B_OMEGA_IB,  // angular velocity base to odom in base
          int B_R_BM>      // imu to base position in base
using PositionPredictionBase =
    Residual<ElementVector<Element<Vec3, 0>>,
             ElementVector<Element<Vec3, I_R_IB>, Element<Vec3, B_V_IM>, Element<Quat, PHI_IB>,
                           Element<Vec3, B_OMEGA_IB>, Element<Vec3, B_R_BM>>,
             ElementVector<Element<Vec3, I_R_IB>>, MeasEmpty>;

template <int I_R_IB, int PHI_IB, int B_V_IM, int B_OMEGA_IB, int B_R_BM>
class PositionPrediction : public PositionPredictionBase<I_R_IB, PHI_IB, B_V_IM, B_OMEGA_IB, B_R_BM> {
 public:
  using Base = PositionPredictionBase<I_R_IB, PHI_IB, B_V_IM, B_OMEGA_IB, B_R_BM>;
  using Base::dt_;
  using Output = typename Base::Output;
  using Previous = typename Base::Previous;
  using Current = typename Base::Current;

  PositionPrediction() : Base(true, true, true) {}
  ~PositionPrediction() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    out.template Get<0>() =
        cur.template Get<I_R_IB>() - pre.template Get<I_R_IB>() -
        dt_ * pre.template Get<PHI_IB>().toRotationMatrix() *
            (pre.template Get<B_V_IM>() - SSM(pre.template Get<B_OMEGA_IB>()) * pre.template Get<B_R_BM>());
    return 0;
  }
  int JacPre(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef /*cur*/) override {
    J.block<3, 3>(Output::Start(0), pre.Start(I_R_IB)) = -Mat3::Identity();
    J.block<3, 3>(Output::Start(0), pre.Start(B_V_IM)) = -pre.template Get<PHI_IB>().toRotationMatrix() * dt_;
    J.block<3, 3>(Output::Start(0), pre.Start(PHI_IB)) =
        SSM(dt_ * pre.template Get<PHI_IB>().toRotationMatrix() *
            (pre.template Get<B_V_IM>() - SSM(pre.template Get<B_OMEGA_IB>()) * pre.template Get<B_R_BM>()));
    J.block<3, 3>(Output::Start(0), pre.Start(B_OMEGA_IB)) =
        -dt_ * pre.template Get<PHI_IB>().toRotationMatrix() * SSM(pre.template Get<B_R_BM>());
    return 0;
  }
  int JacCur(MatRefX J, const typename Previous::CRef /*pre*/, const typename Current::CRef cur) override {
    J.block<3, 3>(Output::Start(0), cur.Start(I_R_IB)) = Mat3::Identity();
    return 0;
  }
  void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef /*pre*/, const typename Current::CRef /*cur*/) override {
    // compute weight using time scaling
    double weight = w_ / sqrt(dt_);
    // scale the innovation and jacobians with the standard weight
    this->AddWeight(weight,out,J_pre,J_cur);
  }
  int Extrapolate(const typename Previous::CRef pre, typename Current::Ref ext, const double& delta_t) {
    // euler forward integration of the linear velocity
    ext.template Get<I_R_IB>() =
        pre.template Get<I_R_IB>() +
        delta_t * pre.template Get<PHI_IB>().toRotationMatrix() *
            (pre.template Get<B_V_IM>() - SSM(pre.template Get<B_OMEGA_IB>()) * pre.template Get<B_R_BM>());
    return 0;
  }
  int LoadParameters(const ros::NodeHandle& handle, const std::string& id) {
    w_ = param_io::param<double>(handle, id + std::string{"/w"}, 1.);
    return 0;
  }

 protected:
  using Base::w_;
};

}  // namespace tsif
