
/*!
 * @file    ExtFrameCentricAttitudeUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    April, 2018
 */

#pragma once

#include <tsif/residual.h>
#include <message_logger/message_logger.hpp>
#include <any_node/Param.hpp>

namespace tsif{

//measurement class for for attitude in external frame
class MeasAttExtFrameCentric: public ElementVector<Element<Quat,0>>{
 public:
  MeasAttExtFrameCentric(): ElementVector<Element<Quat,0>>(Quat(1,0,0,0)){}
  MeasAttExtFrameCentric(const Quat& q_JV): ElementVector<Element<Quat,0>>(q_JV){}
  const Quat& GetAtt() const{
    return Get<0>();
  }
  Quat& GetAtt(){
    return Get<0>();
  }
};

template<int PHI_JB,//base to ext frame orientation
         int PHI_BV,//measurement frame to ext frame orientation
         int Y=0>//innovation
using ExtFrameCentricAttitudeUpdateBase = Residual<ElementVector<Element<Vec3,Y>>,
                                                   ElementVector<>,
                                                   ElementVector<Element<Quat,PHI_JB>,
                                                                 Element<Quat,PHI_BV>>,
                                                   MeasAttExtFrameCentric>;

//residual implementing an update of the orientation in an external frame from direct measurement
template<int PHI_JB,int PHI_BV, int Y=0>
class ExtFrameCentricAttitudeUpdate: public ExtFrameCentricAttitudeUpdateBase<PHI_JB,PHI_BV,Y>{
 public:

  typedef ExtFrameCentricAttitudeUpdateBase<PHI_JB,PHI_BV,Y> Base;
  typedef typename Base::Output Output;
  typedef typename Base::Previous Previous;
  typedef typename Base::Current Current;

  using Base::meas_;

  ExtFrameCentricAttitudeUpdate(): Base(false,false,false) {}
  ~ExtFrameCentricAttitudeUpdate() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //compare transformed measurement to filter state
    out.template Get<Y>() = Log(cur.template Get<PHI_JB>()*cur.template Get<PHI_BV>()*meas_->GetAtt().inverse());
    return 0;
  }

  int JacPre(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //has dimension 0
    J.setZero();
    return 0;
  }

  int JacCur(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    J.setZero();
    const Vec3 y = Log(cur.template Get<PHI_JB>()*cur.template Get<PHI_BV>()*meas_->GetAtt().inverse());
    const Mat3 C_JB =  cur.template Get<PHI_JB>().toRotationMatrix();
    const Mat3 G_inv = GammaMat(y).inverse();
    this->template SetJacCur<Y,PHI_JB>(J,cur,G_inv);
    this->template SetJacCur<Y,PHI_BV>(J,cur,G_inv*C_JB);
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //compute weight using huber loss function
    double weight = w_;
    const double norm = out.template Get<Y>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[ExtFrameCentricAttitudeUpdate] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if(norm > huber_threshold_) {
      weight *= sqrt(2*huber_threshold_ * (norm - 0.5 * huber_threshold_))/norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[ExtFrameCentricAttitudeUpdate] Ooutlier detected!");
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

  double huber_threshold_{std::numeric_limits<double>::max()};
  bool print_diagnostics_{false};
  double max_norm_{0.};
};

} // namespace tsif
