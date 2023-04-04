
/*!
 * @file    ExtFrameCentricPoseUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    April, 2018
 */

#pragma once

#include <tsif/residual.h>
#include <message_logger/message_logger.hpp>
#include <any_node/Param.hpp>

namespace tsif{

//measurement class for position in extrnal frame
class MeasPosExtFrameCentric: public ElementVector<Element<Vec3,0>>{
 public:
  MeasPosExtFrameCentric(): ElementVector<Element<Vec3,0>>(Vec3(0,0,0)){}
  MeasPosExtFrameCentric(const Vec3& J_r_JV): ElementVector<Element<Vec3,0>>(J_r_JV){}
  const Vec3& GetPos() const{
    return Get<0>();
  }
  Vec3& GetPos(){
    return Get<0>();
  }
};

template<int J_R_JB,//base to ext position in ext frame
         int PHI_JB,//base to ext orientation
         int B_R_BV,//position measurement frame to base in base
         int Y>//innovation
using ExtFrameCentricPositionUpdateBase = Residual<ElementVector<Element<Vec3,Y>>,
                                              ElementVector<>,
                                              ElementVector<Element<Vec3,J_R_JB>,
                                                            Element<Quat,PHI_JB>,
                                                            Element<Vec3,B_R_BV>>,
                                              MeasPosExtFrameCentric>;

//residual implementing an update of the position in an external frame from a direct measurement
template<int J_R_JB, int PHI_JB, int B_R_BV, int Y=0>
class ExtFrameCentricPositionUpdate: public ExtFrameCentricPositionUpdateBase<J_R_JB,PHI_JB,B_R_BV,Y>{
 public:

  typedef ExtFrameCentricPositionUpdateBase<J_R_JB,PHI_JB,B_R_BV,Y> Base;
  typedef typename Base::Output Output;
  typedef typename Base::Previous Previous;
  typedef typename Base::Current Current;

  using Base::meas_;

  ExtFrameCentricPositionUpdate(): Base(false,false,false) {}
  ~ExtFrameCentricPositionUpdate() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //compare transformed measurement to filter state
    out.template Get<Y>() = cur.template Get<J_R_JB>() + cur.template Get<PHI_JB>().toRotationMatrix()*cur.template Get<B_R_BV>()
                            -meas_->GetPos();
    return 0;
  }

  int JacPre(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //has dimension zero
    J.setZero();
    return 0;
  }

  int JacCur(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    J.setZero();
    const Mat3 C_JB = cur.template Get<PHI_JB>().toRotationMatrix();
    this->template SetJacCur<Y,J_R_JB>(J,cur,Mat3::Identity());
    this->template SetJacCur<Y,PHI_JB>(J,cur,-SSM(C_JB*cur.template Get<B_R_BV>()));
    this->template SetJacCur<Y,B_R_BV>(J,cur,C_JB);
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //compute weight using huber loss function
    double weight = w_;
    const double norm = out.template Get<Y>().norm();
    if (print_diagnostics_ && norm > max_norm_) {
      MELO_INFO_STREAM("[ExtFrameCentricPositionUpdate] Max innovation norm = " << norm);
      max_norm_ = norm;
    }
    if(norm > huber_threshold_) {
      weight *= sqrt(2*huber_threshold_ * (norm - 0.5 * huber_threshold_))/norm;
      if (print_diagnostics_) MELO_INFO_STREAM("[ExtFrameCentricPositionUpdate] Outlier detected!");
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
