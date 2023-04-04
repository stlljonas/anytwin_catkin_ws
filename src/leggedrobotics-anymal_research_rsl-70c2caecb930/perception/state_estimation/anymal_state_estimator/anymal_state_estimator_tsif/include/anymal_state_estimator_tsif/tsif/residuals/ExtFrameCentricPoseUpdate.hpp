
/*!
 * @file    ExtFrameCentricPoseUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    April, 2018
 */

#pragma once

#include <anymal_state_estimator_tsif/tsif/residuals/ExtFrameCentricPositionUpdate.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/ExtFrameCentricAttitudeUpdate.hpp>
#include <message_logger/message_logger.hpp>
#include <any_node/Param.hpp>

namespace tsif{

//measurement class for a pose measurement in an external frame
class MeasPoseExtFrameCentric: public ElementVector<Element<Vec3,0>,Element<Quat,1>>{
 public:
  MeasPoseExtFrameCentric(): ElementVector<Element<Vec3,0>,Element<Quat,1>>(Vec3(0,0,0),Quat(1,0,0,0)){}
  MeasPoseExtFrameCentric(const Vec3& J_r_JV, const Quat& q_JV): ElementVector<Element<Vec3,0>,Element<Quat,1>>(J_r_JV, q_JV){}
  const Vec3& GetPos() const{
    return Get<0>();
  }
  Vec3& GetPos(){
    return Get<0>();
  }
  const Quat& GetAtt() const{
    return Get<1>();
  }
  Quat& GetAtt(){
    return Get<1>();
  }
};

template<int J_R_JB,//external to base position in map frame
         int PHI_JB,//base to external frame orientation
         int B_R_BV,//position base to measurement frame in base
         int PHI_BV,//measurement frame to base orientation
         int Y_R=0,//linear innovation
         int Y_PHI=1>//angular innovation
using ExtFrameCentricPoseUpdateBase = Residual<ElementVector<Element<Vec3,Y_R>,
                                                             Element<Vec3,Y_PHI>>,
                                               ElementVector<>,
                                               ElementVector<Element<Vec3,J_R_JB>,
                                                             Element<Quat,PHI_JB>,
                                                             Element<Vec3,B_R_BV>,
                                                             Element<Quat,PHI_BV>>,
                                               MeasPoseExtFrameCentric>;

//residual implementing an update of the pose in an external frame from direct measurement
template<int J_R_JB, int PHI_JB, int B_R_BV, int PHI_BV, int Y_R=0, int Y_PHI=1>
class ExtFrameCentricPoseUpdate: public ExtFrameCentricPoseUpdateBase<J_R_JB, PHI_JB, B_R_BV, PHI_BV, Y_R, Y_PHI>{
 public:

  typedef ExtFrameCentricPoseUpdateBase<J_R_JB, PHI_JB, B_R_BV, PHI_BV, Y_R, Y_PHI> Base;
  typedef typename Base::Output Output;
  typedef typename Base::Previous Previous;
  typedef typename Base::Current Current;
  typedef ExtFrameCentricPositionUpdate<J_R_JB, PHI_JB, B_R_BV, Y_R> ExtFrameCentricPositionUpdateType;
  typedef ExtFrameCentricAttitudeUpdate<PHI_JB, PHI_BV, Y_PHI> ExtFrameCentricAttitudeUpdateType;

  using Base::meas_;

  ExtFrameCentricPoseUpdate(): Base(false,false,false) {};
  ~ExtFrameCentricPoseUpdate() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    positionUpdate_.meas_ = std::make_shared<const MeasPosExtFrameCentric>(MeasPosExtFrameCentric(meas_->GetPos()));
    attitudeUpdate_.meas_ = std::make_shared<const MeasAttExtFrameCentric>(MeasAttExtFrameCentric(meas_->GetAtt()));
    positionUpdate_.EvalRes(out,pre,cur);
    attitudeUpdate_.EvalRes(out,pre,cur);
    return 0;
  }

  int JacPre(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    //has dimension zero
    J.setZero();
    return 0;
  }

  int JacCur(MatRefX J, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    positionUpdate_.meas_ = std::make_shared<const MeasPosExtFrameCentric>(MeasPosExtFrameCentric(meas_->GetPos()));
    attitudeUpdate_.meas_ = std::make_shared<const MeasAttExtFrameCentric>(MeasAttExtFrameCentric(meas_->GetAtt()));
    positionUpdate_.JacCur(J.block(Output::Start(Y_R),0,3,cur.Dim()),pre,cur);
    attitudeUpdate_.JacCur(J.block(Output::Start(Y_PHI),0,3,cur.Dim()),pre,cur);
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    positionUpdate_.AddNoise(out,J_pre.block(Output::Start(Y_R),0,3,pre.Dim()),J_cur.block(Output::Start(Y_R),0,3,cur.Dim()), pre, cur);
    attitudeUpdate_.AddNoise(out,J_pre.block(Output::Start(Y_PHI),0,3,pre.Dim()),J_cur.block(Output::Start(Y_PHI),0,3,cur.Dim()), pre, cur);
  }

  int Extrapolate(const typename Current::CRef pre, typename Current::Ref ext, const double& delta_t){
    //nothing to do
    return 0;
  }

  int LoadParameters(const ros::NodeHandle& handle, const std::string& id){
    positionUpdate_.LoadParameters(handle, id+std::string{"/position_update"});
    attitudeUpdate_.LoadParameters(handle, id+std::string{"/attitude_update"});
    return 0;
  }

 protected:
  ExtFrameCentricPositionUpdateType positionUpdate_{ExtFrameCentricPositionUpdateType()};
  ExtFrameCentricAttitudeUpdateType attitudeUpdate_{ExtFrameCentricAttitudeUpdateType()};
};

} // namespace tsif
