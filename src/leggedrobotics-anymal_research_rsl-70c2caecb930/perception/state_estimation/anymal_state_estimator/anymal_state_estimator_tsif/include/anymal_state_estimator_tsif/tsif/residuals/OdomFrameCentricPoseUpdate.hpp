
/*!
 * @file    OdomFrameCentricPoseUpdate.hpp
 * @author  Fabian Tresoldi
 * @date    April, 2018
 */

#pragma once

#include <anymal_state_estimator_tsif/tsif/residuals/OdomFrameCentricPositionUpdate.hpp>
#include <anymal_state_estimator_tsif/tsif/residuals/OdomFrameCentricAttitudeUpdate.hpp>
#include <message_logger/message_logger.hpp>
#include <any_node/Param.hpp>

namespace tsif{

//measurement class for a pose measurement in external frame
class MeasPoseOdomFrameCentric: public ElementVector<Element<Vec3,0>,Element<Quat,1>>{
 public:
  MeasPoseOdomFrameCentric(): ElementVector<Element<Vec3,0>,Element<Quat,1>>(Vec3(0,0,0),Quat(1,0,0,0)){}
  MeasPoseOdomFrameCentric(const Vec3& J_r_JV, const Quat& q_JV): ElementVector<Element<Vec3,0>,Element<Quat,1>>(J_r_JV, q_JV){}
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

template<int I_R_IB,//external to base position in map frame
         int PHI_IB,//base to external frame orientation
         int I_R_IJ,//external to base position in map frame
         int PHI_IJ,//base to external frame orientation
         int B_R_BV,//position base to measurement frame in base
         int PHI_BV,//measurement frame to base orientation
         int Y_R=0,//linear innovation
         int Y_PHI=1>//angular innovation
using OdomFrameCentricPoseUpdateBase = Residual<ElementVector<Element<Vec3,Y_R>,
                                                              Element<Vec3,Y_PHI>>,
                                                ElementVector<>,
                                                ElementVector<Element<Vec3,I_R_IB>,
                                                              Element<Quat,PHI_IB>,
                                                              Element<Vec3,I_R_IJ>,
                                                              Element<Quat,PHI_IJ>,
                                                              Element<Vec3,B_R_BV>,
                                                              Element<Quat,PHI_BV>>,
                                                MeasPoseOdomFrameCentric>;

//residual implementing an update of the pose in odom and the ext to odom pose from a pose measurement in the external frame
template<int I_R_IB, int PHI_IB, int I_R_IJ, int PHI_IJ, int B_R_BV, int PHI_BV, int Y_R=0, int Y_PHI=1>
class OdomFrameCentricPoseUpdate: public OdomFrameCentricPoseUpdateBase<I_R_IB, PHI_IB, I_R_IJ, PHI_IJ, B_R_BV, PHI_BV, Y_R, Y_PHI>{
 public:

  typedef OdomFrameCentricPoseUpdateBase<I_R_IB, PHI_IB, I_R_IJ, PHI_IJ, B_R_BV, PHI_BV, Y_R, Y_PHI> Base;
  typedef typename Base::Output Output;
  typedef typename Base::Previous Previous;
  typedef typename Base::Current Current;
  typedef OdomFrameCentricPositionUpdate<I_R_IB, PHI_IB, I_R_IJ, PHI_IJ, B_R_BV, Y_R> OdomFrameCentricPositionUpdateType;
  typedef OdomFrameCentricAttitudeUpdate<PHI_IB, PHI_IJ, PHI_BV, Y_PHI> OdomFrameCentricAttitudeUpdateType;

  using Base::meas_;

  OdomFrameCentricPoseUpdate(): Base(false,false,false) {};
  ~OdomFrameCentricPoseUpdate() override = default;

  int EvalRes(typename Output::Ref out, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    positionUpdate_.meas_ = std::make_shared<const MeasPos>(MeasPos(meas_->GetPos()));
    attitudeUpdate_.meas_ = std::make_shared<const MeasAtt>(MeasAtt(meas_->GetAtt()));
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
    J.setZero();
    positionUpdate_.meas_ = std::make_shared<const MeasPos>(MeasPos(meas_->GetPos()));
    attitudeUpdate_.meas_ = std::make_shared<const MeasAtt>(MeasAtt(meas_->GetAtt()));
    positionUpdate_.JacCur(J.block(Output::Start(Y_R),0,3,cur.Dim()),pre,cur);
    attitudeUpdate_.JacCur(J.block(Output::Start(Y_PHI),0,3,cur.Dim()),pre,cur);
    return 0;
  }

  virtual void AddNoise(typename Output::Ref out, MatRefX J_pre, MatRefX J_cur, const typename Previous::CRef pre, const typename Current::CRef cur) override {
    positionUpdate_.AddNoise(out,J_pre.block(Output::Start(Y_R),0,3,pre.Dim()),J_cur.block(Output::Start(Y_R),0,3,cur.Dim()), pre, cur);
    attitudeUpdate_.AddNoise(out,J_pre.block(Output::Start(Y_PHI),0,3,pre.Dim()),J_cur.block(Output::Start(Y_PHI),0,3,cur.Dim()), pre, cur);
  }

  int Extrapolate(const typename Current::CRef pre, typename Current::Ref ext, const double& delta_t){
    //TODO
    return 0;
  }

  int LoadParameters(const ros::NodeHandle& handle, const std::string& id){
    positionUpdate_.LoadParameters(handle, id+std::string{"/position_update"});
    attitudeUpdate_.LoadParameters(handle, id+std::string{"/attitude_update"});
    return 0;
  }

 protected:
  OdomFrameCentricPositionUpdateType positionUpdate_{OdomFrameCentricPositionUpdateType()};
  OdomFrameCentricAttitudeUpdateType attitudeUpdate_{OdomFrameCentricAttitudeUpdateType()};
};

} // namespace tsif
