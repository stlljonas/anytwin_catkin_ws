/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef IMUANDDYNPREDICTION_HPP_
#define IMUANDDYNPREDICTION_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/FilterState.hpp"
#include "lightweight_filtering/GIFPrediction.hpp"

namespace LWFM {

namespace ImuAndDyn {

enum ContactState{
  OPEN,
  CLOSED,
  SLIP,
  DISTURBED

};

template<unsigned int nFeet>
class Meas: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>,
LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,
LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,
LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,
LWF::ArrayElement<LWF::VectorElement<3>,nFeet>>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>,
      LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,
      LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,
      LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,
      LWF::ArrayElement<LWF::VectorElement<3>,nFeet>> Base;
  using Base::E_;
  static constexpr unsigned int _acc = 0;
  static constexpr unsigned int _gyr = _acc+1;
  static constexpr unsigned int _enc = _gyr+1;
  static constexpr unsigned int _end = _enc+1;
  static constexpr unsigned int _enf = _end+1;
  static constexpr unsigned int _ctf = _enf+1;
  Meas(){
    static_assert(_ctf+1==E_,"Error with indices");
    this->template getName<_acc>() = "acc";
    this->template getName<_gyr>() = "gyr";
    this->template getName<_enc>() = "enc";
    this->template getName<_end>() = "end";
    this->template getName<_enf>() = "enf";
    this->template getName<_ctf>() = "ctf";
    for(unsigned int i=0;i<nFeet;i++){
      contactState_[i] = OPEN;
    }
    hasValidDynamics_ = false;
    IrIB_.setZero();
    qBI_.setIdentity();
    BvB_.setZero();
    BwB_.setZero();
  }
  ~Meas(){};
  ContactState contactState_[nFeet];
  bool hasValidDynamics_;
  V3D IrIB_; // TODO: fix or remove
  QPD qBI_;
  V3D BvB_;
  V3D BwB_;
  void print() const{
    Base::print();
    for(unsigned int i=0;i<nFeet;i++){
      std::cout << static_cast<int>(contactState_[i]) << " ";
    }
    std::cout << std::endl;
  }
  void getJointTorques(VXD& jointTorques) const{
    jointTorques.resize(nFeet*3);
    for(unsigned int i=0;i<nFeet;i++){
      jointTorques.segment<3>(3*i) = this->template get<_enf>(i);
    }
  }
};

template<unsigned int nFeet>
class StateAuxiliary: public LWF::AuxiliaryBase<StateAuxiliary<nFeet>>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateAuxiliary(){
    for(unsigned int i=0;i<nFeet;i++){
      contactState_[i] = OPEN;
      contactCount_[i] = 0;
    }
    meas_.setIdentity();
    timeSinceLastValidPoseMeas_ = 0.0;
  };
  ~StateAuxiliary(){};
  ContactState contactState_[nFeet];
  int contactCount_[nFeet];
  Meas<nFeet> meas_;
  double timeSinceLastValidPoseMeas_;
  void print() const{
    for(unsigned int i=0;i<nFeet;i++){
      std::cout << static_cast<int>(contactState_[i]) << " ";
    }
    std::cout << std::endl;
  }
};

template<unsigned int nFeet, bool estFP>
class State: public LWF::State<
    LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
    LWF::TH_multiple_elements<LWF::QuaternionElement,1>,
    LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,(int)estFP>,
    StateAuxiliary<nFeet>>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,5>,
      LWF::TH_multiple_elements<LWF::QuaternionElement,1>,
      LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,(int)estFP>,
      StateAuxiliary<nFeet>> Base;
  using Base::E_;
  static constexpr unsigned int nFeet_ = nFeet;
  static constexpr bool estFP_ = estFP;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _ror = _vel+1;
  static constexpr unsigned int _acb = _ror+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _att = _gyb+1;
  static constexpr unsigned int _fpt = _att+(int)estFP;
  static constexpr unsigned int _aux = _fpt+1;
  State(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_ror>() = "ror";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_att>() = "att";
    if(estFP) this->template getName<_fpt>() = "fpt";
    this->template getName<_aux>() = "auxiliary";
  }
  ~State(){};
  StateAuxiliary<nFeet>& aux(){
    return this->template get<_aux>();
  }
  const StateAuxiliary<nFeet>& aux() const{
    return this->template get<_aux>();
  }
};

template<unsigned int nFeet, bool estFP, unsigned int DynD>
class Innovation: public LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,6>,
      LWF::VectorElement<DynD>,
      LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,1+(int)estFP>>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,6>,
      LWF::VectorElement<DynD>,
      LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,1+(int)estFP>> Base;
  using Base::E_;
  static constexpr unsigned int dynD_ = DynD;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  static constexpr unsigned int _acb = _att+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _acc = _gyb+1;
  static constexpr unsigned int _gyr = _acc+1;
  static constexpr unsigned int _dyn = _gyr+1;
  static constexpr unsigned int _kin = _dyn+1;
  static constexpr unsigned int _fpt = _kin+(int)estFP;
  Innovation(){
    static_assert(_fpt+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_att>() = "att";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    this->template getName<_acc>() = "acc";
    this->template getName<_gyr>() = "gyr";
    this->template getName<_dyn>() = "dyn";
    this->template getName<_kin>() = "kin";
    if(estFP) this->template getName<_fpt>() = "fpt";
  }
  ~Innovation(){};
};

template<unsigned int nFeet, bool estFP, unsigned int DynD>
class Noise: public Innovation<nFeet,estFP,DynD>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Noise(){}
  ~Noise(){};
};

template<unsigned int nFeet, bool estFP, unsigned int DynD>
class FilterState: public LWF::FilterState<State<nFeet,estFP>,Meas<nFeet>,Noise<nFeet,estFP,DynD>,0>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::FilterState<State<nFeet,estFP>,Meas<nFeet>,Noise<nFeet,estFP,DynD>,0> Base;
  typedef typename Base::mtState mtState;
  static constexpr int  D_ = mtState::D_;
  static constexpr unsigned int nFeet_ = mtState::nFeet_;
  static constexpr bool estFP_ = mtState::estFP_;
  using Base::state_;
  FilterState(){}
  void initWithMeas(const Meas<nFeet>& meas){
    state_.aux().meas_ = meas;
    initWithAccelerometer(meas.template get<Meas<nFeet>::_acc>());
  }
  void initWithAccelerometer(const V3D& fMeasInit){
    V3D unitZ(0,0,1);
    if(fMeasInit.norm()>1e-6){
      QPD q;
      q.setFromVectors(unitZ,state_.template get<mtState::_att>().rotate(fMeasInit));
      state_.template get<mtState::_att>() = q.inverted()*state_.template get<mtState::_att>();
    }
  }
};

template<unsigned int nFeet, bool estFP, typename KinModel>
class CommonModel{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  V3D g_;
  QPD qMB_;
  V3D BrBM_;
  mutable KinModel* mpKinModel_;
  typedef State<nFeet,estFP> mtState;
  typedef Meas<nFeet> mtMeas;
  CommonModel():g_(0,0,-9.81), mpKinModel_(nullptr){
    qMB_.setIdentity();
    BrBM_.setZero();
  };
  ~CommonModel(){};
  void setKinModel(KinModel* mpKinModel) {
    mpKinModel_ = mpKinModel;
    g_ = V3D(0.0,0.0,-mpKinModel_->getGravityAcceleration());

  }
  V3D get_BrBF(const mtState& state, int i) const{
    return BrBM_ + qMB_.inverseRotate(state.template get<mtState::_fpt>(i));
  }
  V3D get_BrBF(const mtMeas& meas, int i) const{
    return mpKinModel_->forwardKinematicsBaseToFootInBaseFrame(meas.template get<mtMeas::_enc>(i),i);
  }
  V3D get_MrMF(const mtState& state, int i) const{
    return state.template get<mtState::_fpt>(i);
  }
  V3D get_MrMF(const mtMeas& meas, int i) const{
    return qMB_.rotate(V3D(-BrBM_+mpKinModel_->forwardKinematicsBaseToFootInBaseFrame(meas.template get<mtMeas::_enc>(i),i)));
  }
  QPD get_qBI(const mtState& state) const{
    return (state.template get<mtState::_att>()*qMB_).inverted();
  }
  V3D get_IrIB(const mtState& state) const{
    return state.template get<mtState::_pos>()-get_qBI(state).inverseRotate(BrBM_);
  }
  V3D get_BwB(const mtState& state) const{
    return qMB_.inverseRotate(state.template get<mtState::_ror>());
  }
  V3D get_BvB(const mtState& state) const{
    return -qMB_.inverseRotate(state.template get<mtState::_vel>())-get_BwB(state).cross(BrBM_);
  }
  void getGeneralizedPositions(VXD& genPos, const mtState& state) const{
    genPos.resize(6+nFeet*3);
    genPos.segment<3>(0) = get_IrIB(state);
    genPos.segment<3>(3) = rot::EulerAnglesXyzPD(get_qBI(state)).vector();
//    genPos.segment<3>(0) = state.template get<mtState::_aux>().meas_.IrIB_;
//    genPos.segment<3>(3) = rot::EulerAnglesXyzPD(state.template get<mtState::_aux>().meas_.qBI_).vector();
    for(unsigned int i=0;i<nFeet;i++){
      genPos.segment<3>(6+i*3) = (state.aux().meas_).template get<mtMeas::_enc>(i);
    }
  }
  void getGeneralizedVelocities(VXD& genVel, const mtState& state) const{
    genVel.resize(6+nFeet*3);
    genVel.segment<3>(0) = get_qBI(state).inverseRotate(get_BvB(state));
    genVel.segment<3>(3) = rot::EulerAnglesXyzDiffPD(rot::EulerAnglesXyzPD(get_qBI(state)),rot::LocalAngularVelocityPD(get_BwB(state))).toImplementation();
//    QPD qBI = state.template get<mtState::_aux>().meas_.qBI_;
//    genVel.segment<3>(0) = qBI.inverseRotate(state.template get<mtState::_aux>().meas_.BvB_);
//    genVel.segment<3>(3) = rot::EulerAnglesXyzDiffPD(rot::EulerAnglesXyzPD(qBI),rot::LocalAngularVelocityPD(state.template get<mtState::_aux>().meas_.BwB_)).toImplementation();
    for(unsigned int i=0;i<nFeet;i++){
      genVel.segment<3>(6+i*3) = (state.aux().meas_).template get<mtMeas::_end>(i);
    }
  }
  void getMassInertiaMatrix(MXD& M, const mtState& state) const{
    VXD genPos;
    getGeneralizedPositions(genPos,state);
    getMassInertiaMatrix(M,genPos);
  }
  void getMassInertiaMatrix(MXD& M, const VXD& genPos) const{
    mpKinModel_->setGeneralizedPositions(genPos,true);
    mpKinModel_->getMassInertiaMatrix(M);
  }
  void getNonlinearEffects(VXD& h, const mtState& state) const{
    VXD genPos;
    getGeneralizedPositions(genPos,state);
    VXD genVel;
    getGeneralizedVelocities(genVel,state);
    getNonlinearEffects(h,genPos,genVel);
  }
  void getNonlinearEffects(VXD& h, const VXD& genPos, const VXD& genVel) const{
    mpKinModel_->setGeneralizedPositions(genPos,true);
    mpKinModel_->setGeneralizedVelocities(genVel,true);
    mpKinModel_->getNonlinearEffects(h);
  }
  void getFullContactJacobian(MXD& J, const mtState& state) const{
    VXD genPos;
    getGeneralizedPositions(genPos,state);
    getFullContactJacobian(J,genPos);
  }
  void getFullContactJacobian(MXD& J, const VXD& genPos) const{
    J.resize(nFeet*3,nFeet*3+6);
    mpKinModel_->setGeneralizedPositions(genPos,true);
    MXD J_sub;
    J_sub.resize(3,nFeet*3+6);
    for(unsigned int i=0;i<nFeet;i++){
      mpKinModel_->getJacobianFullTranslationBaseToFoot(J_sub,i);
      J.block(3*i,0,3,nFeet*3+6) = J_sub;
    }
  }
  void getSelectionMatrix(MXD& S) const{
    mpKinModel_->getSelectionMatrix(S);
  }
  void getContactForcesInWorld(VXD& contactForces, const mtState& state) const{
    contactForces.resize(nFeet*3);
    VXD genPos;
    getGeneralizedPositions(genPos,state);
    mpKinModel_->setGeneralizedPositions(genPos,true);
    for(unsigned int i=0;i<nFeet;i++){
//      std::cout << mpKinModel_->getOrientationWorldToFoot(i).transpose() << std::endl;
      contactForces.segment<3>(3*i) = mpKinModel_->getOrientationWorldToFoot(i).transpose()*state.aux().meas_.template get<mtMeas::_ctf>(i);
    }
  }
};

template<unsigned int DynD, unsigned int DynN, unsigned int nFeet, bool estFP, typename KinModel> // TODO temlate on dynamic model
class DynamicModelBase: public LWF::ModelBase<DynamicModelBase<DynD,DynN,nFeet,estFP,KinModel>,LWF::State<LWF::VectorElement<DynD>>,State<nFeet,estFP>,State<nFeet,estFP>,LWF::State<LWF::VectorElement<DynN>>>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::ModelBase<DynamicModelBase<DynD,DynN,nFeet,estFP,KinModel>,LWF::State<LWF::VectorElement<DynD>>,State<nFeet,estFP>,State<nFeet,estFP>,LWF::State<LWF::VectorElement<DynN>>> Base;
  typedef typename Base::mtInputTuple mtInputTuple;
  typedef typename Base::mtOutput mtInnovation;
  typedef typename std::tuple_element<0,mtInputTuple>::type mtState;
  typedef typename std::tuple_element<2,mtInputTuple>::type mtNoise;
  using Base::jacInputFD;
  CommonModel<nFeet,estFP,KinModel>* mpCommonModel_;
  DynamicModelBase(CommonModel<nFeet,estFP,KinModel>* mpCommonModel): mpCommonModel_(mpCommonModel){};
  virtual ~DynamicModelBase(){};
  void eval_(mtInnovation& y, const mtInputTuple& inputs, double dt) const{
    evalResidual(y,std::get<0>(inputs),std::get<1>(inputs),std::get<2>(inputs),dt);
  }
  virtual void evalResidual(mtInnovation& y, const mtState& previousState, const mtState& currentState, const mtNoise& noise, double dt) const = 0;
  template<int i>
  void jacInput_(MXD& F, const mtInputTuple& inputs, double dt) const{} // No implementation of Jacobians
  virtual void jacPreviousState(MXD& F, const mtState& previousState, const mtState& currentState, double dt) const{
    mtNoise n;
    n.setIdentity();
    F.setZero();
    this->template jacInputFD<0,0,9>(F,std::forward_as_tuple(previousState,currentState,n),dt,1e-6);
    this->template jacInputFD<0,mtState::template getId<mtState::_att>(),3>(F,std::forward_as_tuple(previousState,currentState,n),dt,1e-6);
  }
  virtual void jacCurrentState(MXD& F, const mtState& previousState, const mtState& currentState, double dt) const{
    mtNoise n;
    n.setIdentity();
    F.setZero();
    this->template jacInputFD<1,0,9>(F,std::forward_as_tuple(previousState,currentState,n),dt,1e-6);
    this->template jacInputFD<1,mtState::template getId<mtState::_att>(),3>(F,std::forward_as_tuple(previousState,currentState,n),dt,1e-6);
  }
  virtual void jacNoise(MXD& F, const mtState& previousState, const mtState& currentState, double dt) const{
    mtNoise n;
    n.setIdentity();
    F.setZero();
    this->template jacInputFD<2>(F,std::forward_as_tuple(previousState,previousState,n),dt,1e-6);
  }
};

template<unsigned int nFeet, bool estFP, typename KinModel>
class DynamicModelMinimal: public DynamicModelBase<nFeet*3+6,nFeet*3+6,nFeet,estFP,KinModel>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef DynamicModelBase<nFeet*3+6,nFeet*3+6,nFeet,estFP,KinModel> Base;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtNoise mtNoise;
  using Base::mpCommonModel_;
  mutable VXD genPosPrev_;
  mutable VXD genPosCurr_;
  mutable VXD genVelPrev_;
  mutable VXD genVelCurr_;
  mutable VXD genAcc_;
  mutable MXD M_;
  mutable VXD h_;
  mutable MXD J_;
  mutable MXD S_;
  mutable VXD jointTorques_;
  mutable VXD contactForces_;
  mutable MXD N_;
  DynamicModelMinimal(CommonModel<nFeet,estFP,KinModel>* mpCommonModel): Base(mpCommonModel){};
  ~DynamicModelMinimal(){};
  void evalResidual(mtInnovation& y, const mtState& previousState, const mtState& currentState, const mtNoise& noise, double dt) const{
    mpCommonModel_->getGeneralizedPositions(genPosPrev_,previousState);
    mpCommonModel_->getGeneralizedPositions(genPosCurr_,currentState);
    mpCommonModel_->getGeneralizedVelocities(genVelPrev_,previousState);
    mpCommonModel_->getGeneralizedVelocities(genVelCurr_,currentState);
    genAcc_ = (genVelCurr_ - genVelPrev_)/dt;
    mpCommonModel_->getMassInertiaMatrix(M_,genPosPrev_);
    mpCommonModel_->getNonlinearEffects(h_,genPosPrev_,genVelPrev_);
    mpCommonModel_->getFullContactJacobian(J_,genPosPrev_);
    mpCommonModel_->getSelectionMatrix(S_);
    previousState.aux().meas_.getJointTorques(jointTorques_);
    mpCommonModel_->getContactForcesInWorld(contactForces_,previousState);
    N_ = Eigen::Matrix<double,nFeet*3+6,nFeet*3+6>::Identity() - J_.transpose()*(J_*J_.transpose()).inverse()*J_;
//    std::cout << "=======================================" << std::endl;
    y.template get<0>() = N_*(M_*genAcc_+h_-S_.transpose()*jointTorques_) + noise.template get<0>()/sqrt(dt); // TODO: check noise, evaluate magnitude, evaluate different residuals
//    std::cout << "Total: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;
//    y.template get<0>() = N_*(M_*genAcc_);
//    std::cout << "Accelerations: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;
//    y.template get<0>() = N_*(h_);
//    std::cout << "Nonlinear Terms: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;
//    y.template get<0>() = N_*(-S_.transpose()*jointTorques_);
//    std::cout << "Joint Torques: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;

//    std::cout << "=======================================" << std::endl;
//    std::cout << J_.transpose().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(M_*genAcc_+h_-S_.transpose()*jointTorques_).transpose() << std::endl;
//    std::cout << contactForces_.transpose() << std::endl;

//    std::cout << "=======================================" << std::endl;
//    y.template get<0>() = M_*genAcc_+h_-J_.transpose()*contactForces_-S_.transpose()*jointTorques_;
//    std::cout << "Total: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;
//    y.template get<0>() = M_*genAcc_;
//    std::cout << "Accelerations: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;
//    y.template get<0>() = h_;
//    std::cout << "Nonlinear Terms: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;
//    y.template get<0>() = -J_.transpose()*contactForces_;
//    std::cout << "Contact Forces: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;
//    y.template get<0>() = -S_.transpose()*jointTorques_;
//    std::cout << "Joint Torques: " << std::endl;
//    std::cout << y.template get<0>().template segment<9>(0).transpose() << std::endl;

//    std::cout << "=======================================" << std::endl;
//    std::cout << J_ << std::endl;
//    std::cout << mpCommonModel_->mpKinModel_->getJacobianTranslationBaseToFoot(previousState.template get<mtState::_aux>().meas_.template get<Meas<nFeet>::_enc>(0),0) << std::endl;
  }
  void jacNoise(MXD& F, const mtState& previousState, const mtState& currentState, double dt) const{
    F.setIdentity();
    F = F/sqrt(dt);
  }
};

template<unsigned int nFeet, bool estFP, typename KinModel, unsigned int DynD>
class Prediction: public LWF::GIFPrediction<FilterState<nFeet,estFP,DynD>,Innovation<nFeet,estFP,DynD>,Meas<nFeet>,Noise<nFeet,estFP,DynD>>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::GIFPrediction<FilterState<nFeet,estFP,DynD>,Innovation<nFeet,estFP,DynD>,Meas<nFeet>,Noise<nFeet,estFP,DynD>> Base;
  using Base::evalResidual;
  using Base::testPredictionJacs;
  using Base::meas_;
  using Base::r_;
  using Base::doubleRegister_;
  using Base::boolRegister_;
  using Base::intRegister_;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtInnovation mtInnovation;
  DynamicModelMinimal<nFeet,estFP,KinModel> dynamicModelMinimal_;
  bool useDynResidual_;
  bool useIMUResidual_;
  bool useConResidual_;
  int minContactCount_;
  typedef CommonModel<nFeet,estFP,KinModel> mtCommonModel;
  mtCommonModel commonModel_;
  Prediction(): dynamicModelMinimal_(&commonModel_){
    useDynResidual_ = true;
    useIMUResidual_ = true;
    useConResidual_ = true;
    minContactCount_ = 1;
    doubleRegister_.registerVector("BrBM",commonModel_.BrBM_);
    doubleRegister_.registerQuaternion("qMB",commonModel_.qMB_);
    boolRegister_.registerScalar("useDynResidual",useDynResidual_);
    boolRegister_.registerScalar("useIMUResidual",useIMUResidual_);
    boolRegister_.registerScalar("useConResidual",useConResidual_);
    intRegister_.registerScalar("minContactCount",minContactCount_);
  };
  ~Prediction(){};
  void test(){
    unsigned int s = 0;
    mtMeas meas;
    meas.setRandom(s);
    meas.contactState_[0] = LWFM::ImuAndDyn::CLOSED;
    meas.contactState_[3] = LWFM::ImuAndDyn::CLOSED;
    mtState previousState;
    previousState.setRandom(s);
    previousState.template get<mtState::_aux>().contactState_[0] = LWFM::ImuAndDyn::CLOSED;
    previousState.template get<mtState::_aux>().contactState_[3] = LWFM::ImuAndDyn::CLOSED;
    previousState.template get<mtState::_aux>().contactCount_[0] = minContactCount_;
    previousState.template get<mtState::_aux>().contactCount_[3] = minContactCount_;
    previousState.template get<mtState::_aux>().meas_ = meas;
    meas.setRandom(s);
    mtState currentState;
    currentState.setRandom(s);
    testPredictionJacs(previousState,currentState,meas,1e-6,1e-5,0.1);
  }
  void setKinModel(KinModel* mpKinModel) {
    commonModel_.setKinModel(mpKinModel);
  }
  void evalResidual(mtInnovation& y, const mtState& previousState, const mtState& currentState, const mtNoise& noise, double dt) const{
    // Derivative of position and attitude
    y.template get<mtInnovation::_pos>() = (currentState.template get<mtState::_pos>() - previousState.template get<mtState::_pos>())/dt
        + currentState.template get<mtState::_att>().rotate(currentState.template get<mtState::_vel>()) + noise.template get<mtNoise::_pos>()/sqrt(dt);
    y.template get<mtInnovation::_att>() = (previousState.template get<mtState::_att>().inverted()*currentState.template get<mtState::_att>()).logarithmicMap()/dt
        + currentState.template get<mtState::_ror>() + noise.template get<mtNoise::_att>()/sqrt(dt);
    // Gyroscope and Acceleromter biases
    y.template get<mtInnovation::_acb>() = (currentState.template get<mtState::_acb>() - previousState.template get<mtState::_acb>())/dt + noise.template get<mtNoise::_acb>()/sqrt(dt);
    y.template get<mtInnovation::_gyb>() = (currentState.template get<mtState::_gyb>() - previousState.template get<mtState::_gyb>())/dt + noise.template get<mtNoise::_gyb>()/sqrt(dt);
    // IMU measurements
    if(useIMUResidual_){
      y.template get<mtInnovation::_acc>() = (currentState.template get<mtState::_vel>() - (M3D::Identity()-gSM(currentState.template get<mtState::_ror>()*dt))*previousState.template get<mtState::_vel>())/dt
          + meas_.template get<mtMeas::_acc>() - currentState.template get<mtState::_acb>() + currentState.template get<mtState::_att>().inverseRotate(commonModel_.g_) + noise.template get<mtNoise::_acc>()/sqrt(dt);
      y.template get<mtInnovation::_gyr>() = currentState.template get<mtState::_ror>() + currentState.template get<mtState::_gyb>() - meas_.template get<mtMeas::_gyr>() + noise.template get<mtNoise::_gyr>()/sqrt(dt);
    } else {
      y.template get<mtInnovation::_acc>() = noise.template get<mtNoise::_acc>()/sqrt(dt);
      y.template get<mtInnovation::_gyr>() = noise.template get<mtNoise::_gyr>()/sqrt(dt);
    }
    // Foothold derivatives and kinematics
    evalResidualFP<estFP>(y,previousState,currentState,noise,dt);
    // Dynamics
    if(useDynResidual_){
      typename DynamicModelMinimal<nFeet,estFP,KinModel>::mtInnovation dynResidual;
      typename DynamicModelMinimal<nFeet,estFP,KinModel>::mtNoise dynNoise;
      dynNoise.template get<0>() = noise.template get<mtNoise::_dyn>();
      dynamicModelMinimal_.evalResidual(dynResidual,previousState,currentState,dynNoise,dt);
      y.template get<mtInnovation::_dyn>() = dynResidual.template get<0>();
    } else {
      y.template get<mtInnovation::_dyn>() = noise.template get<mtNoise::_dyn>()/sqrt(dt);
    }
  }
  template<bool _estFP,typename std::enable_if<_estFP>::type* = nullptr>
  inline void evalResidualFP(mtInnovation& y, const mtState& previousState, const mtState& currentState, const mtNoise& noise, double dt) const{
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      if(meas_.contactState_[i] == CLOSED && previousState.aux().contactCount_[i] >= minContactCount_ && useConResidual_){
//        y.template get<mtInnovation::_fpt>(i) = (currentState.template get<mtState::_fpt>(i) - (M3D::Identity()-gSM(currentState.template get<mtState::_ror>()*dt))*previousState.template get<mtState::_fpt>(i))/dt
//            - currentState.template get<mtState::_vel>() + noise.template get<mtNoise::_fpt>(i)/sqrt(dt);
        y.template get<mtInnovation::_fpt>(i) = currentState.template get<mtState::_pos>()+currentState.template get<mtState::_att>().rotate(currentState.template get<mtState::_fpt>(i))
            - previousState.template get<mtState::_pos>()-previousState.template get<mtState::_att>().rotate(previousState.template get<mtState::_fpt>(i))
            + noise.template get<mtNoise::_fpt>(i)*sqrt(dt);
      } else {
//        y.template get<mtInnovation::_fpt>(i) = noise.template get<mtNoise::_fpt>(i)/sqrt(dt);
        y.template get<mtInnovation::_fpt>(i) = noise.template get<mtNoise::_fpt>(i)*sqrt(dt);
      }
      y.template get<mtInnovation::_kin>(i) = commonModel_.get_BrBF(currentState,i) - commonModel_.get_BrBF(meas_,i) + noise.template get<mtNoise::_kin>(i);
    }
  }
  template<bool _estFP,typename std::enable_if<!_estFP>::type* = nullptr>
  inline void evalResidualFP(mtInnovation& y, const mtState& previousState, const mtState& currentState, const mtNoise& noise, double dt) const{
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      const M3D dMrMF = MPD(commonModel_.qMB_).matrix()*commonModel_.mpKinModel_->getJacobianTranslationBaseToFoot(meas_.template get<mtMeas::_enc>(i),i);
      if(meas_.contactState_[i] == CLOSED && useConResidual_){
        y.template get<mtInnovation::_kin>(i) = -currentState.template get<mtState::_vel>()
            + gSM(currentState.template get<mtState::_ror>())*commonModel_.get_MrMF(meas_,i) + dMrMF*meas_.template get<mtMeas::_end>(i)+noise.template get<mtNoise::_kin>(i);
      } else {
        y.template get<mtInnovation::_kin>(i) = noise.template get<mtNoise::_kin>(i);
      }
    }
  }
  void noMeasCase(mtFilterState& filterState, mtMeas& meas, double dt){
    meas.template get<mtMeas::_gyr>() = filterState.state_.template get<mtState::_gyb>();
    meas.template get<mtMeas::_acc>() = filterState.state_.template get<mtState::_acb>()-filterState.state_.template get<mtState::_att>().inverseRotate(commonModel_.g_);
    meas.hasValidDynamics_ = false;
  }
  void jacPreviousState(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{;
    J.setZero();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pos>()) = -M3D::Identity()/dt;
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_att>()) =
        -Lmat((previousState.template get<mtState::_att>().inverted()*currentState.template get<mtState::_att>()).logarithmicMap()).inverse()/dt*MPD(previousState.template get<mtState::_att>().inverted()).matrix();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acb>(),mtState::template getId<mtState::_acb>()) = -M3D::Identity()/dt;
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_gyb>(),mtState::template getId<mtState::_gyb>()) = -M3D::Identity()/dt;
    if(useIMUResidual_){
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acc>(),mtState::template getId<mtState::_vel>()) = -(M3D::Identity()-gSM(currentState.template get<mtState::_ror>()*dt))/dt;
    }
    footpointsJacPreviousState<estFP>(J,previousState,currentState,dt);
    // Dynamics
    if(useDynResidual_){
      MXD J_sub(DynD,(int)(mtState::D_));
      dynamicModelMinimal_.jacPreviousState(J_sub,previousState,currentState,dt);
      J.template block<DynD,(int)(mtState::D_)>(mtInnovation::template getId<mtInnovation::_dyn>(),0) = J_sub;
    }
  }
  template<bool _estFP,typename std::enable_if<_estFP>::type* = nullptr>
  inline void footpointsJacPreviousState(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      if(meas_.contactState_[i] == CLOSED && previousState.aux().contactCount_[i] >= minContactCount_ && useConResidual_){
//        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_fpt>(i)) =
//            -(M3D::Identity()-gSM(currentState.template get<mtState::_ror>()*dt))/dt;
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_att>()) =
            -gSM(previousState.template get<mtState::_att>().rotate(previousState.template get<mtState::_fpt>(i)));
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_fpt>(i)) =
            -MPD(previousState.template get<mtState::_att>()).matrix();
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_pos>()) =
            -M3D::Identity();
      }
    }
  }
  template<bool _estFP,typename std::enable_if<!_estFP>::type* = nullptr>
  inline void footpointsJacPreviousState(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{
  }
  void jacCurrentState(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{;
    J.setZero();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pos>()) = M3D::Identity()/dt;
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_vel>()) = MPD(currentState.template get<mtState::_att>()).matrix();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_att>()) = gSM(currentState.template get<mtState::_att>().rotate(currentState.template get<mtState::_vel>()));
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_att>()) =
        Lmat((previousState.template get<mtState::_att>().inverted()*currentState.template get<mtState::_att>()).logarithmicMap()).inverse()/dt*MPD(previousState.template get<mtState::_att>().inverted()).matrix();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_ror>()) = M3D::Identity();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acb>(),mtState::template getId<mtState::_acb>()) = M3D::Identity()/dt;
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_gyb>(),mtState::template getId<mtState::_gyb>()) = M3D::Identity()/dt;
    if(useIMUResidual_){
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acc>(),mtState::template getId<mtState::_vel>()) = M3D::Identity()/dt;
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acc>(),mtState::template getId<mtState::_ror>()) = -gSM(previousState.template get<mtState::_vel>());
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acc>(),mtState::template getId<mtState::_acb>()) = -M3D::Identity();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acc>(),mtState::template getId<mtState::_att>()) = - MPD(currentState.template get<mtState::_att>().inverted()).matrix() * gSM(commonModel_.g_);
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_gyr>(),mtState::template getId<mtState::_ror>()) = M3D::Identity();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_gyr>(),mtState::template getId<mtState::_gyb>()) = M3D::Identity();
    }
    footpointsJacCurrentState<estFP>(J,previousState,currentState,dt);
    // Dynamics
    if(useDynResidual_){
      MXD J_sub(DynD,(int)(mtState::D_));
      dynamicModelMinimal_.jacCurrentState(J_sub,previousState,currentState,dt);
      J.template block<DynD,(int)(mtState::D_)>(mtInnovation::template getId<mtInnovation::_dyn>(),0) = J_sub;
    }
  }
  template<bool _estFP,typename std::enable_if<_estFP>::type* = nullptr>
  inline void footpointsJacCurrentState(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      if(meas_.contactState_[i] == CLOSED && previousState.aux().contactCount_[i] >= minContactCount_ && useConResidual_){
//        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_fpt>(i)) = M3D::Identity()/dt;
//        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_ror>()) = -gSM(previousState.template get<mtState::_fpt>(i));
//        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_vel>()) = -M3D::Identity();
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_att>()) =
            gSM(currentState.template get<mtState::_att>().rotate(currentState.template get<mtState::_fpt>(i)));
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_fpt>(i)) =
            MPD(currentState.template get<mtState::_att>()).matrix();
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtState::template getId<mtState::_pos>()) =
            M3D::Identity();
      }
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtState::template getId<mtState::_fpt>(i)) = MPD(commonModel_.qMB_.inverted()).matrix();
    }
  }
  template<bool _estFP,typename std::enable_if<!_estFP>::type* = nullptr>
  inline void footpointsJacCurrentState(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      if(meas_.contactState_[i] == CLOSED && useConResidual_){
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtState::template getId<mtState::_vel>()) = -M3D::Identity();
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtState::template getId<mtState::_ror>()) = -gSM(commonModel_.get_MrMF(meas_,i));
      }
    }
  }
  void jacNoise(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{
    J.setZero();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity()/sqrt(dt);
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtNoise::template getId<mtNoise::_att>()) = M3D::Identity()/sqrt(dt);
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = M3D::Identity()/sqrt(dt);
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = M3D::Identity()/sqrt(dt);
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_acc>(),mtNoise::template getId<mtNoise::_acc>()) = M3D::Identity()/sqrt(dt);
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_gyr>(),mtNoise::template getId<mtNoise::_gyr>()) = M3D::Identity()/sqrt(dt);
    footpointsJacNoise<estFP>(J,previousState,currentState,dt);
    // Dynamics
    if(useDynResidual_){
      MXD J_sub(DynD,DynD);
      dynamicModelMinimal_.jacNoise(J_sub,previousState,currentState,dt);
      J.template block<DynD,DynD>(mtInnovation::template getId<mtInnovation::_dyn>(),mtNoise::template getId<mtNoise::_dyn>()) = J_sub;
    } else {
      J.template block<mtInnovation::dynD_,mtInnovation::dynD_>(mtInnovation::template getId<mtInnovation::_dyn>(),mtNoise::template getId<mtNoise::_dyn>()) = Eigen::Matrix<double,mtInnovation::dynD_,mtInnovation::dynD_>::Identity()/sqrt(dt);
    }
  }
  template<bool _estFP,typename std::enable_if<_estFP>::type* = nullptr>
  inline void footpointsJacNoise(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_fpt>(i),mtNoise::template getId<mtNoise::_fpt>(i)) = M3D::Identity()*sqrt(dt);
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtNoise::template getId<mtNoise::_kin>(i)) = M3D::Identity();
    }
  }
  template<bool _estFP,typename std::enable_if<!_estFP>::type* = nullptr>
  inline void footpointsJacNoise(MXD& J, const mtState& previousState, const mtState& currentState, double dt) const{
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtNoise::template getId<mtNoise::_kin>(i)) = M3D::Identity();
    }
  }
  void preProcess(mtFilterState& filterState, const mtMeas& meas, double dt){
    preProcessFootpoints<estFP>(filterState,meas,dt);
  };
  template<bool _estFP,typename std::enable_if<_estFP>::type* = nullptr>
  void preProcessFootpoints(mtFilterState& filterState, const mtMeas& meas, double dt){
  }
  template<bool _estFP,typename std::enable_if<!_estFP>::type* = nullptr>
  void preProcessFootpoints(mtFilterState& filterState, const mtMeas& meas, double dt){
  }
  void postProcess(mtFilterState& filterState, const mtMeas& meas, double dt){
    for(int i=0;i<mtState::nFeet_;i++){
      filterState.state_.aux().contactState_[i] = meas.contactState_[i];
      if(filterState.state_.aux().contactState_[i] == CLOSED){
        filterState.state_.aux().contactCount_[i]++;
      } else {
        filterState.state_.aux().contactCount_[i] = 0;
      }
    }
    filterState.state_.aux().meas_ = meas;


//    std::cout << "=======================================" << std::endl;
//    std::cout << dt << std::endl;
//    r_.print();
//
////    std::cout << "=======================================" << std::endl;
////    std::cout << filterState.state_.template get<mtState::_att>() << std::endl;
//
//    std::cout << "=======================================" << std::endl;
//    std::cout << filterState.state_.template get<mtState::_att>().inverted() << std::endl;
//    std::cout << filterState.state_.template get<mtState::_aux>().meas_.qBI_ << std::endl;
//    std::cout << commonModel_.get_BvB(filterState.state_).transpose() << std::endl;
//    std::cout << filterState.state_.template get<mtState::_aux>().meas_.BvB_.transpose() << std::endl;
//    std::cout << commonModel_.get_BwB(filterState.state_).transpose() << std::endl;
//    std::cout << filterState.state_.template get<mtState::_aux>().meas_.BwB_.transpose() << std::endl;
//    std::cout << filterState.state_.template get<mtState::_gyb>().transpose() << std::endl;
//    std::cout << filterState.state_.template get<mtState::_acb>().transpose() << std::endl;
  };
  void getLinearizationPoint(mtState& state1, const mtFilterState& filterState, const mtMeas& meas, double dt){
    state1 = filterState.state_;
    const mtState& state0 = filterState.state_;
    state1.template get<mtState::_ror>() = meas.template get<mtMeas::_gyr>()-state0.template get<mtState::_gyb>();
    QPD dQ = dQ.exponentialMap(-dt*state1.template get<mtState::_ror>());
    state1.template get<mtState::_att>() = state0.template get<mtState::_att>()*dQ;
    state1.template get<mtState::_vel>() = (M3D::Identity()-gSM(dt*state1.template get<mtState::_ror>()))*state0.template get<mtState::_vel>()-dt*(meas_.template get<mtMeas::_acc>()-state0.template get<mtState::_acb>()+state1.template get<mtState::_att>().inverseRotate(commonModel_.g_));
    state1.template get<mtState::_pos>() = state0.template get<mtState::_pos>()-dt*state1.template get<mtState::_att>().rotate(state1.template get<mtState::_vel>());
    state1.template get<mtState::_acb>() = state0.template get<mtState::_acb>();
    state1.template get<mtState::_gyb>() = state0.template get<mtState::_gyb>();
    getLinearizationPointFpt<estFP>(state1,filterState,meas,dt);
    state1.aux().meas_ = meas;
    state1.fix();
  };
  template<bool _estFP,typename std::enable_if<_estFP>::type* = nullptr>
  void getLinearizationPointFpt(mtState& state1, const mtFilterState& filterState, const mtMeas& meas, double dt){
    for(int i=0;i<mtState::nFeet_;i++){
      state1.template get<mtState::_fpt>(i) = commonModel_.get_MrMF(meas,i);
    }
  }
  template<bool _estFP,typename std::enable_if<!_estFP>::type* = nullptr>
  void getLinearizationPointFpt(mtState& state1, const mtFilterState& filterState, const mtMeas& meas, double dt){
  }
};

}

}


#endif /* IMUANDDYNPREDICTION_HPP_ */
