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

#ifndef KIN2UPDATE_HPP_
#define KIN2UPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"

namespace LWFM {

template<typename STATE>
class Kin2Innovation: public LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>>::E_;
  static constexpr unsigned int _kin = 0;
  Kin2Innovation(){
    static_assert(_kin+1==E_,"Error with indices");
  };
  ~Kin2Innovation(){};
};
template<typename STATE>
class Kin2UpdateMeasAuxillary: public LWF::AuxiliaryBase<Kin2UpdateMeasAuxillary<STATE>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using LWF::AuxiliaryBase<Kin2UpdateMeasAuxillary<STATE>>::E_;
  Kin2UpdateMeasAuxillary(){
    contactFlags_.setZero();
  };
  ~Kin2UpdateMeasAuxillary(){};
  Eigen::Matrix<bool,STATE::nFeet_,1> contactFlags_;
};
template<typename STATE>
class Kin2UpdateMeas: public LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,Kin2UpdateMeasAuxillary<STATE>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,Kin2UpdateMeasAuxillary<STATE>>::E_;
  static constexpr unsigned int _enc = 0;
  static constexpr unsigned int _end = _enc+1;
  static constexpr unsigned int _aux = _end+1;
  Kin2UpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
  };
  ~Kin2UpdateMeas(){};
};
template<typename STATE>
class Kin2UpdateNoise: public LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>>::E_;
  static constexpr unsigned int _kin = 0;
  Kin2UpdateNoise(){
    static_assert(_kin+1==E_,"Error with indices");
    this->template getName<_kin>() = "kin";
  }
  ~Kin2UpdateNoise(){};
};
template<typename STATE>
class Kin2OutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<Kin2Innovation<STATE>::template getId<Kin2Innovation<STATE>::_kin>(),3,STATE::nFeet_>>{
};

template<typename FILTERSTATE, typename KINMODEL>
class Kin2Update: public LWF::Update<Kin2Innovation<typename FILTERSTATE::mtState>,FILTERSTATE,Kin2UpdateMeas<typename FILTERSTATE::mtState>,Kin2UpdateNoise<typename FILTERSTATE::mtState>,
    Kin2OutlierDetection<typename FILTERSTATE::mtState>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::Update<Kin2Innovation<typename FILTERSTATE::mtState>,FILTERSTATE,Kin2UpdateMeas<typename FILTERSTATE::mtState>,Kin2UpdateNoise<typename FILTERSTATE::mtState>,
      Kin2OutlierDetection<typename FILTERSTATE::mtState>> Base;
  using Base::eval;
  using Base::meas_;
  using Base::doubleRegister_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  Kin2Update(): model_(nullptr){
    qMB_.setIdentity();
    BrBM_.setZero();
    initCovFP_.setIdentity();
    doubleRegister_.registerScaledUnitMatrix("initCovFP",initCovFP_);
    doubleRegister_.registerVector("BrBM",BrBM_);
    doubleRegister_.registerQuaternion("qMB",qMB_);
  };
  ~Kin2Update(){};
  QPD qMB_;
  V3D BrBM_;
  M3D initCovFP_;
  KINMODEL* model_;
  void setModelPtr(KINMODEL* model) {model_ = model;}
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{
    /* Kinematic measurements
     * 0 = BrBM + qMB^T*MrMF - BrBF + Noise
     */
    V3D MrMF;
    for(int i=0;i<mtState::nFeet_;i++){
      MrMF = qMB_.rotate(V3D(-BrBM_+model_->forwardKinematicsBaseToFootInBaseFrame(meas_.template get<mtMeas::_enc>(i),i)));
      if(meas_.template get<mtMeas::_aux>().contactFlags_(i)){
        y.template get<mtInnovation::_kin>(i) = BrBM_ + qMB_.inverseRotate(state.template get<mtState::_fpt>(i))
            - model_->forwardKinematicsBaseToFootInBaseFrame(meas_.template get<mtMeas::_enc>(i),i) + noise.template get<mtNoise::_kin>(i);
      } else {
        y.template get<mtInnovation::_kin>(i) = noise.template get<mtNoise::_kin>(i);
      }
    }
  }
  void jacState(Eigen::MatrixXd& J, const mtState& state) const{
    J.setZero();
    for(int i=0;i<mtState::nFeet_;i++){
      if(meas_.template get<mtMeas::_aux>().contactFlags_(i)){
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtState::template getId<mtState::_fpt>(i)) = MPD(qMB_.inverted()).matrix();
      }
    }
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state) const{
    J.setZero();
    for(int i=0;i<mtState::nFeet_;i++){
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtNoise::template getId<mtNoise::_kin>(i)) = M3D::Identity();
    }
  }
  void refreshProperties(){
  }
  void preProcess(mtFilterState& filterstate, const mtMeas& meas, bool& isFinished){
    isFinished = false;
    V3D MrMF;
    for(int i=0;i<mtState::nFeet_;i++){
      if(meas.template get<mtMeas::_aux>().contactFlags_(i) && !(filterstate.state_.template get<mtState::_aux>().contactFlags_(i))){
        MrMF = qMB_.rotate(V3D(-BrBM_+model_->forwardKinematicsBaseToFootInBaseFrame(meas.template get<mtMeas::_enc>(i),i)));
        filterstate.resetFP(i,MrMF,initCovFP_);
      }
      filterstate.state_.template get<mtState::_aux>().contactFlags_(i) = meas.template get<mtMeas::_aux>().contactFlags_(i);
    }
  }
  void postProcess(mtFilterState& filterstate, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    for(int i=0;i<mtState::nFeet_;i++){
      filterstate.state_.template get<mtState::_aux>().slipFlags_(i) = outlierDetection.isOutlier(i);
    }
    isFinished = true;
  }
};



}


#endif /* KIN2UPDATE_HPP_ */
