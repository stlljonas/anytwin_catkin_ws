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

#ifndef KINUPDATE_HPP_
#define KINUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/FilterState.hpp"
#include "lightweight_filtering_models/Predictions/ImuPrediction.hpp"

namespace LWFM {

template<typename STATE>
class KinInnovation: public LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>>{
 public:
  using LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>>::E_;
  static constexpr unsigned int _kin = 0;
  KinInnovation(){
    static_assert(_kin+1==E_,"Error with indices");
  };
  ~KinInnovation(){};
};
template<typename STATE>
class KinUpdateMeasAuxillary: public LWF::AuxiliaryBase<KinUpdateMeasAuxillary<STATE>>{
 public:
  using LWF::AuxiliaryBase<KinUpdateMeasAuxillary<STATE>>::E_;
  KinUpdateMeasAuxillary(){
    contactFlags_.setZero();
  };
  ~KinUpdateMeasAuxillary(){};
  Eigen::Matrix<bool,STATE::nFeet_,1> contactFlags_;
};
template<typename STATE>
class KinUpdateMeas: public LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,KinUpdateMeasAuxillary<STATE>>{
 public:
  using LWF::State<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,KinUpdateMeasAuxillary<STATE>>::E_;
  static constexpr unsigned int _enc = 0;
  static constexpr unsigned int _end = _enc+1;
  static constexpr unsigned int _aux = _end+1;
  KinUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
  };
  ~KinUpdateMeas(){};
};
template<typename STATE>
class KinUpdateNoise: public LWF::State<LWF::VectorElement<3>,LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>>{
 public:
  using LWF::State<LWF::VectorElement<3>,LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>>::E_;
  static constexpr unsigned int _gyr = 0;
  static constexpr unsigned int _kin = _gyr+1;
  KinUpdateNoise(){
    static_assert(_kin+1==E_,"Error with indices");
    this->template getName<_gyr>() = "gyr";
    this->template getName<_kin>() = "kin";
  }
  ~KinUpdateNoise(){};
};
template<typename STATE>
class KinOutlierDetection: public LWF::OutlierDetection<LWF::ODEntry<KinInnovation<STATE>::template getId<KinInnovation<STATE>::_kin>(),3,STATE::nFeet_>>{
};

template<typename FILTERSTATE, typename KINMODEL>
class KinUpdate: public LWF::Update<KinInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,KinUpdateMeas<typename FILTERSTATE::mtState>,KinUpdateNoise<typename FILTERSTATE::mtState>,
    KinOutlierDetection<typename FILTERSTATE::mtState>,true>{
 public:
  typedef LWF::Update<KinInnovation<typename FILTERSTATE::mtState>,FILTERSTATE,KinUpdateMeas<typename FILTERSTATE::mtState>,KinUpdateNoise<typename FILTERSTATE::mtState>,
      KinOutlierDetection<typename FILTERSTATE::mtState>,true> Base;
  using Base::eval;
  using Base::meas_;
  using Base::preupdnoiP_;
  using Base::updnoiP_;
  using Base::doubleRegister_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtPredictionNoise mtPredictionNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  KinUpdate(): model_(nullptr) {
    qMB_.setIdentity();
    BrBM_.setZero();
    doubleRegister_.registerVector("BrBM",BrBM_);
    doubleRegister_.registerQuaternion("qMB",qMB_);
  };
  ~KinUpdate(){};
  QPD qMB_;
  V3D BrBM_;
  KINMODEL* model_;
  void setModelPtr(KINMODEL* model) {model_ = model;}
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{
    /* Kinematic measurements
     * 0 = MvM + MwM x MrMF + MJ*da + Noise
     */
    V3D M_w = state.template get<mtState::_aux>().MwIMmeas_-state.template get<mtState::_gyb>()+noise.template get<mtNoise::_gyr>();
    V3D MrMF;
    M3D dMrMF;
    for(int i=0;i<mtState::nFeet_;i++){
      MrMF = qMB_.rotate(V3D(-BrBM_+model_->forwardKinematicsBaseToFootInBaseFrame(meas_.template get<mtMeas::_enc>(i),i)));
      dMrMF = MPD(qMB_).matrix()*model_->getJacobianTranslationBaseToFoot(meas_.template get<mtMeas::_enc>(i),i);
      if(meas_.template get<mtMeas::_aux>().contactFlags_(i)){
        y.template get<mtInnovation::_kin>(i) = -state.template get<mtState::_vel>()
            + gSM(M_w)*MrMF + dMrMF*meas_.template get<mtMeas::_end>(i)+noise.template get<mtNoise::_kin>(i);
      } else {
        y.template get<mtInnovation::_kin>(i) = noise.template get<mtNoise::_kin>(i);
      }
    }
  }
  void jacState(Eigen::MatrixXd& J, const mtState& state) const{
    J.setZero();
    V3D MrMF;
    for(int i=0;i<mtState::nFeet_;i++){
      MrMF = qMB_.rotate(V3D(-BrBM_+model_->forwardKinematicsBaseToFootInBaseFrame(meas_.template get<mtMeas::_enc>(i),i)));
      if(meas_.template get<mtMeas::_aux>().contactFlags_(i)){
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtState::template getId<mtState::_vel>()) = -M3D::Identity();
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtState::template getId<mtState::_gyb>()) = gSM(MrMF);
      }
    }
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state) const{
    J.setZero();
    V3D MrMF;
    for(int i=0;i<mtState::nFeet_;i++){
      MrMF = qMB_.rotate(V3D(-BrBM_+model_->forwardKinematicsBaseToFootInBaseFrame(meas_.template get<mtMeas::_enc>(i),i)));
      if(meas_.template get<mtMeas::_aux>().contactFlags_(i)){
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtNoise::template getId<mtNoise::_gyr>()) = -gSM(MrMF);
      }
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_kin>(i),mtNoise::template getId<mtNoise::_kin>(i)) = M3D::Identity();
    }
  }
  void refreshProperties(){
  }
  void preProcess(mtFilterState& filterstate, const mtMeas& meas, bool& isFinished){
    isFinished = false;
    updnoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_gyr>(),mtNoise::template getId<mtNoise::_gyr>()) = filterstate.state_.template get<mtState::_aux>().wMeasCov_;
    preupdnoiP_.setZero();
    preupdnoiP_.template block<3,3>(mtPredictionNoise::template getId<mtPredictionNoise::_att>(),mtNoise::template getId<mtNoise::_gyr>()) = filterstate.state_.template get<mtState::_aux>().wMeasCov_;
    for(int i=0;i<mtState::nFeet_;i++){
      filterstate.state_.template get<mtState::_aux>().contactFlags_(i) = meas.template get<mtMeas::_aux>().contactFlags_(i);
    }
  }
  void postProcess(mtFilterState& filterstate, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    for(int i=0;i<mtState::nFeet_;i++){
      filterstate.state_.template get<mtState::_aux>().slipFlags_(i) = !outlierDetection.isOutlier(i);
    }
    isFinished = true;
  }
};



}


#endif /* KINUPDATE_HPP_ */
