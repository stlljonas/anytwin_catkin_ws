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

#ifndef FILTERSTATES_HPP_
#define FILTERSTATES_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"
#include "lightweight_filtering/FilterState.hpp"

namespace LWFM {

template<unsigned int nClone, unsigned int nFeet>
class StateAuxiliary: public LWF::AuxiliaryBase<StateAuxiliary<nClone,nFeet>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateAuxiliary(){
    for(unsigned int i=0;i<nClone;i++){
      cloneTimestamps_[i] = 0.0;
    }
    for(unsigned int i=0;i<nFeet;i++){
      contactFlags_(i) = false;
      slipFlags_(i) = false;
    }
    MwIMest_.setZero();
    MwIMmeas_.setZero();
    wMeasCov_.setIdentity();
    timeSinceLastValidPoseMeas_ = 1e6;
    isLastPoseMeasAnOutlier_ = false;
  };
  ~StateAuxiliary(){};
  double cloneTimestamps_[nClone];
  Eigen::Matrix<bool,nFeet,1> contactFlags_;
  Eigen::Matrix<bool,nFeet,1> slipFlags_;
  V3D MwIMest_;
  V3D MwIMmeas_;
  M3D wMeasCov_;
  double timeSinceLastValidPoseMeas_;
  bool isLastPoseMeasAnOutlier_;
};

template<unsigned int nFeet,unsigned int nClone, bool calVE, bool calPI, bool estFP>
class State: public LWF::State<
    LWF::TH_multiple_elements<LWF::VectorElement<3>,4+(int)calVE+(int)calPI>,
    LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,nClone>,(int)(nClone>0)>,
    LWF::TH_multiple_elements<LWF::QuaternionElement,1+(int)calVE+(int)calPI>,
    LWF::TH_multiple_elements<LWF::ArrayElement<LWF::QuaternionElement,nClone>,(int)(nClone>0)>,
    LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,(int)estFP>,
    StateAuxiliary<nClone,nFeet>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::State<
      LWF::TH_multiple_elements<LWF::VectorElement<3>,4+(int)calVE+(int)calPI>,
      LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,nClone>,(int)(nClone>0)>,
      LWF::TH_multiple_elements<LWF::QuaternionElement,1+(int)calVE+(int)calPI>,
      LWF::TH_multiple_elements<LWF::ArrayElement<LWF::QuaternionElement,nClone>,(int)(nClone>0)>,
      LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,nFeet>,(int)estFP>,
      StateAuxiliary<nClone,nFeet>> Base;
  using Base::D_;
  using Base::E_;
  static constexpr unsigned int nFeet_ = nFeet;
  static constexpr unsigned int nClone_ = nClone;
  static constexpr bool calVE_ = calVE;
  static constexpr bool calPI_ = calPI;
  static constexpr bool estFP_ = estFP;
  static constexpr unsigned int _pos = 0;                     // Position, MrIM
  static constexpr unsigned int _vel = _pos+1;                // Velocity, MvM
  static constexpr unsigned int _acb = _vel+1;                // Accelerometer Bias
  static constexpr unsigned int _gyb = _acb+1;                // Gyroscope Bias
  static constexpr unsigned int _vep = _gyb+(int)calVE;       // Camera-IMU extrinsics (position)
  static constexpr unsigned int _pip = _vep+(int)calPI;       // Transformation between map and odometry (position), JrJI
  static constexpr unsigned int _clp = _pip+(int)(nClone>0);  // Position clones
  static constexpr unsigned int _att = _clp+1;                // Attitude, qIM
  static constexpr unsigned int _vea = _att+(int)calVE;       // Camera-IMU extrinsics (attitute)
  static constexpr unsigned int _pia = _vea+(int)calPI;       // Transformation between map and odometry (attitude), qIJ
  static constexpr unsigned int _cla = _pia+(int)(nClone>0);  // Attitude clones
  static constexpr unsigned int _fpt = _cla+(int)estFP;       // Footpoints, robocentric, MrMF
  static constexpr unsigned int _aux = _fpt+1;                // Auxillary variables
  State(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    if(calVE) this->template getName<_vep>() = "VEpos";
    if(calPI) this->template getName<_pip>() = "PIpos";
    if(nClone>0) this->template getName<_clp>() = "CLpos";
    this->template getName<_att>() = "att";
    if(calVE) this->template getName<_vea>() = "VEatt";
    if(calPI) this->template getName<_pia>() = "PIatt";
    if(nClone>0) this->template getName<_cla>() = "CLatt";
    if(estFP) this->template getName<_fpt>() = "FP";
    this->template getName<_aux>() = "auxiliary";
  }
  ~State(){};
};



class PredictionMeas: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr unsigned int _acc = 0;
  static constexpr unsigned int _gyr = _acc+1;
  PredictionMeas(){
    static_assert(_gyr+1==E_,"Error with indices");
    this->template getName<_acc>() = "acc";
    this->template getName<_gyr>() = "gyr";
  }
  ~PredictionMeas(){};
};

template<typename STATE>
class PredictionNoise: public LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5+2*(int)(STATE::calVE_)+2*(int)(STATE::calPI_)>,
    LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,(int)STATE::estFP_>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using LWF::State<LWF::TH_multiple_elements<LWF::VectorElement<3>,5+2*(int)(STATE::calVE_)+2*(int)(STATE::calPI_)>,
      LWF::TH_multiple_elements<LWF::ArrayElement<LWF::VectorElement<3>,STATE::nFeet_>,(int)STATE::estFP_>>::E_;
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _vel = _pos+1;
  static constexpr unsigned int _acb = _vel+1;
  static constexpr unsigned int _gyb = _acb+1;
  static constexpr unsigned int _att = _gyb+1;
  static constexpr unsigned int _vep = _att+(int)STATE::calVE_;
  static constexpr unsigned int _vea = _vep+(int)STATE::calVE_;
  static constexpr unsigned int _pip = _vea+(int)STATE::calPI_;
  static constexpr unsigned int _pia = _pip+(int)STATE::calPI_;
  static constexpr unsigned int _fpt = _pia+(int)STATE::estFP_;
  PredictionNoise(){
    static_assert(_fpt+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_vel>() = "vel";
    this->template getName<_acb>() = "acb";
    this->template getName<_gyb>() = "gyb";
    if(STATE::calVE_) this->template getName<_vep>() = "VEpos";
    if(STATE::calPI_) this->template getName<_pip>() = "PIpos";
    this->template getName<_att>() = "att";
    if(STATE::calVE_) this->template getName<_vea>() = "VEatt";
    if(STATE::calPI_) this->template getName<_pia>() = "PIatt";
    if(STATE::estFP_) this->template getName<_fpt>() = "FP";
  }
  ~PredictionNoise(){};
};


template<unsigned int nFeet,unsigned int nClone, bool calVE, bool calPI, bool estFP,unsigned int noiseExtensionDim = 0>
class FilterState: public LWF::FilterState<State<nFeet,nClone,calVE,calPI,estFP>,PredictionMeas,PredictionNoise<State<nFeet,nClone,calVE,calPI,estFP>>,noiseExtensionDim>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::FilterState<State<nFeet,nClone,calVE,calPI,estFP>,PredictionMeas,PredictionNoise<State<nFeet,nClone,calVE,calPI,estFP>>,noiseExtensionDim> Base;
  typedef typename Base::mtState mtState;
  static constexpr int  D_ = mtState::D_;
  static constexpr unsigned int nFeet_ = mtState::nFeet_;
  static constexpr unsigned int nClone_ = mtState::nClone_;
  static constexpr bool calVE_ = mtState::calVE_;
  static constexpr bool calPI_ = mtState::calPI_;
  static constexpr bool estFP_ = mtState::estFP_;
  using Base::state_;
  using Base::cov_;
  FilterState(){
  }
  void clonePose(int origin, unsigned int target, double currentTime = 0.0){
    assert(origin<nClone && target<nClone);
    if(origin<0){
      state_.template get<mtState::_clp>(target) = state_.template get<mtState::_pos>();
      state_.template get<mtState::_cla>(target) = state_.template get<mtState::_att>();
      cov_.template block<D_,3>(0,state_.template getId<mtState::_clp>(target)) = cov_.template block<D_,3>(0,state_.template getId<mtState::_pos>());
      cov_.template block<D_,3>(0,state_.template getId<mtState::_cla>(target)) = cov_.template block<D_,3>(0,state_.template getId<mtState::_att>());
      cov_.template block<3,D_>(state_.template getId<mtState::_clp>(target),0) = cov_.template block<3,D_>(state_.template getId<mtState::_pos>(),0);
      cov_.template block<3,D_>(state_.template getId<mtState::_cla>(target),0) = cov_.template block<3,D_>(state_.template getId<mtState::_att>(),0);
      state_.template get<mtState::_aux>().cloneTimestamps_[target] = currentTime;
    } else {
      state_.template get<mtState::_clp>(target) = state_.template get<mtState::_pos>(origin);
      state_.template get<mtState::_cla>(target) = state_.template get<mtState::_cla>(origin);
      cov_.template block<D_,3>(0,state_.template getId<mtState::_clp>(target)) = cov_.template block<D_,3>(0,state_.template getId<mtState::_clp>(origin));
      cov_.template block<D_,3>(0,state_.template getId<mtState::_cla>(target)) = cov_.template block<D_,3>(0,state_.template getId<mtState::_cla>(origin));
      cov_.template block<3,D_>(state_.template getId<mtState::_clp>(target),0) = cov_.template block<3,D_>(state_.template getId<mtState::_clp>(origin),0);
      cov_.template block<3,D_>(state_.template getId<mtState::_cla>(target),0) = cov_.template block<3,D_>(state_.template getId<mtState::_cla>(origin),0);
      state_.template get<mtState::_aux>().cloneTimestamps_[target] = state_.template get<mtState::_aux>().cloneTimestamps_[origin];
    }
  }
  void cloneCurrentPose(unsigned int target,double currentTime){
    clonePose(-1,target,currentTime);
  }
  void cloneCurrentToTemp(double currentTime){
    cloneCurrentPose(1,currentTime);
  }
  void cloneCurrentToKeyframe(double currentTime){
    cloneCurrentPose(0,currentTime);
  }
  void cloneTempToKeyframe(){
    clonePose(1,0);
  }
  void initWithImuPose(V3D IrIM, QPD qMI){
    state_.template get<mtState::_pos>() = qMI.rotate(IrIM);
    state_.template get<mtState::_att>() = qMI.inverted();
  }
  void initWithAccelerometer(const V3D& fMeasInit){
    V3D unitZ(0,0,1);
    if(fMeasInit.norm()>1e-6){
      QPD q;
      q.setFromVectors(unitZ,state_.template get<mtState::_att>().rotate(fMeasInit));
      state_.template get<mtState::_att>() = q.inverted()*state_.template get<mtState::_att>();
    }
  }
  void resetPI(V3D JrJI, QPD qIJ,const Eigen::Matrix<double,6,6>& initCov){
    state_.template get<mtState::_pip>() = JrJI;
    state_.template get<mtState::_pia>() = qIJ;
    cov_.template block<D_,3>(0,state_.template getId<mtState::_pip>()).setZero();
    cov_.template block<D_,3>(0,state_.template getId<mtState::_pia>()).setZero();
    cov_.template block<3,D_>(state_.template getId<mtState::_pip>(),0).setZero();
    cov_.template block<3,D_>(state_.template getId<mtState::_pia>(),0).setZero();
    cov_.template block<3,3>(state_.template getId<mtState::_pip>(),state_.template getId<mtState::_pip>()) = initCov.block<3,3>(0,0);
    cov_.template block<3,3>(state_.template getId<mtState::_pip>(),state_.template getId<mtState::_pia>()) = initCov.block<3,3>(0,3);
    cov_.template block<3,3>(state_.template getId<mtState::_pia>(),state_.template getId<mtState::_pip>()) = initCov.block<3,3>(3,0);
    cov_.template block<3,3>(state_.template getId<mtState::_pia>(),state_.template getId<mtState::_pia>()) = initCov.block<3,3>(3,3);
  }
  void resetFP(unsigned int i, V3D MrMF,const Eigen::Matrix<double,3,3>& initCov){
    state_.template get<mtState::_fpt>(i) = MrMF;
    cov_.template block<D_,3>(0,state_.template getId<mtState::_fpt>(i)).setZero();
    cov_.template block<3,D_>(state_.template getId<mtState::_fpt>(i),0).setZero();
    cov_.template block<3,3>(state_.template getId<mtState::_fpt>(i),state_.template getId<mtState::_fpt>(i)) = initCov;
  }
  void setVE(V3D MrMV, QPD qVM){
    if(calVE){
      state_.template get<mtState::_vep>() = MrMV;
      state_.template get<mtState::_vea>() = qVM;
    }
  }
};

}


#endif /* FILTERSTATES_HPP_ */
