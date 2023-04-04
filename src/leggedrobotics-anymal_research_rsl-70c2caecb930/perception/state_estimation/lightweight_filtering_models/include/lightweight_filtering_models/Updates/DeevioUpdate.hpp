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

#ifndef DEEVIOUPDATE_HPP_
#define DEEVIOUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"

namespace LWFM {

class DeevioInnovation: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  DeevioInnovation(){
    static_assert(_att+1==E_,"Error with indices");
  };
  ~DeevioInnovation(){};
};
class DeevioUpdateMeasAuxillary: public LWF::AuxiliaryBase<DeevioUpdateMeasAuxillary>{
 public:
  DeevioUpdateMeasAuxillary(){
    currentTimeStamp_ = 0.0;
    referenceTimeStamp_ = 0.0;
  };
  ~DeevioUpdateMeasAuxillary(){};
  double currentTimeStamp_;
  double referenceTimeStamp_;
};
class DeevioUpdateMeas: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement,DeevioUpdateMeasAuxillary>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  static constexpr unsigned int _aux = _att+1;
  DeevioUpdateMeas(){
    static_assert(_aux+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_att>() = "att";
    this->template getName<_aux>() = "auxiliary";
  };
  ~DeevioUpdateMeas(){};
};
class DeevioUpdateNoise: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  DeevioUpdateNoise(){
    static_assert(_att+1==E_,"Error with indices");
  };
  ~DeevioUpdateNoise(){};
};
class DeevioOutlierDetection: public LWF::OutlierDetection<
    LWF::ODEntry<DeevioInnovation::template getId<DeevioInnovation::_pos>(),6>>{
};

template<typename FILTERSTATE>
class DeevioUpdate: public LWF::Update<DeevioInnovation,FILTERSTATE,DeevioUpdateMeas,DeevioUpdateNoise,
    DeevioOutlierDetection,false>{
 public:
  typedef LWF::Update<DeevioInnovation,FILTERSTATE,DeevioUpdateMeas,DeevioUpdateNoise,
      DeevioOutlierDetection,false> Base;
  using Base::eval;
  using Base::doubleRegister_;
  using Base::meas_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  DeevioUpdate(){
    qVM_.setIdentity();
    MrMV_.setZero();
    doubleRegister_.registerVector("MrMV",MrMV_);
    doubleRegister_.registerQuaternion("qVM",qVM_);
  };
  ~DeevioUpdate(){};
  QPD qVM_;
  V3D MrMV_;
  void setVE(V3D MrMV, QPD qVM){
    MrMV_ = MrMV;
    qVM_ = qVM;
  }
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{
    /* Relative 6DOF measurements
     * NrIV = qIN^T*qIM*(MrIM + MrMV)
     * qNW*WrWV = NrIV - NrIN - NrNW
     * qVW = qVM*qIM^T*qIN*qWN^T
     */
    if(meas_.template get<mtMeas::_aux>().referenceTimeStamp_ == state.template get<mtState::_aux>().cloneTimestamps_[0]){
      V3D MrMV = MrMV_;
      QPD qVM = qVM_;
      if(mtState::calVE_){
        MrMV = state.template get<mtState::_vep>();
        qVM = state.template get<mtState::_vea>();
      }
      y.template get<mtInnovation::_pos>() =
          (state.template get<mtState::_cla>(0).inverted()*state.template get<mtState::_att>()).rotate(V3D(state.template get<mtState::_pos>()+MrMV))
          - state.template get<mtState::_clp>(0) - MrMV - qVM.inverted().rotate(meas_.template get<mtMeas::_pos>()) + noise.template get<mtNoise::_pos>();
      QPD attNoise = attNoise.exponentialMap(noise.template get<mtNoise::_att>());
      y.template get<mtInnovation::_att>() = attNoise*qVM*state.template get<mtState::_att>().inverted()
          *state.template get<mtState::_cla>(0)*qVM.inverted()*meas_.template get<mtMeas::_att>().inverted();
    } else {
      y.template get<mtInnovation::_pos>().setZero();
      y.template get<mtInnovation::_att>().setIdentity();
    }
  }
  void jacState(Eigen::MatrixXd& J, const mtState& state) const{
    J.setZero();
    if(meas_.template get<mtMeas::_aux>().referenceTimeStamp_ == state.template get<mtState::_aux>().cloneTimestamps_[0]){
      V3D MrMV = MrMV_;
      QPD qVM = qVM_;
      if(mtState::calVE_){
        MrMV = state.template get<mtState::_vep>();
        qVM = state.template get<mtState::_vea>();
      }
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pos>()) =
          MPD(state.template get<mtState::_cla>(0).inverted()*state.template get<mtState::_att>()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_att>()) =
          -gSM((state.template get<mtState::_cla>(0).inverted()*state.template get<mtState::_att>()).rotate(V3D(state.template get<mtState::_pos>()+MrMV)))
          * MPD(state.template get<mtState::_cla>(0).inverted()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_clp>(0)) =
          -M3D::Identity();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_cla>(0)) =
          gSM((state.template get<mtState::_cla>(0).inverted()*state.template get<mtState::_att>()).rotate(V3D(state.template get<mtState::_pos>()+MrMV)))
          * MPD(state.template get<mtState::_cla>(0).inverted()).matrix();
      if(mtState::calVE_){
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_vep>()) =
            (MPD(state.template get<mtState::_cla>(0).inverted()*state.template get<mtState::_att>()).matrix()-M3D::Identity());
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_vea>()) =
            -gSM(qVM.inverted().rotate(meas_.template get<mtMeas::_pos>())) * MPD(qVM.inverted()).matrix();
      }
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_att>()) = -MPD(qVM*state.template get<mtState::_att>().inverted()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_cla>(0)) = MPD(qVM*state.template get<mtState::_att>().inverted()).matrix();
      if(mtState::calVE_){
        J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_vea>()) =
            M3D::Identity()-MPD(qVM*state.template get<mtState::_att>().inverted()*state.template get<mtState::_cla>(0)*qVM.inverted()).matrix();
      }
    }
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state) const{
    J.setZero();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtNoise::template getId<mtNoise::_att>()) = M3D::Identity();
  }
  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished){
    isFinished = false;
    if(meas.template get<mtMeas::_aux>().referenceTimeStamp_ != filterState.state_.template get<mtState::_aux>().cloneTimestamps_[0]){
      if(meas.template get<mtMeas::_aux>().referenceTimeStamp_ == filterState.state_.template get<mtState::_aux>().cloneTimestamps_[1]){
        filterState.cloneTempToKeyframe();
      } else {
        std::cout << "Invalid reference time stamp: "<< meas.template get<mtMeas::_aux>().referenceTimeStamp_ << " (Keyframe at: " << filterState.state_.template get<mtState::_aux>().cloneTimestamps_[0] << ")" << std::endl;
      }
    }
  }
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    filterState.cloneCurrentToTemp(meas.template get<mtMeas::_aux>().currentTimeStamp_);
    isFinished = true;
  }
};

}


#endif /* DEEVIOUPDATE_HPP_ */
