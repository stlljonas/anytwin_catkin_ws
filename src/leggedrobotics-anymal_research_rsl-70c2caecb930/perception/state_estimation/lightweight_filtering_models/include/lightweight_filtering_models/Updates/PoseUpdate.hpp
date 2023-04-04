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

#ifndef POSEUPDATE_HPP_
#define POSEUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"

namespace LWFM {

class PoseInnovation: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseInnovation(){
    static_assert(_att+1==E_,"Error with indices");
  };
  ~PoseInnovation(){};
};
class PoseUpdateMeas: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseUpdateMeas(){
    static_assert(_att+1==E_,"Error with indices");
  };
  ~PoseUpdateMeas(){};
};
class PoseUpdateNoise: public LWF::State<LWF::VectorElement<3>,LWF::VectorElement<3>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  PoseUpdateNoise(){
    static_assert(_att+1==E_,"Error with indices");
    this->template getName<_pos>() = "pos";
    this->template getName<_att>() = "att";
  };
  ~PoseUpdateNoise(){};
};
class PoseOutlierDetection: public LWF::OutlierDetection<
    LWF::ODEntry<PoseInnovation::template getId<PoseInnovation::_pos>(),6>>{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename FILTERSTATE>
class PoseUpdate: public LWF::Update<PoseInnovation,FILTERSTATE,PoseUpdateMeas,PoseUpdateNoise,
    PoseOutlierDetection,false>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::Update<PoseInnovation,FILTERSTATE,PoseUpdateMeas,PoseUpdateNoise,
      PoseOutlierDetection,false> Base;
  using Base::eval;
  using Base::doubleRegister_;
  using Base::meas_;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtInnovation mtInnovation;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  typedef typename Base::mtOutlierDetection mtOutlierDetection;
  PoseUpdate(){
    static_assert(mtState::calPI_,"PoseUpdate requires enabling of calibration");
    qVM_.setIdentity();
    MrMV_.setZero();
    doubleRegister_.registerVector("MrMV",MrMV_);
    doubleRegister_.registerQuaternion("qVM",qVM_);
  }
  ~PoseUpdate(){}
  QPD qVM_;
  V3D MrMV_;
  void evalInnovation(mtInnovation& y, const mtState& state, const mtNoise& noise) const{
    /* Relative 6DOF measurements
     * JrJV = JrJI + qIJ^T*qIM*(MrIM+MrMV_)
     * qVJ = qVM_*qIM^T*qIJ
     */
    y.template get<mtInnovation::_pos>() = state.template get<mtState::_pip>() + (state.template get<mtState::_pia>().inverted()*state.template get<mtState::_att>()).rotate(
        V3D(state.template get<mtState::_pos>()+MrMV_)) - meas_.template get<mtMeas::_pos>() + noise.template get<mtNoise::_pos>();
    QPD attNoise = attNoise.exponentialMap(noise.template get<mtNoise::_att>());
    y.template get<mtInnovation::_att>() = attNoise*qVM_*state.template get<mtState::_att>().inverted()*state.template get<mtState::_pia>()
        *meas_.template get<mtMeas::_att>().inverted();
  }
  void jacState(Eigen::MatrixXd& J, const mtState& state) const{
    J.setZero();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pos>()) =
          MPD(state.template get<mtState::_pia>().inverted()*state.template get<mtState::_att>()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pip>()) =
                M3D::Identity();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_att>()) =
          MPD(state.template get<mtState::_pia>().inverted()).matrix()*
          -gSM(state.template get<mtState::_att>().rotate(V3D(state.template get<mtState::_pos>()+MrMV_)));
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtState::template getId<mtState::_pia>()) =
          gSM((state.template get<mtState::_pia>().inverted()*state.template get<mtState::_att>()).rotate(
              V3D(state.template get<mtState::_pos>()+MrMV_)))*MPD(state.template get<mtState::_pia>().inverted()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_att>()) =
          -MPD(qVM_*state.template get<mtState::_att>().inverted()).matrix();
      J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtState::template getId<mtState::_pia>()) =
          MPD(qVM_*state.template get<mtState::_att>().inverted()).matrix();
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state) const{
    J.setZero();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity();
    J.template block<3,3>(mtInnovation::template getId<mtInnovation::_att>(),mtNoise::template getId<mtNoise::_att>()) = M3D::Identity();
  }
  void preProcess(mtFilterState& filterState, const mtMeas& meas, bool& isFinished){
    isFinished = false;
  }
  void postProcess(mtFilterState& filterState, const mtMeas& meas, const mtOutlierDetection& outlierDetection, bool& isFinished){
    if(!outlierDetection.isOutlier(0)){
      filterState.state_.template get<mtState::_aux>().timeSinceLastValidPoseMeas_ = 0.0;
    }
    filterState.state_.template get<mtState::_aux>().isLastPoseMeasAnOutlier_ = outlierDetection.isOutlier(0);
    isFinished = true;
  }
};

}


#endif /* POSEUPDATE_HPP_ */
