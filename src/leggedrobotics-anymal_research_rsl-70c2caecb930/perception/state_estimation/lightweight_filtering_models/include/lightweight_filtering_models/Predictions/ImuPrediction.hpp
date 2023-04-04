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

#ifndef IMUPREDICTION_HPP_
#define IMUPREDICTION_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Prediction.hpp"
#include "lightweight_filtering/State.hpp"

namespace LWFM {

template<typename FILTERSTATE>
class ImuPrediction: public LWF::Prediction<FILTERSTATE>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::Prediction<FILTERSTATE> Base;
  using Base::evalPrediction;
  using Base::prenoiP_;
  using Base::meas_;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtState mtState;
  typedef typename Base::mtMeas mtMeas;
  typedef typename Base::mtNoise mtNoise;
  const V3D g_;
  ImuPrediction():g_(0,0,-9.81){
    this->disablePreAndPostProcessingWarning_ = true;
  };
  ~ImuPrediction(){};
  void evalPrediction(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
    output.template get<mtState::_aux>().MwIMmeas_ = meas_.template get<mtMeas::_gyr>();
    output.template get<mtState::_aux>().MwIMest_ = meas_.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>();
    V3D dOmega = dt*(output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt(dt));
    QPD dQ = dQ.exponentialMap(dOmega);
    output.template get<mtState::_att>() = state.template get<mtState::_att>()*dQ;
    output.template get<mtState::_pos>() = (M3D::Identity()-gSM(dOmega))
        *state.template get<mtState::_pos>()-dt*(state.template get<mtState::_vel>()-noise.template get<mtNoise::_pos>()/sqrt(dt));
    output.template get<mtState::_vel>() = (M3D::Identity()-gSM(dOmega))
        *state.template get<mtState::_vel>()-dt*(meas_.template get<mtMeas::_acc>()-state.template get<mtState::_acb>()+state.template get<mtState::_att>().inverseRotate(g_)-noise.template get<mtNoise::_vel>()/sqrt(dt));
    output.template get<mtState::_acb>() = state.template get<mtState::_acb>()+noise.template get<mtNoise::_acb>()*sqrt(dt);
    output.template get<mtState::_gyb>() = state.template get<mtState::_gyb>()+noise.template get<mtNoise::_gyb>()*sqrt(dt);
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      output.template get<mtState::_aux>().contactFlags_(i) = state.template get<mtState::_aux>().contactFlags_(i);
    }
    output.template get<mtState::_aux>().wMeasCov_ = prenoiP_.template block<3,3>(mtNoise::template getId<mtNoise::_att>(),mtNoise::template getId<mtNoise::_att>())*dt;
    predictPoseClones<(FILTERSTATE::nClone_>0)>(output,state,noise,dt);
    predictCalVE<FILTERSTATE::calVE_>(output,state,noise,dt);
    predictCalPI<FILTERSTATE::calPI_>(output,state,noise,dt);
    predictFootpoints<FILTERSTATE::estFP_>(output,state,noise,dt);
    output.template get<mtState::_aux>().timeSinceLastValidPoseMeas_ = state.template get<mtState::_aux>().timeSinceLastValidPoseMeas_+dt;
    output.fix();
  }
  template<bool hasClones,typename std::enable_if<hasClones>::type* = nullptr>
  inline void predictPoseClones(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
    for(unsigned int i=0;i<mtState::nClone_;i++){
      output.template get<mtState::_clp>(i) = state.template get<mtState::_clp>(i);
      output.template get<mtState::_cla>(i) = state.template get<mtState::_cla>(i);
      output.template get<mtState::_aux>().cloneTimestamps_[i] = state.template get<mtState::_aux>().cloneTimestamps_[i];
    }
  }
  template<bool hasClones,typename std::enable_if<!hasClones>::type* = nullptr>
  inline void predictPoseClones(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
  }
  template<bool estFP,typename std::enable_if<estFP>::type* = nullptr>
  inline void predictFootpoints(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
    V3D dOmega = dt*(output.template get<mtState::_aux>().MwIMest_+noise.template get<mtNoise::_att>()/sqrt(dt));
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      output.template get<mtState::_fpt>(i) = (M3D::Identity()-gSM(dOmega)) * state.template get<mtState::_fpt>(i)
          +dt*(state.template get<mtState::_vel>()+noise.template get<mtNoise::_fpt>(i)/sqrt(dt));
    }
  }
  template<bool estFP,typename std::enable_if<!estFP>::type* = nullptr>
  inline void predictFootpoints(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
  }
  template<bool calPI,typename std::enable_if<calPI>::type* = nullptr>
  inline void predictCalPI(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
    output.template get<mtState::_pip>() = state.template get<mtState::_pip>()+noise.template get<mtNoise::_pip>()*sqrt(dt);
    QPD dQ = dQ.exponentialMap(noise.template get<mtNoise::_pia>()*sqrt(dt));
    output.template get<mtState::_pia>() = dQ*state.template get<mtState::_pia>();
  }
  template<bool calPI,typename std::enable_if<!calPI>::type* = nullptr>
  inline void predictCalPI(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
  }
  template<bool calVE,typename std::enable_if<calVE>::type* = nullptr>
  inline void predictCalVE(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
    output.template get<mtState::_vep>() = state.template get<mtState::_vep>()+noise.template get<mtNoise::_vep>()*sqrt(dt);
    QPD dQ = dQ.exponentialMap(noise.template get<mtNoise::_vea>()*sqrt(dt));
    output.template get<mtState::_vea>() = dQ*state.template get<mtState::_vea>();
  }
  template<bool calVE,typename std::enable_if<!calVE>::type* = nullptr>
  inline void predictCalVE(mtState& output, const mtState& state, const mtNoise& noise, double dt) const{
  }
  void noMeasCase(mtFilterState& filterState, mtMeas& meas, double dt){
    meas.template get<mtMeas::_gyr>() = filterState.state_.template get<mtState::_gyb>();
    meas.template get<mtMeas::_acc>() = filterState.state_.template get<mtState::_acb>()-filterState.state_.template get<mtState::_att>().inverseRotate(g_);
  }
  void jacPreviousState(Eigen::MatrixXd& J, const mtState& state, double dt) const{
    V3D dOmega = dt*(meas_.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    J.setIdentity(); // Handles clone and calibrations
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_pos>()) =
        (M3D::Identity()-gSM(dOmega));
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_vel>()) =
        -dt*M3D::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtState::template getId<mtState::_gyb>()) =
        -dt*gSM(state.template get<mtState::_pos>());
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_vel>()) =
        (M3D::Identity()-gSM(dOmega));
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_acb>()) =
        dt*M3D::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_gyb>()) =
        -dt*gSM(state.template get<mtState::_vel>());
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtState::template getId<mtState::_att>()) =
        -dt*MPD(state.template get<mtState::_att>()).matrix().transpose()*gSM(g_);
    J.template block<3,3>(mtState::template getId<mtState::_acb>(),mtState::template getId<mtState::_acb>()) =
        M3D::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtState::template getId<mtState::_gyb>()) =
        M3D::Identity();
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_gyb>()) =
        -dt*MPD(state.template get<mtState::_att>()).matrix()*Lmat(dOmega);
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtState::template getId<mtState::_att>()) =
        M3D::Identity();
    footpointsInputJac<FILTERSTATE::estFP_>(J,state,dt);
  }
  template<bool estFP,typename std::enable_if<estFP>::type* = nullptr>
  inline void footpointsInputJac(Eigen::MatrixXd& J, const mtState& state, double dt) const{
    V3D dOmega = dt*(meas_.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    for(unsigned int i=0;i<mtState::nFeet_;i++){
        J.template block<3,3>(mtState::template getId<mtState::_fpt>(i),mtState::template getId<mtState::_fpt>(i)) =
            (M3D::Identity()-gSM(dOmega));
        J.template block<3,3>(mtState::template getId<mtState::_fpt>(i),mtState::template getId<mtState::_vel>()) =
            dt*M3D::Identity();
        J.template block<3,3>(mtState::template getId<mtState::_fpt>(i),mtState::template getId<mtState::_gyb>()) =
            -dt*gSM(state.template get<mtState::_fpt>(i));
    }
  }
  template<bool estFP,typename std::enable_if<!estFP>::type* = nullptr>
  inline void footpointsInputJac(Eigen::MatrixXd& J, const mtState& state, double dt) const{
  }
  void jacNoise(Eigen::MatrixXd& J, const mtState& state, double dt) const{
    V3D dOmega = dt*(meas_.template get<mtMeas::_gyr>()-state.template get<mtState::_gyb>());
    J.setZero();
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_pos>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_pos>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.template get<mtState::_pos>())*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_vel>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_vel>(),mtNoise::template getId<mtNoise::_att>()) =
        gSM(state.template get<mtState::_vel>())*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_acb>(),mtNoise::template getId<mtNoise::_acb>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_gyb>(),mtNoise::template getId<mtNoise::_gyb>()) = M3D::Identity()*sqrt(dt);
    J.template block<3,3>(mtState::template getId<mtState::_att>(),mtNoise::template getId<mtNoise::_att>()) =
        MPD(state.template get<mtState::_att>()).matrix()*Lmat(dOmega)*sqrt(dt);
    if(FILTERSTATE::calVE_){ // TODO
      J.template block<3,3>(mtState::template getId<mtState::_vep>(),mtNoise::template getId<mtNoise::_vep>()) = M3D::Identity()*sqrt(dt);
      J.template block<3,3>(mtState::template getId<mtState::_vea>(),mtNoise::template getId<mtNoise::_vea>()) = M3D::Identity()*sqrt(dt);
    }
    if(FILTERSTATE::calPI_){ // TODO
      J.template block<3,3>(mtState::template getId<mtState::_pip>(),mtNoise::template getId<mtNoise::_pip>()) = M3D::Identity()*sqrt(dt);
      J.template block<3,3>(mtState::template getId<mtState::_pia>(),mtNoise::template getId<mtNoise::_pia>()) = M3D::Identity()*sqrt(dt);
    }
    footpointsNoiseJac<FILTERSTATE::estFP_>(J,state,dt);
  }
  template<bool estFP,typename std::enable_if<estFP>::type* = nullptr>
  inline void footpointsNoiseJac(Eigen::MatrixXd& J, const mtState& state, double dt) const{
    for(unsigned int i=0;i<mtState::nFeet_;i++){
      J.template block<3,3>(mtState::template getId<mtState::_fpt>(i),mtNoise::template getId<mtNoise::_fpt>(i)) = M3D::Identity()*sqrt(dt);
      J.template block<3,3>(mtState::template getId<mtState::_fpt>(i),mtNoise::template getId<mtNoise::_att>()) =
          gSM(state.template get<mtState::_fpt>(i))*sqrt(dt);
    }
  }
  template<bool estFP,typename std::enable_if<!estFP>::type* = nullptr>
  inline void footpointsNoiseJac(Eigen::MatrixXd& J, const mtState& state, double dt) const{
  }
};

}


#endif /* IMUPREDICTION_HPP_ */
