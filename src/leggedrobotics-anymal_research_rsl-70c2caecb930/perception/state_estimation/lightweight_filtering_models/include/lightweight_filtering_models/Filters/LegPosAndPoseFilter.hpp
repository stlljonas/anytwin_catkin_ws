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

#ifndef LEGPOSANDPOSEFILTER_HPP_
#define LEGPOSANDPOSEFILTER_HPP_

#include "lightweight_filtering_models/FilterStates.hpp"
#include "lightweight_filtering/FilterBase.hpp"
#include "lightweight_filtering_models/Updates/PoseUpdate.hpp"
#include "lightweight_filtering_models/Updates/Kin2Update.hpp"
#include "lightweight_filtering_models/Predictions/ImuPrediction.hpp"


namespace LWFM {

template<unsigned int nFeet, typename KINMODEL>
class LegPosAndPoseFilter:public LWF::FilterBase<ImuPrediction<FilterState<nFeet,0,false,true,true>>,PoseUpdate<FilterState<nFeet,0,false,true,true>>,Kin2Update<FilterState<nFeet,0,false,true,true>,KINMODEL>>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::FilterBase<ImuPrediction<FilterState<nFeet,0,false,true,true>>,PoseUpdate<FilterState<nFeet,0,false,true,true>>,Kin2Update<FilterState<nFeet,0,false,true,true>,KINMODEL>> Base;
  using Base::init_;
  using Base::reset;
  using Base::predictionTimeline_;
  using Base::safe_;
  using Base::front_;
  using Base::readFromInfo;
  using Base::mPrediction_;
  using Base::mUpdates_;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtPrediction mtPrediction;
  typedef typename Base::mtState mtState;
  LegPosAndPoseFilter(){
    std::get<0>(mUpdates_).outlierDetection_.setEnabledAll(true);
    std::get<1>(mUpdates_).outlierDetection_.setEnabledAll(true);
    reset(0.0);
  }
  ~LegPosAndPoseFilter(){};
  void resetToImuPose(V3D IrIM, QPD qMI, double t = 0.0){
    init_.initWithImuPose(IrIM,qMI);
    reset(t);
  }
  void resetToBodyPose(V3D IrIB, QPD qBI, double t = 0.0){
    V3D IrIM = IrIB + qBI.inverseRotate(std::get<1>(mUpdates_).BrBM_);
    QPD qMI = std::get<1>(mUpdates_).qMB_*qBI;
    init_.initWithImuPose(IrIM,qMI);
    reset(t);
  }
  void resetWithAccelerometer(const V3D& fMeasInit, double t = 0.0){
    init_.initWithAccelerometer(fMeasInit);
    reset(t);
  }
  void resetPI(){
    Eigen::Matrix<double,6,6> initCov;
    initCov.block<3,3>(0,0) = init_.cov_.template block<3,3>(mtState::template getId<mtState::_pip>(),mtState::template getId<mtState::_pip>());
    initCov.block<3,3>(0,3) = init_.cov_.template block<3,3>(mtState::template getId<mtState::_pip>(),mtState::template getId<mtState::_pia>());
    initCov.block<3,3>(3,0) = init_.cov_.template block<3,3>(mtState::template getId<mtState::_pia>(),mtState::template getId<mtState::_pip>());
    initCov.block<3,3>(3,3) = init_.cov_.template block<3,3>(mtState::template getId<mtState::_pia>(),mtState::template getId<mtState::_pia>());
    safe_.resetPI(init_.state_.template get<mtState::_pip>(),init_.state_.template get<mtState::_pia>(),initCov);
  }
  void resetPoseOfBodyInMap(V3D JrJB, QPD qBJ){
    Eigen::Matrix<double,6,6> initCov;
    initCov.block<3,3>(0,0) = init_.cov_.template block<3,3>(mtState::template getId<mtState::_pip>(),mtState::template getId<mtState::_pip>());
    initCov.block<3,3>(0,3) = init_.cov_.template block<3,3>(mtState::template getId<mtState::_pip>(),mtState::template getId<mtState::_pia>());
    initCov.block<3,3>(3,0) = init_.cov_.template block<3,3>(mtState::template getId<mtState::_pia>(),mtState::template getId<mtState::_pip>());
    initCov.block<3,3>(3,3) = init_.cov_.template block<3,3>(mtState::template getId<mtState::_pia>(),mtState::template getId<mtState::_pia>());
    /* JrJI = JrJB + JrBI
     *      = JrJB - JrIB
     *      = JrJB - qJM * MrIB
     *      = JrJB - qJM * (MrIM + MrMB)
     *      = JrJB - qBJ^T*qMB^T * (MrIM - qMB*BrBM)
     * qIJ = qIM * qMB * qBJ
     */
    V3D JrJI = JrJB - (qBJ.inverted()*std::get<1>(mUpdates_).qMB_.inverted()).rotate(V3D(safe_.state_.template get<mtState::_pos>() - std::get<1>(mUpdates_).qMB_.rotate(std::get<1>(mUpdates_).BrBM_)));
    QPD qIJ = safe_.state_.template get<mtState::_att>()*std::get<1>(mUpdates_).qMB_*qBJ;
    safe_.resetPI(JrJI,qIJ,initCov);
  }
};

}


#endif /* LEGPOSANDPOSEFILTER_HPP_ */
