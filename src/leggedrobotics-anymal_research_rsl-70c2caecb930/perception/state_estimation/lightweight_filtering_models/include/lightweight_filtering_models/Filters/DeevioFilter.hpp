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

#ifndef DEEVIO_FILTER_HPP_
#define DEEVIO_FILTER_HPP_

#include "lightweight_filtering_models/FilterStates.hpp"
#include "lightweight_filtering/FilterBase.hpp"
#include "lightweight_filtering_models/Updates/DeevioUpdate.hpp"
#include "lightweight_filtering_models/Predictions/ImuPrediction.hpp"

namespace rot = kindr;

namespace LWFM {

template<bool CalVE>
class DeevioFilter:public LWF::FilterBase<ImuPrediction<FilterState<0,2,CalVE,false,false>>,DeevioUpdate<FilterState<0,2,CalVE,false,false>>>{
 public:
  typedef LWF::FilterBase<ImuPrediction<FilterState<0,2,CalVE,false,false>>,DeevioUpdate<FilterState<0,2,CalVE,false,false>>> Base;
  using Base::init_;
  using Base::reset;
  using Base::predictionTimeline_;
  using Base::safe_;
  using Base::front_;
  using Base::readFromInfo;
  using Base::mUpdates_;
  typedef typename Base::mtFilterState mtFilterState;
  typedef typename Base::mtPrediction mtPrediction;
  DeevioFilter(){
    std::get<0>(mUpdates_).outlierDetection_.setEnabledAll(true);
  }
  ~DeevioFilter(){};
  bool getDeevioOutlierFlag(const mtFilterState& filterState){
    return std::get<0>(mUpdates_).outlierDetection_.isOutlier(0);
  }
  void resetToImuPose(Eigen::Vector3d IrIM, rot::RotationQuaternionPD qMI, double t = 0.0){
    init_.state_.initWithImuPose(IrIM,qMI);
    reset(t);
  }
  void resetWithAccelerometer(const Eigen::Vector3d& fMeasInit, double t = 0.0){
    init_.state_.initWithAccelerometer(fMeasInit);
    reset(t);
  }
  void resetWithKeyframe(double t = 0.0) {
    double imuMeasTime = 0.0;
    if(predictionTimeline_.getLastTime(imuMeasTime)){  // Find close accelerometer measurement
      std::cout << "-- Filter: Reseting with IMU!" << std::endl;
      resetWithAccelerometer(predictionTimeline_.measMap_[imuMeasTime].template get<mtPrediction::mtMeas::_acc>(),t); // Initialize with accelerometer
    } else {
      std::cout << "-- Filter: Reseting without IMU!" << std::endl;
      reset(t);
    }
    safe_.state_.cloneCurrentToKeyframe(safe_.cov_,t);
    front_ = safe_;
  }
  void setVE(V3D MrMV, QPD qVM){
    init_.state_.setVE(MrMV,qVM);
    std::get<0>(mUpdates_).setVE(MrMV,qVM);
  }
};

}


#endif /* DEEVIO_FILTER_HPP_ */
