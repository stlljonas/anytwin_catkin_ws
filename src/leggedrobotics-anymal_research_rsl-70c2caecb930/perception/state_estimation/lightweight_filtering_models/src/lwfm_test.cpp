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

#include <ros/ros.h>
#include "lightweight_filtering_models/FilterStates.hpp"
#include "lightweight_filtering_models/Predictions/ImuPrediction.hpp"
#include "lightweight_filtering_models/Updates/PoseUpdate.hpp"
#include "lightweight_filtering_models/Updates/KinUpdate.hpp"
#include "lightweight_filtering_models/Updates/Kin2Update.hpp"
#include "lightweight_filtering_models/Updates/DeevioUpdate.hpp"
#include "lightweight_filtering_models/Testing/AnymalModel.hpp"
#include "lightweight_filtering_models/Filters/DeevioFilter.hpp"
#include "lightweight_filtering_models/Filters/LegPosAndPoseFilter.hpp"
#include "lightweight_filtering_models/Filters/LegVelAndPoseFilter.hpp"
#include "lightweight_filtering_models/Filters/ImuAndDynFilter.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/TransformStandardPoseCF.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/StandardPoseOutputCF.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/StandardKinOutputCF.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/OdometryToBaseOutputCT.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/RelPoseOutputCF.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/KeyframeOutputCF.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/ExtrinsicsOutputCF.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/DeevioOutputCF.hpp"
#include "lightweight_filtering_models/Predictions/ImuAndDynPrediction.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "lwfm_test");
  ros::NodeHandle nh;
  unsigned int s;
  anymal_model::AnymalModel* model = new anymal_model::AnymalModel();

  typedef LWFM::FilterState<4,3,true,true,true> mtFilterState;
  typedef mtFilterState::mtState mtState;
  mtFilterState filterState;
  mtState state;
  state.setRandom(s);
  state.template get<mtState::_aux>().contactFlags_(0) = true;

  LWFM::ImuPrediction<mtFilterState> mPrediction;
  std::cout << "Test IMU Prediction" << std::endl;
  mPrediction.testPredictionJacs();

  LWFM::PoseUpdate<mtFilterState> mPoseUpdate;
  std::cout << "Test Pose Update" << std::endl;
  mPoseUpdate.testUpdateJacs();

  LWFM::KinUpdate<LWFM::FilterState<4,3,true,true,true,LWFM::KinUpdateNoise<mtFilterState::mtState>::D_>,anymal_model::AnymalModel> mKinUpdate;
  typename LWFM::KinUpdate<LWFM::FilterState<4,3,true,true,true,LWFM::KinUpdateNoise<mtFilterState::mtState>::D_>,anymal_model::AnymalModel>::mtMeas mKinUpdateMeas;
  mKinUpdate.setModelPtr(model);
  mKinUpdateMeas.setRandom(s);
  mKinUpdateMeas.template get<LWFM::KinUpdate<LWFM::FilterState<4,3,true,true,true,LWFM::KinUpdateNoise<mtFilterState::mtState>::D_>,anymal_model::AnymalModel>::mtMeas::_aux>().contactFlags_(0) = true;
  mKinUpdateMeas.template get<LWFM::KinUpdate<LWFM::FilterState<4,3,true,true,true,LWFM::KinUpdateNoise<mtFilterState::mtState>::D_>,anymal_model::AnymalModel>::mtMeas::_aux>().contactFlags_(2) = true;
  std::cout << "Test Kin Update" << std::endl;
  mKinUpdate.testUpdateJacs(state,mKinUpdateMeas);

  LWFM::Kin2Update<mtFilterState,anymal_model::AnymalModel> mKin2Update;
  typename LWFM::Kin2Update<mtFilterState,anymal_model::AnymalModel>::mtMeas mKin2UpdateMeas;
  mKin2Update.setModelPtr(model);
  mKin2UpdateMeas.setRandom(s);
  mKin2UpdateMeas.template get<LWFM::Kin2Update<mtFilterState,anymal_model::AnymalModel>::mtMeas::_aux>().contactFlags_(0) = true;
  mKin2UpdateMeas.template get<LWFM::Kin2Update<mtFilterState,anymal_model::AnymalModel>::mtMeas::_aux>().contactFlags_(2) = true;
  std::cout << "Test Kin2 Update" << std::endl;
  mKin2Update.testUpdateJacs(state,mKin2UpdateMeas);

  LWFM::DeevioUpdate<mtFilterState> mDeevioUpdate;
  std::cout << "Test Deevio Update" << std::endl;
  mDeevioUpdate.testUpdateJacs();

  LWFM::DeevioFilter<true> deevioFilter;
  LWFM::LegPosAndPoseFilter<4,anymal_model::AnymalModel> legPosAndPoseFilter;
  LWFM::LegVelAndPoseFilter<4,anymal_model::AnymalModel> legVelAndPoseFilter;

  LWFM::TransformStandardPoseCF transformStandardPoseCF;
  std::cout << "Test TransformStandardPoseCF" << std::endl;
  transformStandardPoseCF.testTransformJac();

  LWFM::StandardPoseOutputCF<LWFM::PoseUpdate<mtFilterState>> standardPoseOutputCF(mPoseUpdate);
  std::cout << "Test StandardPoseOutputCF" << std::endl;
  standardPoseOutputCF.testTransformJac();

  LWFM::StandardKinOutputCF<LWFM::Kin2Update<mtFilterState,anymal_model::AnymalModel>> standardKinOutputCF(&mKin2Update);
  std::cout << "Test StandardKinOutputCF" << std::endl;
  standardKinOutputCF.testTransformJac();

  LWFM::OdometryToBaseOutputCT<LWFM::Kin2Update<mtFilterState,anymal_model::AnymalModel>> odometryToBaseOutputCT(mKin2Update);
  std::cout << "Test OdometryToBaseOutputCT" << std::endl;
  odometryToBaseOutputCT.testTransformJac();

  LWFM::RelPoseOutputCF<LWFM::DeevioUpdate<mtFilterState>> relPoseOutputCF(mDeevioUpdate);
  std::cout << "Test RelPoseOutputCF" << std::endl;
  relPoseOutputCF.testTransformJac(1e-8,1e-5);

  LWFM::KeyframeOutputCF<mtState> keyframeOutputCF;
  std::cout << "Test KeyframeOutputCF" << std::endl;
  keyframeOutputCF.testTransformJac(1e-8,1e-6);

  LWFM::ExtrinsicsOutputCF<mtState> extrinsicsOutputCF;
  std::cout << "Test ExtrinsicsOutputCF" << std::endl;
  extrinsicsOutputCF.testTransformJac();

  LWFM::DeevioOutputCF<LWFM::DeevioUpdate<mtFilterState>> deevioOutputCF(mDeevioUpdate);
  std::cout << "Test DeevioOutputCF" << std::endl;
  deevioOutputCF.testTransformJac();

  // Tests with dynamics only if dynamic model available
  Eigen::MatrixXd M;
  if(model->getMassInertiaMatrix(M)){
    // IMU and Dynamic Prediction with footpoint estimation
    typedef LWFM::ImuAndDyn::Filter<4,true,anymal_model::AnymalModel> mtImuAndDynFilterWithFP;
    mtImuAndDynFilterWithFP imuAndDynFilterWithFP;
    typedef LWFM::ImuAndDyn::Prediction<4,true,anymal_model::AnymalModel,18> mtImuAndDynPredictionWithFP;
    mtImuAndDynPredictionWithFP imuAndDynPredictionWithFP;
    imuAndDynPredictionWithFP.setKinModel(model);
    mtImuAndDynPredictionWithFP::mtMeas imuAndDynMeasWithFP_;
    imuAndDynMeasWithFP_.setRandom(s);
    imuAndDynMeasWithFP_.contactState_[0] = LWFM::ImuAndDyn::CLOSED;
    imuAndDynMeasWithFP_.contactState_[3] = LWFM::ImuAndDyn::CLOSED;
    mtImuAndDynPredictionWithFP::mtState imuAndDynPreviousStateWithFP_;
    imuAndDynPreviousStateWithFP_.setRandom(s);
    imuAndDynPreviousStateWithFP_.template get<mtImuAndDynPredictionWithFP::mtState::_aux>().contactState_[0] = LWFM::ImuAndDyn::CLOSED;
    imuAndDynPreviousStateWithFP_.template get<mtImuAndDynPredictionWithFP::mtState::_aux>().contactState_[3] = LWFM::ImuAndDyn::CLOSED;
    imuAndDynPreviousStateWithFP_.template get<mtImuAndDynPredictionWithFP::mtState::_aux>().contactCount_[0] = 10;
    imuAndDynPreviousStateWithFP_.template get<mtImuAndDynPredictionWithFP::mtState::_aux>().contactCount_[3] = 10;
    imuAndDynPreviousStateWithFP_.template get<mtImuAndDynPredictionWithFP::mtState::_aux>().meas_ = imuAndDynMeasWithFP_;
    imuAndDynMeasWithFP_.setRandom(s);
    mtImuAndDynPredictionWithFP::mtState imuAndDynCurrentStateWithFP_;
    imuAndDynCurrentStateWithFP_.setRandom(s);
    std::cout << "Test IMU and Dyn Prediction" << std::endl;
    imuAndDynPredictionWithFP.testPredictionJacs(imuAndDynPreviousStateWithFP_,imuAndDynCurrentStateWithFP_,imuAndDynMeasWithFP_,1e-6,1e-5,0.1);

    // IMU and Dynamic Prediction without footpoint estimation
    typedef LWFM::ImuAndDyn::Filter<4,false,anymal_model::AnymalModel> mtImuAndDynFilterWithoutFP;
    mtImuAndDynFilterWithoutFP imuAndDynFilterWithoutFP;
    typedef LWFM::ImuAndDyn::Prediction<4,false,anymal_model::AnymalModel,18> mtImuAndDynPredictionWithoutFP;
    mtImuAndDynPredictionWithoutFP imuAndDynPredictionWithoutFP;
    imuAndDynPredictionWithoutFP.setKinModel(model);
    mtImuAndDynPredictionWithoutFP::mtMeas imuAndDynMeasWithoutFP_;
    imuAndDynMeasWithoutFP_.setRandom(s);
    imuAndDynMeasWithoutFP_.contactState_[0] = LWFM::ImuAndDyn::CLOSED;
    imuAndDynMeasWithoutFP_.contactState_[3] = LWFM::ImuAndDyn::CLOSED;
    mtImuAndDynPredictionWithoutFP::mtState imuAndDynPreviousStateWithoutFP_;
    imuAndDynPreviousStateWithoutFP_.setRandom(s);
    imuAndDynPreviousStateWithoutFP_.template get<mtImuAndDynPredictionWithoutFP::mtState::_aux>().contactState_[0] = LWFM::ImuAndDyn::CLOSED;
    imuAndDynPreviousStateWithoutFP_.template get<mtImuAndDynPredictionWithoutFP::mtState::_aux>().contactState_[3] = LWFM::ImuAndDyn::CLOSED;
    imuAndDynPreviousStateWithoutFP_.template get<mtImuAndDynPredictionWithoutFP::mtState::_aux>().contactCount_[0] = 10;
    imuAndDynPreviousStateWithoutFP_.template get<mtImuAndDynPredictionWithoutFP::mtState::_aux>().contactCount_[3] = 10;
    imuAndDynPreviousStateWithoutFP_.template get<mtImuAndDynPredictionWithoutFP::mtState::_aux>().meas_ = imuAndDynMeasWithoutFP_;
    imuAndDynMeasWithoutFP_.setRandom(s);
    mtImuAndDynPredictionWithoutFP::mtState imuAndDynCurrentStateWithoutFP_;
    imuAndDynCurrentStateWithoutFP_.setRandom(s);
    std::cout << "Test IMU and Dyn Prediction Without FP" << std::endl;
    imuAndDynPredictionWithoutFP.testPredictionJacs(imuAndDynPreviousStateWithoutFP_,imuAndDynCurrentStateWithoutFP_,imuAndDynMeasWithoutFP_,1e-6,1e-5,0.1);
  }

  delete model;

  return 0;
}
