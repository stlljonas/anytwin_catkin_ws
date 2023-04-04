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

#ifndef IMUANDDYNFILTER_HPP_
#define IMUANDDYNFILTER_HPP_

#include "lightweight_filtering/FilterBase.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"
#include "lightweight_filtering_models/Predictions/ImuAndDynPrediction.hpp"
#include "lightweight_filtering_models/CoordinateTransforms/StandardOutput.hpp"

namespace LWFM {

namespace ImuAndDyn {

template<unsigned int nFeet, bool estFP, typename CommonModel>
class BodyOutputCT:public LWF::CoordinateTransform<State<nFeet,estFP>,StandardOutput>{
 public:
  typedef LWF::CoordinateTransform<State<nFeet,estFP>,StandardOutput> Base;
  using Base::eval;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  BodyOutputCT(const CommonModel* mpCommonModel): mpCommonModel_(mpCommonModel){};
  ~BodyOutputCT(){};
  const CommonModel* mpCommonModel_;
  void evalTransform(mtOutput& output, const mtInput& input) const{
    output.template get<mtOutput::_pos>() = input.template get<mtInput::_pos>() - input.template get<mtInput::_att>().rotate(mpCommonModel_->qMB_.rotate(mpCommonModel_->BrBM_));
    output.template get<mtOutput::_ror>() = mpCommonModel_->qMB_.inverseRotate(input.template get<mtInput::_ror>());
    output.template get<mtOutput::_vel>() = -mpCommonModel_->qMB_.inverseRotate(input.template get<mtInput::_vel>())
                                            - gSM(output.template get<mtOutput::_ror>())*mpCommonModel_->BrBM_;
    output.template get<mtOutput::_att>() = mpCommonModel_->qMB_.inverted()*input.template get<mtInput::_att>().inverted();
  }
  void jacTransform(Eigen::MatrixXd& J, const mtInput& input) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) = Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_att>()) =
        gSM(input.template get<mtInput::_att>().rotate(mpCommonModel_->qMB_.rotate(mpCommonModel_->BrBM_)));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_att>()) =
        -MPD(mpCommonModel_->qMB_.inverted()*input.template get<mtInput::_att>().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vel>()) = -rot::RotationMatrixPD(mpCommonModel_->qMB_.inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_ror>()) =
        gSM(mpCommonModel_->BrBM_)*rot::RotationMatrixPD(mpCommonModel_->qMB_.inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_ror>()) = Eigen::Matrix3d::Identity();
  }
};

template<unsigned int nFeet, bool estFP, typename KinModel>
class Filter:public LWF::FilterBase<LWFM::ImuAndDyn::Prediction<nFeet,estFP,KinModel,nFeet*3+6>>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::FilterBase<LWFM::ImuAndDyn::Prediction<nFeet,estFP,KinModel,nFeet*3+6>> Base;
  using Base::init_;
  using Base::mPrediction_;
  using Base::reset;
  typedef typename Base::mtPrediction::mtMeas mtPredictionMeas;
  typedef typename Base::mtState mtState;
  Filter(){
    reset(0.0);
    mPrediction_.doubleRegister_.removeScalarByStr("alpha");
    mPrediction_.doubleRegister_.removeScalarByStr("beta");
    mPrediction_.doubleRegister_.removeScalarByStr("kappa");
  }
  ~Filter(){};
  void resetWithMeas(const mtPredictionMeas& meas, double t = 0.0){
    init_.initWithMeas(meas);
//    init_.state_.template get<mtState::_att>() = meas.qBI_.inverted()*mPrediction_.commonModel_.qMB_.inverted();
//    init_.state_.template get<mtState::_pos>() = meas.IrIB_ + meas.qBI_.inverted().rotate(mPrediction_.commonModel_.BrBM_);
    reset(t);
  }
};

}

}


#endif /* IMUANDDYNFILTER_HPP_ */
