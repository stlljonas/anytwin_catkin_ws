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

#ifndef STANDARDPOSEOUTPUTCF_HPP_
#define STANDARDPOSEOUTPUTCF_HPP_

#include "lightweight_filtering_models/CoordinateTransforms/StandardOutput.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"

namespace LWFM {

template<typename PoseUpdate>
class StandardPoseOutputCF:public LWF::CoordinateTransform<typename PoseUpdate::mtState,StandardOutput>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::CoordinateTransform<typename PoseUpdate::mtState,StandardOutput> Base;
  using Base::evalTransform;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  StandardPoseOutputCF(const PoseUpdate& poseUpdate): mpPoseUpdate_(&poseUpdate){};
  ~StandardPoseOutputCF(){};
  const PoseUpdate* mpPoseUpdate_;
  void evalTransform(mtOutput& output, const mtInput& input) const{
    // JrJV = qIJ^T*qIM*(MrIM + MrMV) + JrJI
    // VvV = qVM*(MvM + MwIM x MrMV)
    // qVJ = qVM*qIM^T*qIJ
    // VwJV = qVM*MwIM
    output.template get<mtOutput::_pos>() = input.template get<mtInput::_pip>() + (input.template get<mtInput::_pia>().inverted()*input.template get<mtInput::_att>()).rotate(
        V3D(input.template get<mtInput::_pos>()+mpPoseUpdate_->MrMV_));
    output.template get<mtOutput::_ror>() = mpPoseUpdate_->qVM_.rotate(Eigen::Vector3d(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()));
    output.template get<mtOutput::_vel>() = mpPoseUpdate_->qVM_.rotate(Eigen::Vector3d(-input.template get<mtInput::_vel>() + kindr::getSkewMatrixFromVector(
        Eigen::Vector3d(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()))*mpPoseUpdate_->MrMV_));
    output.template get<mtOutput::_att>() = mpPoseUpdate_->qVM_*input.template get<mtInput::_att>().inverted()*input.template get<mtInput::_pia>();
  }
  void jacTransform(Eigen::MatrixXd& J, const mtInput& input) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) =
        MPD(input.template get<mtInput::_pia>().inverted()*input.template get<mtInput::_att>()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pip>()) =
              M3D::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_att>()) =
        MPD(input.template get<mtInput::_pia>().inverted()).matrix()*
        -gSM(input.template get<mtInput::_att>().rotate(V3D(input.template get<mtInput::_pos>()+mpPoseUpdate_->MrMV_)));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pia>()) =
        gSM((input.template get<mtInput::_pia>().inverted()*input.template get<mtInput::_att>()).rotate(
            V3D(input.template get<mtInput::_pos>()+mpPoseUpdate_->MrMV_)))*MPD(input.template get<mtInput::_pia>().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_att>()) =
        -MPD(mpPoseUpdate_->qVM_*input.template get<mtInput::_att>().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_pia>()) =
        MPD(mpPoseUpdate_->qVM_*input.template get<mtInput::_att>().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vel>()) = -rot::RotationMatrixPD(mpPoseUpdate_->qVM_).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_gyb>()) = rot::RotationMatrixPD(mpPoseUpdate_->qVM_).matrix()
        * kindr::getSkewMatrixFromVector(mpPoseUpdate_->MrMV_);
    J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_gyb>()) = -rot::RotationMatrixPD(mpPoseUpdate_->qVM_).matrix();
  }
  void postProcess(Eigen::MatrixXd& cov,const mtInput& input){
    cov.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtOutput::template getId<mtOutput::_ror>()) += input.template get<mtInput::_aux>().wMeasCov_;
  }
};

}


#endif /* STANDARDPOSEOUTPUTCF_HPP_ */
