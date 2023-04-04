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

#ifndef RELPOSEOUTPUTCF_HPP_
#define RELPOSEOUTPUTCF_HPP_

#include "lightweight_filtering/CoordinateTransform.hpp"

namespace LWFM {

class RelPoseOutput: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  RelPoseOutput(){
  }
  ~RelPoseOutput(){};
};
template<typename DeevioUpdate>
class RelPoseOutputCF:public LWF::CoordinateTransform<typename DeevioUpdate::mtState,RelPoseOutput>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::CoordinateTransform<typename DeevioUpdate::mtState,RelPoseOutput> Base;
  using Base::eval;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  RelPoseOutputCF(const DeevioUpdate& poseUpdate): mpDeevioUpdate_(&poseUpdate){};
  ~RelPoseOutputCF(){};
  const DeevioUpdate* mpDeevioUpdate_;
  void evalTransform(mtOutput& output, const mtInput& input) const{
    Eigen::Vector3d MrMV = mpDeevioUpdate_->MrMV_;
    rot::RotationQuaternionPD qVM = mpDeevioUpdate_->qVM_;
    if(mtInput::calVE_){
      MrMV = input.template get<mtInput::_vep>();
      qVM = input.template get<mtInput::_vea>();
    }
    // WrWV = qWN*(qIN^T*qIM*(MrIM+MrMV) - NrIN - NrNW)
    // qVW = qVM*qIM^T*qIN*qWN^T
    // NrNW = MrMV, qVM = qWN
    output.template get<mtOutput::_pos>() = qVM.rotate(Eigen::Vector3d((input.template get<mtInput::_cla>(0).inverted()
        *input.template get<mtInput::_att>()).rotate(Eigen::Vector3d(input.template get<mtInput::_pos>()+MrMV))-input.template get<mtInput::_clp>(0)-MrMV));
    output.template get<mtOutput::_att>() = qVM*input.template get<mtInput::_att>().inverted()*input.template get<mtInput::_cla>(0)*qVM.inverted();
  }
  void jacTransform(Eigen::MatrixXd& J, const mtInput& input) const{
    Eigen::Vector3d MrMV = mpDeevioUpdate_->MrMV_;
    rot::RotationQuaternionPD qVM = mpDeevioUpdate_->qVM_;
    if(mtInput::calVE_){
      MrMV = input.template get<mtInput::_vep>();
      qVM = input.template get<mtInput::_vea>();
    }
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_cla>(0)) = -rot::RotationMatrixPD(qVM).matrix()
            *-gSM((input.template get<mtInput::_cla>(0).inverted()
            *input.template get<mtInput::_att>()).rotate(Eigen::Vector3d(input.template get<mtInput::_pos>()+MrMV)))
            *rot::RotationMatrixPD(input.template get<mtInput::_cla>(0).inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_att>()) = rot::RotationMatrixPD(qVM*input.template get<mtInput::_cla>(0).inverted()).matrix()
            *-gSM(input.template get<mtInput::_att>().rotate(Eigen::Vector3d(input.template get<mtInput::_pos>()+MrMV)));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) =
        rot::RotationMatrixPD(qVM*input.template get<mtInput::_cla>(0).inverted()*input.template get<mtInput::_att>()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_clp>(0)) = -rot::RotationMatrixPD(qVM).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_att>()) =
        -rot::RotationMatrixPD(qVM*input.template get<mtInput::_att>().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_cla>(0)) =
        rot::RotationMatrixPD(qVM*input.template get<mtInput::_att>().inverted()).matrix();
    if(mtInput::calVE_){
      J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_vep>()) = -rot::RotationMatrixPD(qVM).matrix()
          + rot::RotationMatrixPD(qVM*input.template get<mtInput::_cla>(0).inverted()*input.template get<mtInput::_att>()).matrix();
      J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_vea>()) =
          -gSM(qVM.rotate(Eigen::Vector3d((input.template get<mtInput::_cla>(0).inverted()
              *input.template get<mtInput::_att>()).rotate(Eigen::Vector3d(input.template get<mtInput::_pos>()+MrMV))-input.template get<mtInput::_clp>(0)-MrMV)));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_vea>()) = Eigen::Matrix3d::Identity()
          -rot::RotationMatrixPD(qVM*input.template get<mtInput::_att>().inverted()*input.template get<mtInput::_cla>(0)*qVM.inverted()).matrix();
    }
  }
};

}


#endif /* RELPOSEOUTPUTCF_HPP_ */
