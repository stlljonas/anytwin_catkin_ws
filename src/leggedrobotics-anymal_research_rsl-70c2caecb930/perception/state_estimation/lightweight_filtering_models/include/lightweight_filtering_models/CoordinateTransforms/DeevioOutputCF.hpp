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

#ifndef DEEVIOOUTPUTCF_HPP_
#define DEEVIOOUTPUTCF_HPP_

#include "lightweight_filtering_models/CoordinateTransforms/StandardOutput.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"

namespace LWFM {

template<typename DeevioUpdate>
class DeevioOutputCF:public LWF::CoordinateTransform<typename DeevioUpdate::mtState,StandardOutput>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::CoordinateTransform<typename DeevioUpdate::mtState,StandardOutput> Base;
  using Base::eval;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  DeevioOutputCF(const DeevioUpdate& deevioUpdate): mpDeevioUpdate_(&deevioUpdate){};
  ~DeevioOutputCF(){};
  const DeevioUpdate* mpDeevioUpdate_;
  void evalTransform(mtOutput& output, const mtInput& input) const{
    // IrIV = qIM*(MrIM + MrMV)
    // VvV = qVM*(MvM + MwIM x MrMV)
    // qVI = qVM*qIM^T
    // VwIV = qVM*MwIM
    V3D MrMV = mpDeevioUpdate_->MrMV_;
    QPD qVM = mpDeevioUpdate_->qVM_;
    if(mtInput::calVE_){
      MrMV = input.template get<mtInput::_vep>();
      qVM = input.template get<mtInput::_vea>();
    }
    output.template get<mtOutput::_pos>() = input.template get<mtInput::_att>().rotate(V3D(input.template get<mtInput::_pos>()+MrMV));
    output.template get<mtOutput::_ror>() = qVM.rotate(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()));
    output.template get<mtOutput::_vel>() =
        qVM.rotate(V3D(-input.template get<mtInput::_vel>() + gSM(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()))*MrMV));
    output.template get<mtOutput::_att>() = qVM*input.template get<mtInput::_att>().inverted();
  }
  void jacTransform(Eigen::MatrixXd& J, const mtInput& input) const{
    J.setZero();
    V3D MrMV = mpDeevioUpdate_->MrMV_;
    QPD qVM = mpDeevioUpdate_->qVM_;
    if(mtInput::calVE_){
      MrMV = input.template get<mtInput::_vep>();
      qVM = input.template get<mtInput::_vea>();
    }
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) =
        MPD(input.template get<mtInput::_att>()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_att>()) =
        -gSM(input.template get<mtInput::_att>().rotate(V3D(input.template get<mtInput::_pos>()+MrMV)));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_att>()) =
        -MPD(qVM*input.template get<mtInput::_att>().inverted()).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vel>()) = -MPD(qVM).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_gyb>()) = MPD(qVM).matrix()
        * gSM(MrMV);
    J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_gyb>()) = -MPD(qVM).matrix();
    if(mtInput::calVE_){
      J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_vep>()) =
          MPD(input.template get<mtInput::_att>()).matrix();
      J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vep>()) =
          MPD(qVM).matrix()*gSM(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_vea>()) =
          -gSM(qVM.rotate(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>())));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vea>()) =
          -gSM(qVM.rotate(V3D(-input.template get<mtInput::_vel>() + gSM(V3D(input.template get<mtInput::_aux>().MwIMmeas_-input.template get<mtInput::_gyb>()))*MrMV)));
      J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_vea>()) =
          M3D::Identity();
    }
  }
  void postProcess(Eigen::MatrixXd& cov,const mtInput& input){
    cov.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtOutput::template getId<mtOutput::_ror>()) += input.template get<mtInput::_aux>().wMeasCov_;
  }
};

}


#endif /* DEEVIOOUTPUTCF_HPP_ */
