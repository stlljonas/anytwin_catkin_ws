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

#ifndef TRANSFORMSTANDARDPOSECF_HPP_
#define TRANSFORMSTANDARDPOSECF_HPP_

#include "lightweight_filtering_models/CoordinateTransforms/StandardOutput.hpp"
#include "lightweight_filtering/CoordinateTransform.hpp"

namespace LWFM {

class TransformStandardPoseCF:public LWF::CoordinateTransform<StandardOutput,StandardOutput>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef LWF::CoordinateTransform<StandardOutput,StandardOutput> Base;
  using Base::eval;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  TransformStandardPoseCF(){
    qJI_.setIdentity();
    IrIJ_.setZero();
  };
  ~TransformStandardPoseCF(){};
  rot::RotationQuaternionPD qJI_;
  Eigen::Vector3d IrIJ_;
  void evalTransform(mtOutput& output, const mtInput& input) const{
    // JrJB = qJI*(IrIB-IrIJ)
    // BvB = BvB
    // qBJ = qBI*qJI^T
    // BwJB = BwIB
    output.template get<mtOutput::_pos>() = qJI_.rotate(Eigen::Vector3d(input.template get<mtInput::_pos>()-IrIJ_));
    output.template get<mtOutput::_ror>() = input.template get<mtInput::_ror>();
    output.template get<mtOutput::_vel>() = input.template get<mtInput::_vel>();
    output.template get<mtOutput::_att>() = input.template get<mtInput::_att>()*qJI_.inverted();
  }
  void jacTransform(Eigen::MatrixXd& J, const mtInput& input) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_pos>()) = MPD(qJI_).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_att>()) = Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_vel>(),mtInput::template getId<mtInput::_vel>()) = Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_ror>(),mtInput::template getId<mtInput::_ror>()) = Eigen::Matrix3d::Identity();
  }
};

}


#endif /* TRANSFORMSTANDARDPOSECF_HPP_ */
