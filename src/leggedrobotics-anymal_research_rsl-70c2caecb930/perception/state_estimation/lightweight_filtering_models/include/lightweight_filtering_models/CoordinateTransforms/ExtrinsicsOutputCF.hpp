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

#ifndef EXTRINSICSOUTPUTCF_HPP_
#define EXTRINSICSOUTPUTCF_HPP_

#include "lightweight_filtering/CoordinateTransform.hpp"

namespace LWFM {

class ExtrinsicsOutput: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  ExtrinsicsOutput(){
  }
  ~ExtrinsicsOutput(){};
};
template<typename STATE>
class ExtrinsicsOutputCF:public LWF::CoordinateTransform<STATE,ExtrinsicsOutput>{
 public:
  typedef LWF::CoordinateTransform<STATE,ExtrinsicsOutput> Base;
  using Base::eval;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  ExtrinsicsOutputCF(){};
  ~ExtrinsicsOutputCF(){};
  void evalTransform(mtOutput& output, const mtInput& input) const{
    // MrMV = MrMV
    // qVM = qVM
    output.template get<mtOutput::_pos>() = input.template get<mtInput::_vep>();
    output.template get<mtOutput::_att>() = input.template get<mtInput::_vea>();
  }
  void jacTransform(Eigen::MatrixXd& J, const mtInput& input) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_vep>()) = Eigen::Matrix3d::Identity();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_vea>()) = Eigen::Matrix3d::Identity();
  }
};

}


#endif /* EXTRINSICSOUTPUTCF_HPP_ */
