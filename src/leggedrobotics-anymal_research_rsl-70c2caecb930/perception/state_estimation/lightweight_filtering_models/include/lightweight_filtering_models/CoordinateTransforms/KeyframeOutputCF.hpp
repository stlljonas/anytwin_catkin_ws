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

#ifndef KEYFRAMEOUTPUTCF_HPP_
#define KEYFRAMEOUTPUTCF_HPP_

#include "lightweight_filtering/CoordinateTransform.hpp"

namespace LWFM {

class KeyframeOutput: public LWF::State<LWF::VectorElement<3>,LWF::QuaternionElement>{
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr unsigned int _pos = 0;
  static constexpr unsigned int _att = _pos+1;
  KeyframeOutput(){
  }
  ~KeyframeOutput(){};
};
template<typename STATE>
class KeyframeOutputCF:public LWF::CoordinateTransform<STATE,KeyframeOutput>{
 public:
  typedef LWF::CoordinateTransform<STATE,KeyframeOutput> Base;
  using Base::eval;
  typedef typename Base::mtInput mtInput;
  typedef typename Base::mtOutput mtOutput;
  KeyframeOutputCF(){};
  ~KeyframeOutputCF(){};
  void evalTransform(mtOutput& output, const mtInput& input) const{
    // IrIC = qIC*CrIC
    // qIC = qIC
    output.template get<mtOutput::_pos>() = input.template get<mtInput::_cla>(0).rotate(input.template get<mtInput::_clp>(0));
    output.template get<mtOutput::_att>() = input.template get<mtInput::_cla>(0);
  }
  void jacTransform(Eigen::MatrixXd& J, const mtInput& input) const{
    J.setZero();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_clp>(0)) =
        rot::RotationMatrixPD(input.template get<mtInput::_cla>(0)).matrix();
    J.template block<3,3>(mtOutput::template getId<mtOutput::_pos>(),mtInput::template getId<mtInput::_cla>(0)) =
        -gSM(input.template get<mtInput::_cla>(0).rotate(input.template get<mtInput::_clp>(0)));
    J.template block<3,3>(mtOutput::template getId<mtOutput::_att>(),mtInput::template getId<mtInput::_cla>(0)) = Eigen::Matrix3d::Identity();
  }
};

}


#endif /* KEYFRAMEOUTPUTCF_HPP_ */
