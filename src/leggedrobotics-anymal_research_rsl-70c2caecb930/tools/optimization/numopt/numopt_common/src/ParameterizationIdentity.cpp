/*
 * ParameterizationIdentity.cpp
 *
 *  Created on: Jun 9, 2016
 *      Author: Christian Gehring
 */

#include "numopt_common/ParameterizationIdentity.hpp"

namespace numopt_common {

ParameterizationIdentity::ParameterizationIdentity(int size):
    Parameterization(),
    params_(size) {

}

ParameterizationIdentity::ParameterizationIdentity(const Params& p) :
      Parameterization(),
      params_(p) {
}

bool ParameterizationIdentity::plus(Params& result,
                                    const Params& p,
                                    const Delta& dp) const {
  result = p + dp;
  return true;
}

bool ParameterizationIdentity::getTransformMatrixLocalToGlobal(numopt_common::SparseMatrix& transformation, const Params& p) const {
  transformation.resize(getGlobalSize(), getLocalSize());
  transformation.setIdentity();
  return true;
}

bool ParameterizationIdentity::getTransformMatrixGlobalToLocal(numopt_common::SparseMatrix& transformation, const Params& p) const {
  transformation.resize(getLocalSize(), getGlobalSize());
  transformation.setIdentity();
  return true;
}

bool ParameterizationIdentity::transformLocalVectorToGlobalVector(numopt_common::Vector& globalVector,
                                    const Params& params,
                                    const numopt_common::Vector& localVector) const {
  globalVector = localVector;
  return true;
}


bool ParameterizationIdentity::setRandom(Params& p) const
{
  p.setRandom(params_.size());
  return true;
}

bool ParameterizationIdentity::setIdentity(Params& p) const {
  p.setZero(params_.size());
  return true;
}

} /* namespace numopt_common */
