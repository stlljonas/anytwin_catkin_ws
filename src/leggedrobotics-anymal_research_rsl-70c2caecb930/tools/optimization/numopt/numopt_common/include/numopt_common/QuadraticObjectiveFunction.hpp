/*
 * QuadraticObjectiveFunction.hpp
 *
 *  Created on: Jan 6, 2016
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

#include "numopt_common/NonlinearObjectiveFunction.hpp"

// Printing
#include <iostream>
#include <fstream>
#include <iomanip>

namespace numopt_common {

/*!
 * This quadratic objective function has the form of f(p) = 1/2 p' Q p + c' p,
 * where Q is the symmetric positive semidefinite Hessian matrix.
 *
 */
class QuadraticObjectiveFunction: public NonlinearObjectiveFunction
{
 public:
  explicit QuadraticObjectiveFunction(const numopt_common::SparseMatrix& Q = SparseMatrix(),
                             const Vector& c = Vector());
  ~QuadraticObjectiveFunction() override = default;
  bool computeValue(Scalar& value, const Parameterization& p, bool newParams = true) override;
  virtual void setGlobalHessian(const numopt_common::SparseMatrix& Q);
  virtual void setLinearTerm(const Vector& c);
  virtual bool getLinearTerm(Vector& linearTerm);
  bool getLocalGradient(Vector& gradient, const Parameterization& p, bool newParams = true) override;
  bool getGlobalGradient(Vector& gradient, const Parameterization& p, bool newParams = true) override;
  bool getGlobalHessian(numopt_common::SparseMatrix& hessian, const Parameterization& p, bool newParams = true) override;
  bool getLocalHessian(numopt_common::SparseMatrix& Q, const Parameterization& p, bool newParams) override;
  bool getGlobalHessianTriplets(SMTriplets& triplets, const Parameterization& p, bool newParams = true);

  bool printToFile(std::ofstream& file) const;

 protected:
  void setHessianTripletsFromMatrix(SMTriplets& triplets, numopt_common::SparseMatrix& hessian);

  Vector linearTerm_;
  numopt_common::SparseMatrix globalHessian_;
  SMTriplets globalHessianTriplets_;
};

} /* namespace numopt_common */
