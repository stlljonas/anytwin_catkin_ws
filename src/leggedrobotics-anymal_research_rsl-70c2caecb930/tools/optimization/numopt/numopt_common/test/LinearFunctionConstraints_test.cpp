/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Christian Gehring, Stelian Coros
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich nor
 *     Carnegie Mellon University nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
* @file    LinearFunctionConstraints_test.cpp
* @author  Gabriel Hottiger
* @date    Aug 16, 2015
*/


#include <gtest/gtest.h>
#include <numopt_common/numopt_gtest_eigen.hpp>
#include <numopt_common/LinearFunctionConstraints.hpp>

#include <Eigen/Core>
#include <Eigen/SparseCore>


TEST(LinearFunctionContraintsTest, appendLinearFunctionConstraint)
{
  using namespace numopt_common;

  constexpr unsigned int r = 3;
  constexpr unsigned int c = 3;
  constexpr unsigned int r_a = 2;
  constexpr unsigned int r_b = r-r_a;

  LinearFunctionConstraints a, b;

  // Eq
  Eigen::Matrix<double, r, 1> eqTargetValues = Eigen::Matrix<double, r, 1>::Random();
  Eigen::Matrix<double, r, c> eqJacobian  = Eigen::Matrix<double, r, c>::Random();

  Eigen::Matrix<double, r_a, 1> eqTargetValuesA = eqTargetValues.segment<r_a>(0);
  Eigen::Matrix<double, r_a, c> eqJacobianA  = eqJacobian.block<r_a,c>(0,0);

  Eigen::Matrix<double, r_b, 1> eqTargetValuesB = eqTargetValues.segment<r_b>(r_a);
  Eigen::Matrix<double, r_b, c> eqJacobianB  = eqJacobian.block<r_b,c>(r_a,0);

  // Set constraint to A
  a.setEqualityConstraintTargetValues(eqTargetValuesA);
  a.setGlobalEqualityConstraintJacobian(eqJacobianA.sparseView());

  // Set constraint to B
  b.setEqualityConstraintTargetValues(eqTargetValuesB);
  b.setGlobalEqualityConstraintJacobian(eqJacobianB.sparseView());

  // Ineq
  Eigen::Matrix<double, r, 1> ineqMinValues = Eigen::Matrix<double, r, 1>::Random();
  Eigen::Matrix<double, r, 1> ineqMaxValues = Eigen::Matrix<double, r, 1>::Random();
  Eigen::Matrix<double, r, c> ineqJacobian  = Eigen::Matrix<double, r, c>::Random();

  Eigen::Matrix<double, r_a, 1> ineqMinValuesA = ineqMinValues.segment<r_a>(0);
  Eigen::Matrix<double, r_a, 1> ineqMaxValuesA = ineqMaxValues.segment<r_a>(0);
  Eigen::Matrix<double, r_a, c> ineqJacobianA  = ineqJacobian.block<r_a,c>(0,0);

  Eigen::Matrix<double, r_b, 1> ineqMinValuesB = ineqMinValues.segment<r_b>(r_a);
  Eigen::Matrix<double, r_b, 1> ineqMaxValuesB = ineqMaxValues.segment<r_b>(r_a);
  Eigen::Matrix<double, r_b, c> ineqJacobianB  = ineqJacobian.block<r_b,c>(r_a,0);

  // Set constraint to A
  a.setInequalityConstraintMinValues(ineqMinValuesA);
  a.setInequalityConstraintMaxValues(ineqMaxValuesA);
  a.setGlobalInequalityConstraintJacobian(ineqJacobianA.sparseView());

  // Set constraint to B
  b.setInequalityConstraintMinValues(ineqMinValuesB);
  b.setInequalityConstraintMaxValues(ineqMaxValuesB);
  b.setGlobalInequalityConstraintJacobian(ineqJacobianB.sparseView());

  // Bound
  Eigen::Matrix<double, r, 1> boundMinValues = Eigen::Matrix<double, r, 1>::Random();
  Eigen::Matrix<double, r, 1> boundMaxValues = Eigen::Matrix<double, r, 1>::Random();

  Eigen::Matrix<double, r_a, 1> boundMinValuesA = boundMinValues.segment<r_a>(0);
  Eigen::Matrix<double, r_a, 1> boundMaxValuesA = boundMaxValues.segment<r_a>(0);

  Eigen::Matrix<double, r_b, 1> boundMinValuesB = boundMinValues.segment<r_b>(r_a);
  Eigen::Matrix<double, r_b, 1> boundMaxValuesB = boundMaxValues.segment<r_b>(r_a);

  // Set constraint to A
  a.setGlobalBoundConstraintMinValues(boundMinValuesA);
  a.setGlobalBoundConstraintMaxValues(boundMaxValuesA);

  // Set constraint to B
  b.setGlobalBoundConstraintMinValues(boundMinValuesB);
  b.setGlobalBoundConstraintMaxValues(boundMaxValuesB);

  // Append b to a
  a.append(b);

  // Test
  {
    Eigen::SparseMatrix<double> appendedEqJacobian = a.getGlobalEqualityConstraintJacobian();
    Eigen::SparseMatrix<double> referenceEqJacobian = eqJacobian.sparseView();
    Eigen::Matrix<double, r, 1> appendedEqTargetValues = a.getEqualityConstraintTargetValues();
    NUMOPT_ASSERT_DOUBLE_MX_EQ(appendedEqTargetValues, eqTargetValues, 1e-20, "equality constraints target values");
    NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(appendedEqJacobian, referenceEqJacobian, 1e-20, "equality constraints jacobian");
  }
  {
    Eigen::SparseMatrix<double> appendedIneqJacobian = a.getGlobalInequalityConstraintJacobian();
    Eigen::SparseMatrix<double> referenceIneqJacobian = ineqJacobian.sparseView();
    Eigen::Matrix<double, r, 1> appendedIneqMinValues = a.getInequalityConstraintMinValues();
    Eigen::Matrix<double, r, 1> appendedIneqMaxValues = a.getInequalityConstraintMaxValues();
    NUMOPT_ASSERT_DOUBLE_MX_EQ(appendedIneqMinValues, ineqMinValues, 1e-20, "inequality constraints min values");
    NUMOPT_ASSERT_DOUBLE_MX_EQ(appendedIneqMaxValues, ineqMaxValues, 1e-20, "inequality constraints max values");
    NUMOPT_ASSERT_DOUBLE_SPARSE_MX_EQ(appendedIneqJacobian, referenceIneqJacobian, 1e-20, "inequality constraints jacobian");
  }
  {
    Eigen::Matrix<double, r, 1> appendedBoundMinValues = a.getGlobalBoundConstraintMinValues();
    Eigen::Matrix<double, r, 1> appendedBoundMaxValues = a.getGlobalBoundConstraintMaxValues();
    NUMOPT_ASSERT_DOUBLE_MX_EQ(appendedBoundMinValues, boundMinValues, 1e-20, "bound constraints min values");
    NUMOPT_ASSERT_DOUBLE_MX_EQ(appendedBoundMaxValues, boundMaxValues, 1e-20, "bound constraints max values");
  }
}
