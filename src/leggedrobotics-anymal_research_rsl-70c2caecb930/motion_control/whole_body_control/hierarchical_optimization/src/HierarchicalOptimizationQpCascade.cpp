/*
 * HierarchicalOptimizationQpCascade.cpp
 *
 *  Created on: Oct 27, 2015
 *      Author: Dario Bellicoso
 */

#include <hierarchical_optimization/HierarchicalOptimizationQpCascade.hpp>
#include <numopt_common/ParameterizationIdentity.hpp>

namespace hopt {

HierarchicalOptimizationQpCascade::HierarchicalOptimizationQpCascade(
    int solutionDimension,
    std::unique_ptr<numopt_common::QuadraticProblemSolver>&& minimizer)
    : HierarchicalOptimizationBase(),
      minimizer_(std::move(minimizer)),
      costFunction_(new numopt_common::QuadraticObjectiveFunction()),
      functionConstraints_(new numopt_common::LinearFunctionConstraints()),
      quadraticProblem_(costFunction_, functionConstraints_)
{
  resetOptimization();
  timer_.setTimerNamespace("[HierarchicalOptimizationQpCascade]");
}

bool HierarchicalOptimizationQpCascade::solveOptimization(Eigen::VectorXd& solution) {

  // stack the optimization problems according to their priority
  buildPrioritizedOptimizationProblem();

  // Set initial unshaped admissible set
  Eigen::MatrixXd Abar, Cbar;
  Eigen::VectorXd bbar, dbar;

  // solution vector
  Eigen::VectorXd xStar;

//  timer_.pinTime("solve_full_opt");

  // solve the hierarchically prioritized problems
  for (const auto& minProb : stackedTasks_) {

    const std::string& minProbName = minProb.getName();

//    std::cout << minProbName << std::endl;

//    timer_.pinTime(minProbName + "_solve_prob");

    const Eigen::MatrixXd& equalityConstraintJacobian = minProb.getEqualityConstraintJacobian();
    const Eigen::VectorXd& equalityConstraintTargetValues = minProb.getEqualityConstraintTargetValues();
    const Eigen::MatrixXd& inequalityConstraintJacobian = minProb.getInequalityConstraintJacobian();
    const Eigen::VectorXd& inequalityConstraintMaxValues = minProb.getInequalityConstraintMaxValues();

    const int minProbEqualityConstraintsCount = equalityConstraintJacobian.rows();
    const int minProbInequalityConstraintsCount = inequalityConstraintJacobian.rows();
    const int stackedInequalityConstraintsCount = Cbar.rows();
    const int qDimension = solutionDimension_ + minProbInequalityConstraintsCount;

    // quadratic terms
//    timer_.pinTime(minProbName + "_quad_terms");
    constexpr double regularizer = 1.0e-5;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(qDimension, qDimension);
    if (minProbEqualityConstraintsCount) {
      Q.topLeftCorner(solutionDimension_, solutionDimension_) = equalityConstraintJacobian.transpose()*equalityConstraintJacobian + regularizer*Eigen::MatrixXd::Identity(solutionDimension_, solutionDimension_);
    }
//    timer_.splitTime(minProbName + "_quad_terms");



    // linear terms
//    timer_.pinTime(minProbName + "_lin_terms");
    Eigen::MatrixXd Ahat = Eigen::MatrixXd::Zero(minProbEqualityConstraintsCount, qDimension);
    Eigen::VectorXd c = Eigen::VectorXd::Zero(qDimension);
    if (minProbEqualityConstraintsCount) {
      Ahat.leftCols(solutionDimension_) = equalityConstraintJacobian;
      c = -(Ahat.transpose()*equalityConstraintTargetValues);
    }
//    timer_.splitTime(minProbName + "_lin_terms");




    // inequality constraints
//    timer_.pinTime(minProbName + "_ineq_constr");
    Eigen::MatrixXd Chat = Eigen::MatrixXd::Zero(Cbar.rows()+minProbInequalityConstraintsCount, qDimension);
    Eigen::VectorXd dhat = Eigen::VectorXd::Zero(Cbar.rows()+minProbInequalityConstraintsCount);
    if (stackedInequalityConstraintsCount) {
      Chat.topLeftCorner(Cbar.rows(), solutionDimension_) = Cbar;
      dhat.topRows(Cbar.rows()) = dbar;
    }
    if (minProbInequalityConstraintsCount) {
      Chat.bottomLeftCorner(minProbInequalityConstraintsCount, solutionDimension_) = inequalityConstraintJacobian;
      Chat.bottomRightCorner(minProbInequalityConstraintsCount, minProbInequalityConstraintsCount) = -Eigen::MatrixXd::Identity(minProbInequalityConstraintsCount, minProbInequalityConstraintsCount);
      dhat.bottomRows(minProbInequalityConstraintsCount) = inequalityConstraintMaxValues;
    }

    Eigen::VectorXd minBounds = -std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(dhat.rows());
    minBounds.bottomRows(minProbInequalityConstraintsCount) = Eigen::VectorXd::Zero(minProbInequalityConstraintsCount);

//    timer_.splitTime(minProbName + "_ineq_constr");

    // equality constraints
//    timer_.pinTime(minProbName + "_eq_constr");
    Eigen::MatrixXd Abarhat = Eigen::MatrixXd::Zero(Abar.rows(), solutionDimension_+minProbInequalityConstraintsCount);
    if (Abar.size() != 0) {
      Abarhat.leftCols(solutionDimension_) = Abar;
    }
//    timer_.splitTime(minProbName + "_eq_constr");




//    timer_.pinTime(minProbName + "_update_min_prob_constr");
    costFunction_->setGlobalHessian(Q.sparseView());
    costFunction_->setLinearTerm(c);

    functionConstraints_->setGlobalEqualityConstraintJacobian(Abarhat.sparseView());
    functionConstraints_->setEqualityConstraintTargetValues(bbar);
    functionConstraints_->setGlobalInequalityConstraintJacobian(Chat.sparseView());
    functionConstraints_->setInequalityConstraintMaxValues(dhat);
    functionConstraints_->setInequalityConstraintMinValues(minBounds);

//    timer_.splitTime(minProbName + "_update_min_prob_constr");
    numopt_common::ParameterizationIdentity xiStar(Q.cols());

    // solve qp problem
//    timer_.pinTime(minProbName + "_minimize");
    try {
      double cost = 0.0;
      minimizer_->minimize(&quadraticProblem_, xiStar, cost);
    } catch (const std::runtime_error& error) {
      std::cout << "caught exception: " << error.what() << std::endl;
    }
//    timer_.splitTime(minProbName + "_minimize");



    // extract x from xi = [x' w']'
    xStar = xiStar.getParams().topRows(solutionDimension_);
    const Eigen::VectorXd wStar = xiStar.getParams().bottomRows(qDimension - solutionDimension_);




//    timer_.pinTime(minProbName + "_update_sol_space");
    // update admissible solution space with current constraints
    if (equalityConstraintJacobian.size() != 0) {
      appendMatrix(Abar, equalityConstraintJacobian);
      bbar.conservativeResize(bbar.rows()+equalityConstraintJacobian.rows());
      bbar.bottomRows(equalityConstraintJacobian.rows()) = equalityConstraintJacobian*xStar;
    }

    Cbar.conservativeResize(Cbar.rows() + minProbInequalityConstraintsCount, solutionDimension_);
    dbar.conservativeResize(dbar.rows() + minProbInequalityConstraintsCount);
    Cbar.bottomRows(minProbInequalityConstraintsCount) = inequalityConstraintJacobian;
    dbar.bottomRows(minProbInequalityConstraintsCount) = inequalityConstraintMaxValues + wStar;

//    for (int h=0; h<inequalityConstraintJacobian.rows(); h++) {
//      const Eigen::VectorXd& ch = inequalityConstraintJacobian.row(h);
//
//      if (ch.transpose()*xStar <= inequalityConstraintMaxValues[h]) {
//        Cbar.conservativeResize(Cbar.rows() + 1, solutionDimension_);
//        Cbar.bottomRows(1) = ch.transpose();
//        dbar.conservativeResize(dbar.size() + 1);
//        dbar(dbar.size()-1, 0) = inequalityConstraintMaxValues[h] + wStar[h];
//      } else {
//        Abar.conservativeResize(Abar.rows() + 1, solutionDimension_);
//        Abar.bottomRows(1) = ch.transpose();
//        bbar.conservativeResize(bbar.size()+1);
//        bbar(bbar.size()-1, 0) = ch.transpose()*xStar;
//      }
//    }
//    timer_.splitTime(minProbName + "_update_sol_space");




//    timer_.splitTime(minProbName + "_solve_prob");
  }

  solution = xStar;

//  timer_.splitTime("solve_full_opt");

//  std::cout << timer_ << std::endl;

  return true;
}


} /* namespace hopt */
