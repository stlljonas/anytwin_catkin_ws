/*
 * QPReader.hpp
 *
 *  Created on: Dez 21, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// zmp optimizer
#include <zmp_optimizer/zmp_optimizer.hpp>
#include "zmp_optimizer/ZmpOptimizerDynamicWalk.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Sparse>

// numerical optimization
#include <numopt_common/ParameterizationIdentity.hpp>
#include <numopt_common/numopt_common.hpp>
#include <numopt_quadprog/ActiveSetFunctionMinimizer.hpp>
#include "numopt_sqp/SQPFunctionMinimizer.hpp"

namespace loco {

bool readQPProblem(const std::string& path, const std::string& problem, const std::string& gait_name) {
  bool success = true;

  // Timer to measure elapsed time for optimization.
  std_utils::HighResolutionClockTimer timer;
  double totalElapsedTime = 0.0;
  std::vector<double> elapsedTime;

  // Optimization problem ans QP solver.
  std::unique_ptr<numopt_common::QuadraticProblem> optimizationProblemQP_;
  std::shared_ptr<numopt_common::QuadraticProblemSolver> qpSolver_;
  numopt_common::ParameterizationIdentity solutionVector;

  optimizationProblemQP_.reset(new numopt_common::QuadraticProblem(
      std::shared_ptr<numopt_common::QuadraticObjectiveFunction>(new numopt_common::QuadraticObjectiveFunction),
      std::shared_ptr<numopt_common::LinearFunctionConstraints>(new numopt_common::LinearFunctionConstraints)));

  qpSolver_.reset(new numopt_quadprog::ActiveSetFunctionMinimizer());

  // QP matrices.
  Eigen::MatrixXd costFunctionHessian_;
  Eigen::VectorXd costFunctionLinearTerm_;
  Eigen::MatrixXd equalityConstraintJacobian_;
  Eigen::VectorXd equalityConstraintTargetValues_;
  Eigen::MatrixXd inequalityConstraintJacobian_;
  Eigen::VectorXd inequalityConstraintMinValues_;
  Eigen::VectorXd inequalityConstraintMaxValues_;
  Eigen::VectorXd globalMinValues_;
  Eigen::VectorXd globalMaxValues_;
  Eigen::VectorXd solution;

  Eigen::MatrixXd Mat;
  Eigen::VectorXd Vec;
  double cost;

  // Path to file that stores the QP matrices.
  const std::string filename = path + problem + "_problem_" + gait_name + "_success.txt";
  std::ifstream file(filename);
  std::string line;

  if (!file.is_open()) {
    std::cout << "failed to open file " << filename << std::endl;
    return false;
  }

  // Matrix dimensions.
  unsigned int row = 0u;
  unsigned int col = 0u;
  unsigned int rows = 0u;
  unsigned int cols = 0u;
  unsigned int sqp_iter = 0u;

  std::string name;

  while (std::getline(file, line)) {
    // Extract line and reset.
    char* line_ptr = (char*)line.c_str();

    /*
     * Check if
     *  1) a new matrix starts, i.e. the line is
     *        [matrix_name] rows cols
     *  2) a new SQP iteration start, i.e., the line is
     *        [sqp_iteration] iter
     *  3) the end of an interation is reached, i.e., the line is
     *        [end_of_iter]
     *  4) the solution is read, i.e., the line is
     *        [solution]
     */
    if (line_ptr[0] == '[') {
      // Extract name.
      unsigned int delimiter = line.find(']');
      name = line.substr(1u, delimiter - 1u);

      // Case 2) Start a new QP formulation.
      if (name == "sqp_iteration") {
        sqp_iter = atof(line_ptr + delimiter + 2u);
        continue;
      }

      // Case 3) Solve QP problem.
      else if (name == "end_of_iter") {
        timer.pinTime();

        optimizationProblemQP_->setOptimizationMatrices(costFunctionHessian_.sparseView(), costFunctionLinearTerm_,
                                                        equalityConstraintJacobian_.sparseView(),
                                                        inequalityConstraintJacobian_.sparseView(), equalityConstraintTargetValues_,
                                                        inequalityConstraintMinValues_, globalMinValues_, globalMaxValues_);

        solutionVector.getParams().setZero(costFunctionLinearTerm_.size());
        success &= qpSolver_->minimize(optimizationProblemQP_.get(), solutionVector, cost);
        totalElapsedTime += timer.getElapsedTimeMsec();
        elapsedTime.push_back(timer.getElapsedTimeMsec());

        continue;
      }

      // Case 4)
      else if (name == "solution") {
        rows = solutionVector.getParams().size();
        cols = 1u;
        Vec.resize(rows);
        row = 0u;
        continue;
      }

      // Case 1) Get information about the matrix dimensions.
      delimiter += 2u;  // skip white space
      rows = (unsigned int)std::atof(line_ptr + delimiter);
      cols = (unsigned int)std::atof(line_ptr + line.find(' ', delimiter) + 1u);

      if (cols > 1u) {
        Mat.resize(rows, cols);
      } else {
        Vec.resize(rows);
      }
      row = 0u;

      continue;
    }

    // Iterate through the line.
    col = 0u;
    for (unsigned int charId = 0u; charId < line.length(); ++charId) {
      // Skip white spaces.
      while (charId < line.length() && line_ptr[charId] == ' ') {
        ++charId;
      }

      if (cols == 1u) {
        Vec(row) = std::atof(line_ptr + charId);
        break;
      }

      // Read between two white spaces.
      else if (charId < line.length() && row < rows && col < cols) {
        Mat(row, col) = std::atof(line_ptr + charId);

        // Skip number.
        while (charId < line.length() && line_ptr[charId] != ' ') {
          ++charId;
        }
        ++col;
      }
    }

    ++row;

    // Allocate matrix
    if (row == rows) {
      if (name == "hessian") {
        costFunctionHessian_ = Mat;
      } else if (name == "linear_term") {
        costFunctionLinearTerm_ = Vec;
      } else if (name == "eq_jacobian") {
        equalityConstraintJacobian_ = Mat;
      } else if (name == "eq_target") {
        equalityConstraintTargetValues_ = Vec;
      } else if (name == "ineq_jacobian") {
        inequalityConstraintJacobian_ = Mat;
      } else if (name == "ineq_min") {
        inequalityConstraintMinValues_ = Vec;
      } else if (name == "ineq_max") {
        inequalityConstraintMaxValues_ = Vec;
      } else if (name == "min_bound") {
        globalMinValues_ = Vec;
      } else if (name == "max_bound") {
        globalMaxValues_ = Vec;
      } else if (name == "solution") {
        solution = Vec;
      } else {
        success = false;
      }
    }
  }

  file.close();

  // Output.
  if (problem == "QP") {
    std::cout << "problem type = QP, gait = " << gait_name << ", elapsed time = " << totalElapsedTime << " ms.\n";
  }

  else if (problem == "SQP") {
    std::cout << "problem type = SQP, num of sqp iterations = " << sqp_iter + 1 << ", gait = " << gait_name
              << ", elapsed time = " << totalElapsedTime << " ms ( ";
    for (unsigned int id = 0u; id < elapsedTime.size(); ++id) {
      std::cout << "iter " << id << ": " << elapsedTime[id] / totalElapsedTime * 100.0 << "%";
      if (id < elapsedTime.size() - 1) {
        std::cout << ", ";
      } else {
        std::cout << " ";
      }
    }
    std::cout << ").\n";
  }

  if (name == "solution" && !solution.isApprox(solutionVector.getParams(), 0.01)) {
    std::cout << "solution are different\n";
    return false;
  }

  return success;
}

} /* namespace loco */
