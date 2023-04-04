/*
 * HierarchicalOptimizationBase.hpp
 *
 *  Created on: Oct 27, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

// hierarchical optimization
#include <hierarchical_optimization/common/PrioritizedProblem.hpp>

// stl
#include <deque>
#include <vector>
#include <memory>

namespace hopt {

class HierarchicalOptimizationBase {
 public:
  using TaskList = std::list<PrioritizedTask>;
  using PriorityList = std::vector<int>;

  HierarchicalOptimizationBase();
  explicit HierarchicalOptimizationBase(unsigned int solutionDimension);
  virtual ~HierarchicalOptimizationBase() = default;

  virtual bool resetOptimization();

  //! Add a prioritized task to the list of tasks to solve.
  template<typename... Args>
  bool addOptimizationProblem(Args&&... args) {
    prioritizedTasks_.emplace_back(std::forward<Args>(args)...);
    const auto& task = prioritizedTasks_.back();

    // Check for consistency in the problem definition.
    assert(task.getEqualityConstraintJacobian().rows() == task.getEqualityConstraintTargetValues().rows());
    assert(task.getEqualityConstraintJacobian().rows() == task.getEqualityConstraintRelativeWeights().rows());
    assert(task.getInequalityConstraintJacobian().rows() == task.getInequalityConstraintMaxValues().rows());
    assert(task.getInequalityConstraintJacobian().rows() == task.getInequalityConstraintRelativeWeights().rows());

    priorityVec_.push_back(task.getPriority());
    return true;
  }

  virtual bool solveOptimization(Eigen::VectorXd& solution) = 0;

  void setSolutionDimension(unsigned int solutionDimension);

  //! Util methods.
  const TaskList& getOptimizationProblems() const;
  const TaskList& getStackedProblems() const;

 protected:
  //! Stack two matrices vertically.
  static bool appendMatrix(Eigen::MatrixXd& A, const Eigen::MatrixXd& Anew);

  //! Append a row to a matrix.
  static bool appendRow(Eigen::MatrixXd& A, const Eigen::VectorXd& a);

  //! Stack tasks according to their priority.
  bool buildPrioritizedOptimizationProblem();

  //! Stack all problems with same priority in one task.
  void stackTasksWithPriority(int priority);

  //! A list of prioritized tasks.
  TaskList prioritizedTasks_;

  //! A list of tasks stacked according to their priority.
  TaskList stackedTasks_;

  //! A list of task priorities.
  PriorityList priorityVec_;

  //! The dimension of the solution space.
  unsigned int solutionDimension_;
};

using HierarchicalOptimizationBasePtr = std::unique_ptr<HierarchicalOptimizationBase>;

} /* namespace hopt */