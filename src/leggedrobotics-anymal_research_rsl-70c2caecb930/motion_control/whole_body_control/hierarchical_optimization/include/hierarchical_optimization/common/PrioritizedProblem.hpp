/*
 * PrioritizedProblem.hpp
 *
 *  Created on: Jan 27, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

// hierarchical optimization
#include <hierarchical_optimization/common/hierarchical_optimization.hpp>

// std
#include <string>


namespace hopt {

enum class ConstraintType : unsigned int {
  Equality = 0,
  Inequality,
  Mixed
};

/*
 * A task T is defined a:
 *
 *
 *      [ W(Ax - b)  = 0
 *  T : [
 *      [ V(Dx - d) <= 0
 */
class PrioritizedTask {
 public:
  using JacobianMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;

  template<typename NameType_, typename PriorityType_>
  PrioritizedTask(NameType_&& name, PriorityType_&& priority)
  : name_(std::forward<NameType_>(name)),
    priority_(std::forward<PriorityType_>(priority))
  {

  }

  template<typename EqJacobianMatrixType_,
           typename EqTargetValuesType_,
           typename NameType_,
           typename PriorityType_,
           typename IneqJacobianMatrixType_,
           typename IneqTargetValuesType_,
           typename EqWeightsType_,
           typename IneqWeightsType_,
           typename ConstraintType_
  >
  PrioritizedTask(NameType_&& name, PriorityType_&& priority,
                  EqJacobianMatrixType_&& equalityConstraintJacobian,
                  EqTargetValuesType_&& equalityConstraintTargetValues,
                  IneqJacobianMatrixType_&& inequalityConstraintJacobian,
                  IneqTargetValuesType_&& inequalityConstraintMaxValues,
                  EqWeightsType_&& equalityConstraintRelativeWeights,
                  IneqWeightsType_&& inequalityConstraintRelativeWeights,
                  ConstraintType_&& constraintType)
      : name_(std::forward<NameType_>(name)),
        priority_(std::forward<PriorityType_>(priority)),
        equalityConstraintJacobian_(std::forward<EqJacobianMatrixType_>(equalityConstraintJacobian)),
        equalityConstraintTargetValues_(std::forward<EqTargetValuesType_>(equalityConstraintTargetValues)),
        inequalityConstraintJacobian_(std::forward<IneqJacobianMatrixType_>(inequalityConstraintJacobian)),
        inequalityConstraintMaxValues_(std::forward<IneqTargetValuesType_>(inequalityConstraintMaxValues)),
        equalityConstraintRelativeWeights_(std::forward<EqWeightsType_>(equalityConstraintRelativeWeights)),
        inequalityConstraintRelativeWeights_(std::forward<IneqWeightsType_>(inequalityConstraintRelativeWeights)),
        constraintType_(std::forward<ConstraintType_>(constraintType))
  {
    static_assert(std::is_integral<typename std::remove_reference<PriorityType_>::type>::value, "[PrioritizedTask] PriorityType should be an integral type.");
    checkConsistency();
  }

  template<typename EqJacobianMatrixType_,
           typename EqTargetValuesType_,
           typename NameType_,
           typename PriorityType_,
           typename IneqJacobianMatrixType_,
           typename IneqTargetValuesType_,
           typename ConstraintType_
  >
  PrioritizedTask(NameType_&& name, PriorityType_&& priority,
                  EqJacobianMatrixType_&& equalityConstraintJacobian,
                  EqTargetValuesType_&& equalityConstraintTargetValues,
                  IneqJacobianMatrixType_&& inequalityConstraintJacobian,
                  IneqTargetValuesType_&& inequalityConstraintMaxValues,
                  ConstraintType_&& constraintType)
      : name_(std::forward<NameType_>(name)),
        priority_(std::forward<PriorityType_>(priority)),
        equalityConstraintJacobian_(std::forward<EqJacobianMatrixType_>(equalityConstraintJacobian)),
        equalityConstraintTargetValues_(std::forward<EqTargetValuesType_>(equalityConstraintTargetValues)),
        inequalityConstraintJacobian_(std::forward<IneqJacobianMatrixType_>(inequalityConstraintJacobian)),
        inequalityConstraintMaxValues_(std::forward<IneqTargetValuesType_>(inequalityConstraintMaxValues)),
        equalityConstraintRelativeWeights_(Eigen::VectorXd::Ones(equalityConstraintJacobian_.rows())),
        inequalityConstraintRelativeWeights_(Eigen::VectorXd::Ones(inequalityConstraintJacobian_.rows())),
        constraintType_(std::forward<ConstraintType_>(constraintType))
  {
    static_assert(std::is_integral<typename std::remove_reference<PriorityType_>::type>::value, "[PrioritizedTask] PriorityType should be an integral type.");
    checkConsistency();
  }

  PrioritizedTask(PrioritizedTask&& rhs) = default;
  PrioritizedTask& operator=(PrioritizedTask&& rhs) = default;

  PrioritizedTask(const PrioritizedTask& rhs) = default;
  PrioritizedTask& operator=(const PrioritizedTask& rhs) = default;

  virtual ~PrioritizedTask() = default;

  template<typename T>
  void setName(T&& name) {
    name_ = std::forward<T>(name);
  }

  template<typename T>
  void setPriority(T&& priority) {
    priority_ = std::forward<T>(priority);
  }

  template<typename T>
  void setEqualityConstraintJacobian(T&& equalityConstraintJacobian) {
    equalityConstraintJacobian_ = std::forward<T>(equalityConstraintJacobian);
  }

  template<typename T>
  void setEqualityConstraintTargetValues(T&& equalityConstraintTargetValues) {
    equalityConstraintTargetValues_ = std::forward<T>(equalityConstraintTargetValues);
  }

  template<typename T>
  void setInequalityConstraintJacobian(T&& inequalityConstraintJacobian) {
    inequalityConstraintJacobian_ = std::forward<T>(inequalityConstraintJacobian);
  }

  template<typename T>
  void setInequalityConstraintMaxValues(T&& inequalityConstraintMaxValues) {
    inequalityConstraintMaxValues_ = std::forward<T>(inequalityConstraintMaxValues);
  }

  template<typename T>
  void setEqualityConstraintRelativeWeights(T&& equalityConstraintRelativeWeights) {
    equalityConstraintRelativeWeights_ = std::forward<T>(equalityConstraintRelativeWeights);
  }

  template<typename T>
  void setInequalityConstraintRelativeWeights(T&& inequalityConstraintRelativeWeights) {
    inequalityConstraintRelativeWeights_ = std::forward<T>(inequalityConstraintRelativeWeights);
  }

  template<typename T, typename = typename std::enable_if<std::is_same<T, ConstraintType>::value>::type>
  void setConstraintType(T&& constraintType) {
    constraintType_ = std::forward<T>(constraintType);
  }

  const std::string& getName() const { return name_; }
  int getPriority() const { return priority_; }
  const Eigen::MatrixXd& getEqualityConstraintJacobian() const { return equalityConstraintJacobian_; }
  const Eigen::VectorXd& getEqualityConstraintTargetValues() const { return equalityConstraintTargetValues_; }
  const Eigen::MatrixXd& getInequalityConstraintJacobian() const { return inequalityConstraintJacobian_; }
  const Eigen::VectorXd& getInequalityConstraintMaxValues() const { return inequalityConstraintMaxValues_; }
  const Eigen::VectorXd& getEqualityConstraintRelativeWeights() const { return equalityConstraintRelativeWeights_; }
  const Eigen::VectorXd& getInequalityConstraintRelativeWeights() const { return inequalityConstraintRelativeWeights_; }
  const ConstraintType& getConstraintType() const { return constraintType_; }

  friend std::ostream& operator<< (std::ostream& stream, const PrioritizedTask& prioritizedTask) {
    stream << "Name: " << prioritizedTask.getName() << std::endl
           << "priority: " << prioritizedTask.getPriority() << std::endl
           << "A: " << prioritizedTask.getEqualityConstraintJacobian() << std::endl
           << "b: " << prioritizedTask.getEqualityConstraintTargetValues() << std::endl
           << "C: " << prioritizedTask.getInequalityConstraintJacobian() << std::endl
           << "d: " << prioritizedTask.getInequalityConstraintMaxValues() << std::endl;
    return stream;
  }

 protected:
  //! Check task consistency.
  void checkConsistency() {
    assert(equalityConstraintJacobian_.rows() == equalityConstraintTargetValues_.rows());
    assert(equalityConstraintJacobian_.rows() == equalityConstraintRelativeWeights_.rows());
    assert(inequalityConstraintJacobian_.rows() == inequalityConstraintMaxValues_.rows());
    assert(inequalityConstraintJacobian_.rows() == inequalityConstraintRelativeWeights_.rows());

    assert(equalityConstraintTargetValues_.cols() == 1);
    assert(equalityConstraintRelativeWeights_.cols() == 1);
    assert(inequalityConstraintMaxValues_.cols() == 1);
    assert(inequalityConstraintRelativeWeights_.cols() == 1);
  }

  //! A string identifier of this task.
  std::string name_;

  //! The priority associated to this task.
  int priority_;

  //! The unweighed jacobian matrix of the equality constraints.
  Eigen::MatrixXd equalityConstraintJacobian_;

  //! The unweighed target values of the equality constraints.
  Eigen::VectorXd equalityConstraintTargetValues_;

  //! The unweighed jacobian matrix of the inequality constraints.
  Eigen::MatrixXd inequalityConstraintJacobian_;

  //! The unweighed jacobian matrix of the equality constraints.
  Eigen::VectorXd inequalityConstraintMaxValues_;

  //! The equality constraints relative weights.
  Eigen::VectorXd equalityConstraintRelativeWeights_;

  //! The inequality constraints relative weights.
  Eigen::VectorXd inequalityConstraintRelativeWeights_;

  //! An identifier which specifies the type of task.
  ConstraintType constraintType_;
};

} /* namespace hopt */
