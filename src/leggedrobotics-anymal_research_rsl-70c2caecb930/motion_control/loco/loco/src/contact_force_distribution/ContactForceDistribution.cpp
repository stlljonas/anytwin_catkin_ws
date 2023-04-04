/*!
 * @file     ContactForceDistribution.cpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Aug 6, 2013 / Jan 17, 2017
 * @brief
 */

// loco
#include "loco/contact_force_distribution/ContactForceDistribution.hpp"
#include "tinyxml_tools/tinyxml_tools.hpp"

// numopt
#include "numopt_common/ParameterizationIdentity.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace loco {

ContactForceDistribution::ContactForceDistribution(WholeBody& wholeBody, TerrainModelBase& terrain,
                                                   std::unique_ptr<numopt_common::QuadraticProblemSolver>&& minimizer)
    : ContactForceDistributionBase(wholeBody, terrain),
      s_(),
      virtualForceWeights_(),
      minimizer_(std::forward<std::unique_ptr<numopt_common::QuadraticProblemSolver> >(minimizer)),
      costFunction_(std::make_shared<numopt_common::QuadraticObjectiveFunction>(numopt_common::QuadraticObjectiveFunction())),
      functionConstraints_(std::make_shared<numopt_common::LinearFunctionConstraints>(numopt_common::LinearFunctionConstraints())),
      quadraticProblem_(costFunction_, functionConstraints_),
      cost_(0.0) {}

bool ContactForceDistribution::resetOptimization() {
  // Reset flag
  isForceDistributionComputed_ = false;

  // Reset jacobians, target/min/max values
  functionConstraints_->setGlobalEqualityConstraintJacobian(numopt_common::SparseMatrix());
  functionConstraints_->setGlobalInequalityConstraintJacobian(numopt_common::SparseMatrix());
  functionConstraints_->setGlobalBoundConstraintMinValues(numopt_common::Vector());
  functionConstraints_->setGlobalBoundConstraintMaxValues(numopt_common::Vector());
  functionConstraints_->setEqualityConstraintTargetValues(numopt_common::Vector());
  functionConstraints_->setInequalityConstraintMinValues(numopt_common::Vector());
  functionConstraints_->setInequalityConstraintMaxValues(numopt_common::Vector());

  return true;
}

bool ContactForceDistribution::prepareOptimization(const Force& virtualForce, const Torque& virtualTorque) {
  //! s_ allows prioratizing a cartesian quantity
  s_ = virtualForceWeights_.asDiagonal();

  return ContactForceDistributionBase::prepareOptimization(virtualForce, virtualTorque);
}

bool ContactForceDistribution::setupConstraint(const ConstraintInterfacePtr& constraint) {
  functionConstraints_->append(*constraint);
  return true;
}

bool ContactForceDistribution::solveOptimization() {
  //-- Setup cost function
#if EIGEN_VERSION_AT_LEAST(3, 2, 92)
  costFunction_->setGlobalHessian(a_.transpose() * s_ * a_ + Eigen::SparseMatrix<double, Eigen::RowMajor>(w_.toDenseMatrix().sparseView()));
#else
  costFunction_->setGlobalHessian(a_.transpose() * s_ * a_ + w_.toDenseMatrix().sparseView());
#endif

  costFunction_->setLinearTerm(-a_.transpose() * s_ * b_);

  //-- Solve qp problem
  numopt_common::ParameterizationIdentity params(x_.size());
  params.getParams() = x_;
  if (!minimizer_->minimize(&quadraticProblem_, params, cost_)) {
    return false;
  }
  x_ = params.getParams();

  return !(x_.array() == 0.0).all();
}

bool ContactForceDistribution::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle forceDistributionHandle = tinyxml_tools::getChildHandle(handle, "ContactForceDistribution");
  TiXmlHandle weightsHandle = tinyxml_tools::getChildHandle(forceDistributionHandle, "Weights");
  TiXmlHandle tempHandle = handle;

  if (tinyxml_tools::getChildHandle(tempHandle, weightsHandle, "Force")) {
    // Virtual force weights
    tinyxml_tools::loadParameter(virtualForceWeights_(0), tempHandle, "heading", 1.0);
    tinyxml_tools::loadParameter(virtualForceWeights_(1), tempHandle, "lateral", 1.0);
    tinyxml_tools::loadParameter(virtualForceWeights_(2), tempHandle, "vertical", 1.0);
  }

  if (tinyxml_tools::getChildHandle(tempHandle, weightsHandle, "Torque")) {
    // Virtual torque weights
    tinyxml_tools::loadParameter(virtualForceWeights_(3), tempHandle, "roll", 1.0);
    tinyxml_tools::loadParameter(virtualForceWeights_(4), tempHandle, "pitch", 1.0);
    tinyxml_tools::loadParameter(virtualForceWeights_(5), tempHandle, "yaw", 1.0);
  }

  return ContactForceDistributionBase::loadParameters(handle);
}

void ContactForceDistribution::print(std::ostream& out) const {
  ContactForceDistributionBase::print(out);
  out << "Loco: CFD: S:\n" << Eigen::MatrixXd(s_.toDenseMatrix()).format(print::CleanFmt);
  out << "Loco: CFD: C:\n" << Eigen::MatrixXd(functionConstraints_->getGlobalEqualityConstraintJacobian()).format(print::CleanFmt);
  out << "Loco: CFD: c:\n" << Eigen::MatrixXd(functionConstraints_->getEqualityConstraintTargetValues()).format(print::CleanFmt);
  out << "Loco: CFD: D:\n" << Eigen::MatrixXd(functionConstraints_->getGlobalInequalityConstraintJacobian()).format(print::CleanFmt);
  out << "Loco: CFD: d:\n" << Eigen::MatrixXd(functionConstraints_->getInequalityConstraintMinValues()).format(print::CleanFmt);
  out << "Loco: CFD: f:\n" << Eigen::MatrixXd(functionConstraints_->getInequalityConstraintMaxValues()).format(print::CleanFmt);
}

} /* namespace loco */
