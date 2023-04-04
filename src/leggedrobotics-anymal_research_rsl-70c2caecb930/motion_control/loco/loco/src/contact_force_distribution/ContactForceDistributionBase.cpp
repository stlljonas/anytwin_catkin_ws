/*!
 * @file     ContactForceDistribution.cpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Aug 6, 2013 / Jan 17, 2017
 * @brief
 */

// loco
#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// tinyxml_tools
#include "tinyxml_tools/tinyxml_tools.hpp"

namespace loco {

ContactForceDistributionBase::ContactForceDistributionBase(WholeBody& wholeBody, TerrainModelBase& terrain)
    : ContactForceDistributionInterface(),
      wholeBody_(wholeBody),
      terrain_(terrain),
      limbInfos_(),
      n_(0),
      numberOfFailures_(0),
      numberOfToleratedConsecutiveFailures_(10),
      isForceDistributionComputed_(),
      x_(),
      a_(),
      b_(),
      constraints_(),
      distributedVirtualForceInBaseFrame_(),
      distributedVirtualTorqueInBaseFrame_() {
  // Add limb infos
  for (auto limb : wholeBody.getLimbs()) {
    limbInfos_[limb] = LimbInfo();
  }
}

bool ContactForceDistributionBase::addConstraint(std::unique_ptr<ConstraintInterface>&& constraint) {
  constraints_.emplace_back(std::move(constraint));
}

bool ContactForceDistributionBase::computeForceDistribution(const Force& virtualForceInBaseFrame, const Torque& virtualTorqueInBaseFrame) {
  // Reset algorithm specifics
  resetOptimization();

  // Check which limbs are part of force distribution
  updateLimbInfos(wholeBody_.getLimbs());

  if (limbInfos_.getNumLimbsInForceDistribution() > 0) {
    prepareOptimization(virtualForceInBaseFrame, virtualTorqueInBaseFrame);

    // Set constraints to current optimization problem
    setupConstraints();

    // compute the solution of the optimization
    isForceDistributionComputed_ = solveOptimization();

    // set computed forces to containers
    setDesiredEndeffectorForces();

    if (!isForceDistributionComputed_) {
      MELO_WARN_STREAM(*this);
    }
  } else {
    // No leg is part of the force distribution
    isForceDistributionComputed_ = true;
  }

  // for logging, compute resulting net force and torque
  getNetForceAndTorqueOnBase(distributedVirtualForceInBaseFrame_, distributedVirtualTorqueInBaseFrame_);

  // Tolerate numberOfToleratedConsecutiveFailures_ failures before emergency stop.
  numberOfFailures_ = isForceDistributionComputed_ ? 0 : numberOfFailures_ + 1;
  return numberOfFailures_ <= numberOfToleratedConsecutiveFailures_;
}

bool ContactForceDistributionBase::getNetForceAndTorqueOnBase(Force& netForce, Torque& netTorque) {
  if (!isForceDistributionComputed_ || limbInfos_.getNumLimbsInForceDistribution() == 0) {
    return false;
  }

  Eigen::Matrix<double, cartesian::wrenchVectorSize, 1> stackedNetForceAndTorque;
  stackedNetForceAndTorque = a_ * x_;
  netForce = Force(stackedNetForceAndTorque.head(3));
  netTorque = Torque(stackedNetForceAndTorque.tail(3));
  return true;
}

bool ContactForceDistributionBase::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(distributedVirtualForceInBaseFrame_, "distVirtualForceInBaseFrame", "/loco/cfd/", "N");
  signal_logger::add(distributedVirtualTorqueInBaseFrame_, "distVirtualTorqueInBaseFrame", "/loco/cfd/", "Nm");
  return ContactForceDistributionInterface::addVariablesToLog(ns);
}

bool ContactForceDistributionBase::updateLimbInfos(const Limbs& limbs) {
  // Reset previous limbs that are no longer part of the force distribution
  for (auto& limbInfo : limbInfos_.get()) {
    limbInfo.isPartOfForceDistribution_ = false;
    limbInfo.isLoadConstraintActive_ = false;
  }

  // Check whether limb contributes to force distribution
  unsigned int i = 0;
  for (auto limb : limbs) {
    if ((limb->getLoadFactor() > 0.0) && ((limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
                                          (limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant))) {
      limbInfos_[limb].isPartOfForceDistribution_ = true;
      limbInfos_[limb].isLoadConstraintActive_ = false;
      limbInfos_[limb].limbIndexInLimbList_ = i++;
      limbInfos_[limb].limbStartIndexInSolutionVector_ = limbInfos_[limb].limbIndexInLimbList_ * cartesian::forceVectorSize;

      if (limb->getLoadFactor() < 1.0) {
        limbInfos_[limb].isLoadConstraintActive_ = true;
      }
    } else {
      limbInfos_[limb].isPartOfForceDistribution_ = false;
      limbInfos_[limb].isLoadConstraintActive_ = false;
    }
  }

  return true;
}

bool ContactForceDistributionBase::prepareOptimization(const Force& virtualForce, const Torque& virtualTorque) {
  n_ = cartesian::forceVectorSize * limbInfos_.getNumLimbsInForceDistribution();

  //! b_ is the desired virtual torso wrench
  b_.resize(cartesian::wrenchVectorSize);
  b_.segment<cartesian::forceVectorSize>(0) = virtualForce.toImplementation();
  b_.segment<cartesian::torqueVectorSize>(cartesian::forceVectorSize) = virtualTorque.toImplementation();

  // The upper part of A_ defines the mapping from contact forces to torso forces (repeated identity)
  // The lower part of A_ defines the mapping from contact forces to torso torques.
  a_.resize(cartesian::wrenchVectorSize, n_);
  a_.topRows(cartesian::forceVectorSize) =
      (Eigen::Matrix3d::Identity().replicate(1, limbInfos_.getNumLimbsInForceDistribution())).sparseView();
  Eigen::MatrixXd A_bottomMatrix(cartesian::torqueVectorSize, n_);
  A_bottomMatrix.setZero();

  // W_ is a regularizer applied to the force minimization problem
  w_.setIdentity(cartesian::forceVectorSize * limbInfos_.getNumLimbsInForceDistribution());

  for (auto limb : wholeBody_.getLimbs()) {
    if (limbInfos_[limb].isPartOfForceDistribution_) {
      const Eigen::Vector3d& r = limb->getEndEffector().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame().toImplementation();
      A_bottomMatrix.block<cartesian::torqueVectorSize, cartesian::torqueVectorSize>(0, limbInfos_[limb].limbStartIndexInSolutionVector_) =
          kindr::getSkewMatrixFromVector(r);
      w_.diagonal().segment<cartesian::forceVectorSize>(limbInfos_[limb].limbStartIndexInSolutionVector_) =
          Eigen::Vector3d::Constant(limbInfos_[limb].groundForceWeight_);
    }
  }
  a_.bottomRows(cartesian::torqueVectorSize) = A_bottomMatrix.sparseView();

  // x_ is the solution vector of contact forces
  x_.resize(n_);

  return true;
}

bool ContactForceDistributionBase::setupConstraints() {
  bool success = true;
  for (auto& constraint : constraints_) {
    success = constraint->updateConstraint(limbInfos_) && success;
    success = setupConstraint(constraint) && success;
  }
  return success;
}

bool ContactForceDistributionBase::setDesiredEndeffectorForces() {
  for (auto limb : *wholeBody_.getLimbsPtr()) {
    if (limbInfos_[limb].isPartOfForceDistribution_) {
      // The forces we computed here are actually the ground reaction forces.
      Force desiredContactForceInBaseFrame =
          Force(Eigen::Vector3d(x_.segment<cartesian::forceVectorSize>(limbInfos_[limb].limbStartIndexInSolutionVector_)));
      limb->getEndEffectorPtr()->getStateDesiredPtr()->setForceAtEndEffectorInWorldFrame(
          wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase().inverseRotate(desiredContactForceInBaseFrame));
    } else {
      limb->getEndEffectorPtr()->getStateDesiredPtr()->setForceAtEndEffectorInWorldFrame(Force::Zero());
    }
  }
}

bool ContactForceDistributionBase::loadParameters(const TiXmlHandle& handle) {
  for (auto& constraint : constraints_) {
    constraint->loadParameters(handle);
  }

  TiXmlHandle weightsHandle = tinyxml_tools::getChildHandle(tinyxml_tools::getChildHandle(handle, "ContactForceDistribution"), "Weights");
  TiXmlHandle tempHandle = handle;

  double groundForceWeight;
  if (tinyxml_tools::getChildHandle(tempHandle, weightsHandle, "Regularizer")) {
    tinyxml_tools::loadParameter(groundForceWeight, tempHandle, "value", 1.0);
    for (auto limb : wholeBody_.getLimbs()) {
      double limbGroundForceWeight;
      if (tempHandle.Element()->QueryValueAttribute(limb->getName(), &limbGroundForceWeight) == TIXML_SUCCESS) {
        limbInfos_[limb].groundForceWeight_ = limbGroundForceWeight;
      } else {
        limbInfos_[limb].groundForceWeight_ = groundForceWeight;
      }
    }
  }

  return ContactForceDistributionInterface::loadParameters(handle);
}

bool ContactForceDistributionBase::setToInterpolated(const ContactForceDistributionInterface& contactForceDistribution1,
                                                     const ContactForceDistributionInterface& contactForceDistribution2, double t) {
  auto cfd1 = dynamic_cast<const ContactForceDistributionBase*>(&contactForceDistribution1);
  auto cfd2 = dynamic_cast<const ContactForceDistributionBase*>(&contactForceDistribution2);

  for (const auto& constraint : constraints_) {
    auto it1 = std::find_if(cfd1->constraints_.begin(), cfd1->constraints_.end(), [&constraint](const ConstraintInterfacePtr& c) {
      return constraint->getConstraintName() == c->getConstraintName();
    });
    auto it2 = std::find_if(cfd2->constraints_.begin(), cfd2->constraints_.end(), [&constraint](const ConstraintInterfacePtr& c) {
      return constraint->getConstraintName() == c->getConstraintName();
    });
    if (it1 != cfd1->constraints_.end() && it2 != cfd2->constraints_.end()) {
      constraint->setToInterpolated(*(*it1), *(*it2), t);
    } else if (it1 != cfd1->constraints_.end()) {
      constraint->setToInterpolated(*(*it1), *(*it1), 0.0);
    } else if (it2 != cfd2->constraints_.end()) {
      constraint->setToInterpolated(*(*it2), *(*it2), 0.0);
    }
  }

  return true;
}

void ContactForceDistributionBase::print(std::ostream& out) const {
  ContactForceDistributionInterface::print(out);
  out << "Constraints: " << std::endl;
  for (const auto& constraint : constraints_) {
    out << *constraint << std::endl;
  }
  out << "Loco: CFD::solveOptimization() returned " << std::boolalpha << isForceDistributionComputed_ << std::endl;
  out << "Loco: CFD:nLimbsInForceDistribution: " << limbInfos_.getNumLimbsInForceDistribution() << std::endl;
  out << "Loco: CFD: A:\n" << Eigen::MatrixXd(a_).format(print::CleanFmt) << std::endl;
  out << "Loco: CFD: b:\n" << Eigen::MatrixXd(b_).format(print::CleanFmt) << std::endl;
  out << "Loco: CFD: x:\n" << Eigen::MatrixXd(x_).format(print::CleanFmt) << std::endl;
  out << "Loco: CFD: W:\n" << Eigen::MatrixXd(w_.toDenseMatrix()).format(print::CleanFmt) << std::endl;
}

} /* namespace loco */
