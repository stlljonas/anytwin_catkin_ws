/*!
 * @file     ContactForceDistribution.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Aug 6, 2013 / Jan 17, 2017
 * @brief
 */

#pragma once

// loco
#include "loco/contact_force_distribution/ContactForceDistributionInterface.hpp"
#include "loco/contact_force_distribution/LimbInfos.hpp"
#include "loco/contact_force_distribution/constraints/ConstraintInterface.hpp"

#include "loco/common/WholeBody.hpp"
#include "loco/common/limbs/Limbs.hpp"

// eigen
#include <Eigen/SparseCore>

// system
#include <mutex>

namespace loco {

//! This class distributes a virtual force and torque on the base as forces to the leg contact points.
/*!
 * The optimization problem is formulated as:
 *
 * [ I    I   ...] [f1] = [F] ==> A*x = b
 * [r1x  r2x  ...] [f2]   [T]
 *                 [ .]
 *                 [ .]
 *                 [ .]
 */
class ContactForceDistributionBase : public ContactForceDistributionInterface {
 public:
  //! Constructor.
  ContactForceDistributionBase(WholeBody& wholeBody, TerrainModelBase& terrain);

  //! Destructor.
  ~ContactForceDistributionBase() override = default;

  /**
   * @brief Add a constraint to the force distribution
   * @param constraint Constraint to be added
   * @return true if successfully added constraint
   */
  bool addConstraint(ConstraintInterfacePtr&& constraint) override;

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force on the base (in base frame).
   * @param virtualTorque the desired virtual torque on the base (in base frame).
   * @return true if successful.
   */
  bool computeForceDistribution(const Force& virtualForceInBaseFrame, const Torque& virtualTorqueInBaseFrame) override;

  /*!
   * Gets the distributed net forces and torques that act on the base, i.e.
   * this force and torque are computed from the distributed contact forces and
   * should ideally be equal to the desired net forces and torques.
   * @param[out] netForce.
   * @param[out] netTorque.
   * @return true if net force and torque can be calculated, false otherwise.
   */
  bool getNetForceAndTorqueOnBase(Force& netForce, Torque& netTorque) override;

  /*!
   * Adds class data to the logger (optional).
   * @return true if successful.
   */
  bool addVariablesToLog(const std::string& ns) const override;

  //! @return number of limbs that are part of the force distribution
  const unsigned int getNumberOfLimbsInForceDistribution() const { return limbInfos_.getNumLimbsInForceDistribution(); }

  //! @param numberOfToleratedConsecutiveFailures Number of consecutive failures that lead to returning false
  void setNumberOfToleratedConsecutiveFailures(const unsigned int numberOfToleratedConsecutiveFailures) {
    numberOfToleratedConsecutiveFailures_ = numberOfToleratedConsecutiveFailures;
  }

 protected:
  /** Resets the optimization (e.g. clears previous constraints)
   *  @return true if successful
   */
  virtual bool resetOptimization() = 0;

  /*!
   * Reads foot contact flags and includes user leg load settings from changeLegLoad().
   * @return true if successful.
   */
  virtual bool updateLimbInfos(const Limbs& limbs);

  /*!
   * Prepare matrices for the optimization problem.
   * @return true if successful
   */
  virtual bool prepareOptimization(const Force& virtualForce, const Torque& virtualTorque);

  /** Calls setupConstraint for all constraints in the optimization problem
   *  @return true if all constraint were setup properly
   */
  virtual bool setupConstraints();

  /** Adds a constraint to the optimization.
   *  @param constraint, Linear function constraint that shall be added
   *  @return true if successful
   */
  virtual bool setupConstraint(const ConstraintInterfacePtr& constraint) = 0;

  /** Solves the optimization and sets the desired contact forces
   *  @return true if successful
   */
  virtual bool solveOptimization() = 0;

  /** Sets the optimization solution to the limbs
   *  @return true if successful
   */
  virtual bool setDesiredEndeffectorForces();

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Computes an interpolated version of the two force distributions passed in as parameters.
   *  if t is 0, the current setting is set to contactForceDistribution1,
   *  1 -> contactForceDistribution2, and values in between
   *  correspond to interpolated parameter set.
   * @param contactForceDistribution1
   * @param contactForceDistribution2
   * @param t interpolation parameter
   * @returns true if successful
   */
  bool setToInterpolated(const ContactForceDistributionInterface& contactForceDistribution1,
                         const ContactForceDistributionInterface& contactForceDistribution2, double t) override;

 protected:
  /** Allows printing over cout (ContactForceDistributionBase implements operator<< )
   *  @param stream that will be printed by cout
   */
  void print(std::ostream& out) const override;

 protected:
  //! Wholebody container to access limbs and torso
  WholeBody& wholeBody_;

  //! Terrain model, for friction cones etc
  TerrainModelBase& terrain_;

  //! Container for additional limb information (Contains all limbs that were part of force distribution once)
  LimbInfos limbInfos_;

  //-- Optimization problem setup

  //! Number of variables to optimize (size of x, n = nTranslationalDofPerFoot_ * nLegsInStance_)
  int n_;
  //! Count number of failures of the optimization problem
  unsigned int numberOfFailures_;
  unsigned int numberOfToleratedConsecutiveFailures_;

  //!
  bool isForceDistributionComputed_;

  //! Stacked contact forces (in base frame)
  Eigen::VectorXd x_;
  //! The matrix A in the optimization formulation (nElementsInStackedVirtualForceTorqueVector_ x n).
  Eigen::SparseMatrix<double, Eigen::RowMajor> a_;
  //! The vector b in the optimization formulation (stacked vector of desired net virtual forces and torques).
  Eigen::VectorXd b_;
  //! Weighting matrix for the ground reaction forces (regularizer).
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> w_;
  //! Vector of constraints that act on the optimization
  std::vector<ConstraintInterfacePtr> constraints_;

  //-- Logging

  //! Resulting virtual net force (acting on torso) expressed in base frame from distributed contact forces (for logging)
  Force distributedVirtualForceInBaseFrame_;
  //! Resulting virtual net torque (acting on torso) expressed in base frame from distributed contact forces (for logging)
  Torque distributedVirtualTorqueInBaseFrame_;
};

} /* namespace loco */
