/*!
 * @file     ContactForceDistribution.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Aug 6, 2013 / Jan 17, 2017
 * @brief
 */

#pragma once

// loco
#include "loco/contact_force_distribution/ContactForceDistributionBase.hpp"
#include "loco/contact_force_distribution/constraints/ConstraintInterface.hpp"

#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"

// numerical optimization
#include "numopt_common/QuadraticProblemSolver.hpp"

// xml parser
#include <tinyxml.h>

// STL
#include <memory>
#include <ostream>

namespace loco {

/*!
 * Based on 'Control of Dynamic Gaits for a Anymalal Robot', C. Gehring, ICRA, 2013.
 */
class ContactForceDistribution : public ContactForceDistributionBase {
 public:
  //! Constructor.
  ContactForceDistribution(WholeBody& wholeBody, TerrainModelBase& terrain,
                           std::unique_ptr<numopt_common::QuadraticProblemSolver>&& minimizer);

  //! Destructor.
  ~ContactForceDistribution() override = default;

 protected:
  /** Resets the optimization (e.g. clears previous constraints)
   *  @return true if successful
   */
  bool resetOptimization() override;

  /*!
   * Prepare matrices for the optimization problem.
   * @return true if successful
   */
  bool prepareOptimization(const Force& virtualForce, const Torque& virtualTorque) override;

  /** Adds a constraint to the optimization.
   *  @param constraint, Linear function constraint that shall be added
   *  @return true if successful
   */
  bool setupConstraint(const ConstraintInterfacePtr& constraint) override;

  /** Solves the optimization and sets the desired contact forces
   *  @return true if successful
   */
  bool solveOptimization() override;

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  bool loadParameters(const TiXmlHandle& handle) override;

 protected:
  /** Allows printing over cout (ContactForceDistributionBase implements operator<< )
   *  @param stream that will be printed by cout
   */
  void print(std::ostream& out) const override;

 protected:
  //! Weighting matrix for the desired virtual forces and torques.
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> s_;
  //! Diagonal elements of the weighting matrix for the desired virtual forces and torques (for S).
  Eigen::Matrix<double, cartesian::wrenchVectorSize, 1> virtualForceWeights_;
  //! Solver for the quadratic program
  std::unique_ptr<numopt_common::QuadraticProblemSolver> minimizer_;
  //! Quadratic cost function
  std::shared_ptr<numopt_common::QuadraticObjectiveFunction> costFunction_;
  //! Linear function constraints
  std::shared_ptr<numopt_common::LinearFunctionConstraints> functionConstraints_;
  //! Quadratic problem formulation
  numopt_common::QuadraticProblem quadraticProblem_;
  //! Cost of the optimization
  double cost_;
};

} /* namespace loco */
