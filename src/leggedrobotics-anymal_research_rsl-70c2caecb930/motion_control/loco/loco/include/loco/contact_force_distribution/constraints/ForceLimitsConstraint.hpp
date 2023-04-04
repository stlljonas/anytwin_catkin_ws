/*
 * ForceLimitsConstraint.hpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"
#include "loco/contact_force_distribution/constraints/ConstraintInterface.hpp"

// robot_utils
#include "robot_utils/math/math.hpp"

// STL
#include <map>

namespace loco {

//! Force limit container for min/max ground contact force
struct ForceLimit {
  //! Default constructor
  ForceLimit() = default;

  /** Constructor from min / max force
   * @param minForce, minimal ground contact force
   * @param maxForce, maximal ground contact force
   * @param maxForceLoadConstrained, maximal ground contact force load constrained
   * @param maxIncrement, maximal ground contact force increment
   */
  ForceLimit(const double minForce, const double maxForce, const double maxForceLoadConstrained = std::numeric_limits<double>::max(),
             const double maxIncrement = std::numeric_limits<double>::max())
      : minForce_(minForce), maxForce_(maxForce), maxForceLoadConstrained_(maxForceLoadConstrained), maxIncrement_(maxIncrement) {}

  bool setToInterpolated(const ForceLimit& limit1, const ForceLimit& limit2, double t) {
    minForce_ = robot_utils::linearlyInterpolate(limit1.minForce_, limit2.minForce_, 0.0, 1.0, t);
    maxForce_ = robot_utils::linearlyInterpolate(limit1.maxForce_, limit2.maxForce_, 0.0, 1.0, t);
    maxForceLoadConstrained_ =
        robot_utils::linearlyInterpolate(limit1.maxForceLoadConstrained_, limit2.maxForceLoadConstrained_, 0.0, 1.0, t);
    maxIncrement_ = robot_utils::linearlyInterpolate(limit1.maxIncrement_, limit2.maxIncrement_, 0.0, 1.0, t);
    return true;
  }

  friend std::ostream& operator<<(std::ostream& out, const ForceLimit& limit) {
    out << "Minimal Force: " << limit.minForce_ << std::endl;
    out << "Maximal Force: " << limit.maxForce_ << std::endl;
    out << "Maximal Force (Load Constraint): " << limit.maxForceLoadConstrained_ << std::endl;
    out << "Maximal Force Increment: " << limit.maxIncrement_ << std::endl;
    return out;
  }

  //! Minimal ground contact force
  double minForce_ = 0.0;
  //! Maximal ground contact force
  double maxForce_ = std::numeric_limits<double>::max();
  //! Maximal ground contact force load constrained
  double maxForceLoadConstrained_ = std::numeric_limits<double>::max();
  //! Maximal ground contact force increment
  double maxIncrement_ = std::numeric_limits<double>::max();
};

template <cartesian::TranslationalCoordinates TranslationalCoordinates_ = cartesian::TranslationalCoordinates::XYZ>
class ForceLimitsConstraint : public ConstraintInterface {
 public:
  /** Constructor from default min / max force
   * @param wholeBody whole body container
   * @param terrain terrain model
   * @param defaultForceLimit, default force limits if no other was specified
   */
  ForceLimitsConstraint(const WholeBody& wholeBody, const TerrainModelBase& terrain, const ForceLimit& defaultForceLimit = ForceLimit());

  //! Destructor
  ~ForceLimitsConstraint() override = default;

  /** Update the jacobian and target values of the constraint
   * @param limbInfos  Distribution specific limb data
   */
  bool updateConstraint(const LimbInfos& limbInfos) override;

  //! Sets this constraint to a linear interpolation of constraint 1 and 2
  bool setToInterpolated(const ConstraintInterface& constraint1, const ConstraintInterface& constraint2, double t) override;

  /** Set default limits for normal ground forces
   * @param defaultNormalGroundForceLimits, default minimal/maximal ground contact force
   */
  void setDefaultNormalGroundForceLimits(const ForceLimit& defaultNormalGroundForceLimits);

  /** Set a specific limit to a limb (overwrite the default value for this limb)
   * @param limbID, Limb id for which the special force limit shall be active
   * @param normalGroundForceLimit, minimal/maximal ground contact force for limb
   */
  void addLimit(int limbId, const ForceLimit& normalGroundForceLimit);

  /** Get limit for a given limb
   * @param limbId, Limb id of the requested limit. If there is a special limit it is returned,
   *              else the default limit is returned
   */
  ForceLimit getLimit(int limbId) const;

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  bool loadParameters(const TiXmlHandle& handle) override;

 protected:
  /** Allows printing over cout
   *  @param stream that will be printed by cout
   */
  void print(std::ostream& out) const override;

 protected:
  //! Default force limits
  ForceLimit defaultNormalGroundForceLimits_;
  //! Special limits for limbs
  std::map<int, ForceLimit> limbNormalGroundForceLimits_;
};

} /* namespace loco */

#include "loco/contact_force_distribution/constraints/ForceLimitsConstraint.tpp"