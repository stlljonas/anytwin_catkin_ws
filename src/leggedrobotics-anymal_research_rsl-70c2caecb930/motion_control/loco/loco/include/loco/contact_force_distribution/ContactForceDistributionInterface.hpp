/*!
 * @file     ContactForceDistributionInterface.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso, Gabriel Hottiger
 * @date     Aug 6, 2013 / Jan 17, 2017
 * @brief
 */

#pragma once

// loco
#include "loco/common/limbs/Limbs.hpp"
#include "loco/common/typedefs.hpp"
#include "loco/contact_force_distribution/constraints/ConstraintInterface.hpp"

// stl
#include <ostream>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {

//! This class distributes a virtual force and torque on the base as forces to the leg contact points.
class ContactForceDistributionInterface {
 public:
  //! Default Constructor
  ContactForceDistributionInterface() = default;

  //! Default Destructor
  virtual ~ContactForceDistributionInterface() = default;

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  virtual bool loadParameters(const TiXmlHandle& handle) { return true; }

  /*!
   * Adds class data to the logger (optional).
   * @return true if successful.
   */
  virtual bool addVariablesToLog(const std::string& ns) const { return true; }

  /**
   * @brief Add a constraint to the force distribution
   * @param constraint Constraint to be added
   * @return true if successfully added constraint
   */
  virtual bool addConstraint(ConstraintInterfacePtr&& constraint) = 0;

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force on the base (in base frame).
   * @param virtualTorque the desired virtual torque on the base (in base frame).
   * @return true if successful.
   */
  virtual bool computeForceDistribution(const Force& virtualForceInBaseFrame, const Torque& virtualTorqueInBaseFrame) = 0;

  /*!
   * Gets the distributed net forces and torques that act on the base, i.e.
   * this force and torque are computed from the distributed contact forces and
   * should ideally be equal to the desired net forces and torques.
   * @param[out] netForce.
   * @param[out] netTorque.
   * @return true if net force and torque can be calculated, false otherwise.
   */
  virtual bool getNetForceAndTorqueOnBase(Force& netForce, Torque& netTorque) = 0;

  /*! Computes an interpolated version of the two force distributions passed in as parameters.
   *  if t is 0, the current setting is set to contactForceDistribution1,
   *  1 -> contactForceDistribution2, and values in between
   *  correspond to interpolated parameter set.
   * @param contactForceDistribution1
   * @param contactForceDistribution2
   * @param t interpolation parameter
   * @returns true if successful
   */
  virtual bool setToInterpolated(const ContactForceDistributionInterface& contactForceDistribution1,
                                 const ContactForceDistributionInterface& contactForceDistribution2, double t) = 0;

  /** Overloads to << ostream operator for cfds, this allows e.g. std::cout << cfd << std::endl
   * @param out ostream of the << operation
   * @return ostream
   */
  friend std::ostream& operator<<(std::ostream& out, const ContactForceDistributionInterface& cfd) {
    cfd.print(out);
    return out;
  }

 protected:
  /** Allows printing over cout
   *  @param stream that will be printed by cout
   */
  virtual void print(std::ostream& out) const {}
};

} /* namespace loco */
