/*!
 * @file     ContactForceDistribution.hpp
 * @author   Péter Fankhauser
 * @date     Mar 11, 2016
 * @brief
 */
#pragma once

#include <loco/contact_force_distribution/ContactForceDistributionInterface.hpp>

namespace loco {

class ContactForceDistributionDummy : public ContactForceDistributionInterface {
 public:
  /*!
   * Constructor.
   */
  ContactForceDistributionDummy() = default;

  /*!
   * Destructor.
   */
  ~ContactForceDistributionDummy() override = default;

  /*!
   * Computes the contact force distribution of the virtual force and torque.
   * @param virtualForce the desired virtual force on the base (in base frame).
   * @param virtualTorque the desired virtual torque on the base (in base frame).
   * @return true if successful.
   */
  bool computeForceDistribution(const Force& virtualForceInBaseFrame, const Torque& virtualTorqueInBaseFrame) override { return false; }

  /*!
   * Gets the distributed net forces and torques that act on the base, i.e.
   * this force and torque are computed from the distributed contact forces and
   * should ideally be equal to the desired net forces and torques.
   * @param[out] netForce.
   * @param[out] netTorque.
   * @return true if net force and torque can be calculated, false otherwise.
   */
  bool getNetForceAndTorqueOnBase(Force& netForce, Torque& netTorque) override { return false; }

  bool setToInterpolated(const ContactForceDistributionInterface& contactForceDistribution1,
                         const ContactForceDistributionInterface& contactForceDistribution2, double t) override {
    return false;
  }

  bool addConstraint(ConstraintInterfacePtr&& constraint) { return false; }
};

} /* namespace loco */
