/*!
 * @file     VirtualModelController.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso
 * @date     Aug 6, 2013
 * @brief
 */

#pragma once

#include <loco/motion_control/ContactInvariantDamper.hpp>
#include <loco/motion_control/VirtualModelController.hpp>

namespace loco {

class VirtualModelControllerContactInvariantDamper : public VirtualModelController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   */
  VirtualModelControllerContactInvariantDamper(WholeBody& wholeBody, ContactForceDistributionInterface& contactForceDistribution);

  /*!
   * Destructor.
   */
  ~VirtualModelControllerContactInvariantDamper() override = default;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*!
   * Add variables to log (optional).
   * @return true if successful.
   */
  bool addVariablesToLog(const std::string& ns) const override;

  bool addParametersToHandler(const std::string& ns) override;

  bool initialize(double dt) override;

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful.
   */
  bool advance(double dt) override;

 protected:
  //! Contact invariant damper
  ContactInvariantDamper contactInvariantDamper_;
};

} /* namespace loco */
