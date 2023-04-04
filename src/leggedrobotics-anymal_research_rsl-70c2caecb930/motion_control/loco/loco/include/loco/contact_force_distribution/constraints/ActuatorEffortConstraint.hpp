/*
 * ActuatorEffortConstraint.hpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"
#include "loco/contact_force_distribution/constraints/ConstraintInterface.hpp"

// STL
#include <unordered_map>

namespace loco {

class ActuatorEffortConstraint : public ConstraintInterface {
 public:
  struct ActuatorEffortLimit {
    double minEffort_;
    double maxEffort_;
  };

 public:
  //! Constructor
  ActuatorEffortConstraint(const WholeBody& wholeBody, const TerrainModelBase& terrain);

  //! Destructor
  ~ActuatorEffortConstraint() override = default;

  /** Update the jacobian and target values of the constraint
   * @param limbInfos  Distribution specific limb data
   */
  bool updateConstraint(const LimbInfos& limbInfos) override;

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  bool loadParameters(const TiXmlHandle& handle) override;

 protected:
  ActuatorEffortLimit actuatorEffortLimit_;
};

} /* namespace loco */
