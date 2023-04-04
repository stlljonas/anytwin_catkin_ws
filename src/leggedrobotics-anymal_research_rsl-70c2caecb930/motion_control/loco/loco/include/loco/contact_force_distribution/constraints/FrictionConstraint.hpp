/*
 * FrictionConstraint.hpp
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

class FrictionConstraint : public ConstraintInterface {
 public:
  //! Container for friction cone properties
  struct FrictionCone {
    //! Normal direction of the friction cone (z-dir)
    Vector normalDirectionInWorldFrame_;
    //! Primary direction of the friction cone (x-dir)
    Vector primaryDirectionInWorldFrame_;
    //! Secondary direction of the friction cone (y-dir)
    Vector secondaryDirectionInWorldFrame_;
    //! Friction coefficient at the contact point
    double frictionCoefficient_ = 0.5;
  };

 public:
  //! Constructor
  FrictionConstraint(const WholeBody& wholeBody, const TerrainModelBase& terrain);

  //! Destructor
  ~FrictionConstraint() override = default;

  /** Update the jacobian and target values of the constraint
   * @param limbInfos  Distribution specific limb data
   */
  bool updateConstraint(const LimbInfos& limbInfos) override;

 protected:
  //! Number of directions that are considered per limb (-x, x, -y, y)
  static constexpr unsigned int nDirections_ = 4;

  //! Map of the friction cones (used for logging)
  std::map<int, FrictionCone> frictionCones_;
};

} /* namespace loco */
