/*
 * ConstraintInterface.hpp
 *
 *  Created on: Jan 16, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// numopt
#include "numopt_common/LinearFunctionConstraints.hpp"

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/contact_force_distribution/LimbInfos.hpp"

// STL
#include <memory>
#include <string>

namespace loco {

class ConstraintInterface : public numopt_common::LinearFunctionConstraints {
 public:
  /** Constructor
   * @param name Constraint name
   * @param wholeBody whole body container
   * @param terrain terrain model
   */
  ConstraintInterface(std::string name, const WholeBody& wholeBody, const TerrainModelBase& terrain)
      : wholeBody_(wholeBody), terrain_(terrain), name_(std::move(name)) {}

  //! Destructor
  ~ConstraintInterface() override = default;

  /** Update the jacobian and target values of the constraint
   * @param limbInfos  Distribution specific limb data
   */
  virtual bool updateConstraint(const LimbInfos& limbInfos) = 0;

  //! Sets this constraint to a linear interpolation of constraint 1 and 2
  virtual bool setToInterpolated(const ConstraintInterface& constraint1, const ConstraintInterface& constraint2, double t) { return true; }

  /*!
   * Loads the parameters. Has to be done before using this class.
   * @return true if successful.
   */
  virtual bool loadParameters(const TiXmlHandle& handle) { return true; }

  //! @param Constraint name to be set
  void setConstraintName(const std::string& name) { name_ = name; }

  //! @return Constraint name
  const std::string& getConstraintName() const { return name_; }

  /** Overloads to << ostream operator for constraints, this allows e.g. std::cout << constraint << std::endl
   * @param out ostream of the << operation
   * @return ostream
   */
  friend std::ostream& operator<<(std::ostream& out, const ConstraintInterface& constraint) {
    constraint.print(out);
    return out;
  }

 protected:
  /** Allows printing over cout
   *  @param stream that will be printed by cout
   */
  virtual void print(std::ostream& out) const {}

 protected:
  //! Whole body
  const loco::WholeBody& wholeBody_;
  //! Terrain
  const loco::TerrainModelBase& terrain_;
  //! Name of the constraint
  std::string name_;
};

using ConstraintInterfacePtr = std::unique_ptr<ConstraintInterface>;

}  // namespace loco
