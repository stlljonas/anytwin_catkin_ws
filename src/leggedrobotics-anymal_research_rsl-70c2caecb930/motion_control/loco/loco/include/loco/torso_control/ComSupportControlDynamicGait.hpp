/*!
 * @file 	  ComSupportControl.hpp
 * @author 	Christian Gehring, Stelian Coros
 * @date		  Jul 17, 2012
 * @brief
 */

#pragma once

#include "loco/torso_control/ComSupportControlBase.hpp"

namespace loco {

//! Support Polygon Task
/*! To maintain balance, we want to keep the projected CoM within the support polygon.
 * This class computes the error vector from the center of all feet to the desired weighted location of the center of all feet in world
 * coordinates. The error vector can then be used by a virtual force controller.
 */
class ComSupportControlDynamicGait : public ComSupportControlBase {
 public:
  //! Constructor
  explicit ComSupportControlDynamicGait(Legs& legs);

  //! Destructor
  ~ComSupportControlDynamicGait() override = default;

  /*! Gets the error vector from the center of all feet to the desired weighted location of the center of all feet in world coordinates
   * @param legs	references to the legs
   * @return error vector expressed in world frame
   */
  const Position& getPositionWorldToDesiredCoMInWorldFrame() const override;

  bool advance(double dt) override;

  bool initialize(double dt) override;

  bool setToInterpolated(const ComSupportControlBase& supportPolygon1, const ComSupportControlBase& supportPolygon2, double t) override;

 protected:
};

}  // namespace loco
