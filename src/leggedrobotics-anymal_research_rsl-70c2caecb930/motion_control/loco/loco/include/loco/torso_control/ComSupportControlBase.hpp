/*!
 * @file     ComSupportControlBase.hpp
 * @author   Christian Gehring, C. Dario Bellicoso
 * @date     Oct 7, 2014
 * @brief
 */

#pragma once

// tinyxml
#include <tinyxml_tools/tinyxml_tools.hpp>

// eigen
#include <Eigen/Core>

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/legs/Legs.hpp"
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/common/typedefs.hpp"

namespace loco {

class ComSupportControlBase : public ModuleBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ComSupportControlBase(Legs& legs);
  ~ComSupportControlBase() override = default;

  /*! Gets the error vector from the center of all feet to the desired weighted location of the center of all feet in world coordinates
   * @param legs  references to the legs
   * @return error vector expressed in world frame
   */
  virtual const loco::Position& getPositionWorldToDesiredCoMInWorldFrame() const = 0;
  virtual const loco::LinearVelocity& getLinearVelocityDesiredBaseInWorldFrame() const;

  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  bool loadParameters(const TiXmlHandle& hParameterSet) override;

  /*! Stores the current paramters in the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool saveParameters(TiXmlHandle& hParameterSet);

  /*! Computes an interpolated version of the two support polygon settings passed in as parameters.
   *  if t is 0, the current setting is set to supportPolygon1, 1 -> supportPolygon2, and values in between
   *  correspond to interpolated parameter set.
   * @param supportPolygon1
   * @param supportPolygon2
   * @param t   interpolation parameter
   * @return  true if successful
   */
  virtual bool setToInterpolated(const ComSupportControlBase& supportPolygon1, const ComSupportControlBase& supportPolygon2, double t) = 0;

  double getMinSwingLegWeight() const;
  double getStartShiftAwayFromLegAtStancePhase() const;
  double getStartShiftTowardsLegAtSwingPhase() const;
  double getLateralOffset() const;
  double getHeadingOffset() const;

 protected:
  Legs& legs_;

  Position positionWorldToDesiredCoMInWorldFrame_;
  LinearVelocity linearVelocityDesiredBaseInWorldFrame_;

  //! this is the minimum weight any leg can have...
  //! if this is zero, then the COM will try to be right at center of the support polygon [0,1]
  double minSwingLegWeight_;

  //! this is the point in the stance phase when the body should start shifting away from the leg...
  //! if the stance phase is 1, the leg weight will be minimal
  double startShiftAwayFromLegAtStancePhase_;

  //! this is the point in the swing phase when the body should start shifting back towards the leg...
  //! if the swing phase is 1, the weight is fully back on the leg
  double startShiftTowardsLegAtSwingPhase_;

  double lateralOffset_;
  double headingOffset_;
};

} /* namespace loco */
