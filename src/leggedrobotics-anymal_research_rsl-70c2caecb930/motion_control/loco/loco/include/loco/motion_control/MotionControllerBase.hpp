/*!
 * @file     MotionControllerBase.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso
 * @date     March 6, 2014
 * @brief
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/WholeBody.hpp"

namespace loco {

class MotionControllerBase : public ModuleBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Constructor.
   */
  explicit MotionControllerBase(WholeBody& wholeBody);

  /*!
   * Destructor.
   */
  ~MotionControllerBase() override = default;

  /*! Sets the parameters to the interpolated ones between motionController1 and controller2.
   * @param motionController1     If the interpolation parameter is 0, then the parameter set is equal to the one of motionController1.
   * @param motionController2     If the interpolation parameter is 1, then the parameter set is equal to the one of motionController2.
   * @param t                     interpolation parameter in [0, 1]
   * @return                      true if successful
   */
  virtual bool setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2, double t);

 protected:
  WholeBody& wholeBody_;
  Limbs& limbs_;
  TorsoBase& torso_;
};

} /* namespace loco */
