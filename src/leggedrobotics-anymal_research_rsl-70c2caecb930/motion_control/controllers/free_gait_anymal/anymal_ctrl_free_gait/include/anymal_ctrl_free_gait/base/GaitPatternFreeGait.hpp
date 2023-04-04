/*
 * GaitPatternFreeGait.hpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "tinyxml.h"

// STL
#include <memory>
#include <vector>

// Loco
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "loco_anymal/common/LegsAnymal.hpp"

// Anymal model
#include "anymal_model/AnymalModel.hpp"

// Free Gait
#include "free_gait_core/free_gait_core.hpp"

namespace loco {

class GaitPatternFreeGait : public GaitPatternBase {
 public:
  GaitPatternFreeGait(loco_anymal::LegsAnymal& legs, TorsoBase& torso, anymal_model::AnymalModel& anymalModel,
                      free_gait::Executor& executor);

  ~GaitPatternFreeGait() override = default;

  /*! Loads the parameters from the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  bool loadParameters(const TiXmlHandle& hParameterSet) override;

  /*! Stores the current parameters in the XML object
   * @param hParameterSet   handle
   * @return  true if all parameters could be loaded
   */
  virtual bool saveParameters(TiXmlHandle& hParameterSet);

  bool initialize(double dt) override;

  bool isInitialized();

  bool reset();

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  double getSwingPhaseForLeg(int iLeg) const override;

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  double getSwingPhaseForLeg(int iLeg, double stridePhase) const override;

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  double getStancePhaseForLeg(int iLeg) const override;

  /*!  @returns the relative stance phase for the leg. If the limb is in swing mode, it returns -1
   */
  double getStancePhaseForLeg(int iLeg, double stridePhase) const override;

  //! @returns the total length of the stance phase in seconds
  double getStanceDuration(int iLeg) const override;

  //! @returns the total length of the swing phase in seconds
  double getSwingDuration(int iLeg, double strideDuration) const override;

  //! @returns the total length of the swing phase in seconds, ignores the stride duration
  virtual double getSwingDuration(int iLeg) const { return getSwingDuration(iLeg, 0.0); }

  //! @returns the total length of the stance phase in seconds for a a given stride duration
  double getStanceDuration(int iLeg, double strideDuration) const override;

  double getStrideDuration() const override;
  void setStrideDuration(double strideDuration) override;

  unsigned long int getNGaitCycles() const override;

  void setVelocity(double value);

  double getVelocity();

  /*! Advance in time
   * @param dt  time step [s]
   */
  bool advance(double dt) override;

  bool shouldBeLegGrounded(int iLeg) const override;

  /*! @returns stride (cycle) phase, which is between [0, 1].
   */
  double getStridePhase() const override;

  /*! Sets the stride (cycle phase), which is between [0, 1].
   * @param stridePhase cycle phase
   */
  void setStridePhase(double stridePhase) override;

  /*! @returns the time left in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  double getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const override;

  /*! @returns the time left in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  double getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const override;

  /*! @returns the time spent in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  double getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const override;

  /*! @returns the time spent in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  double getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const override;

  /*! @returns time until next stance phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  double getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const override;

  /*! @returns time until next swing phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  double getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const override;

  /*! @returns number of legs that are in stance mode
   * @param stridePhase   stride phase
   */
  int getNumberOfStanceLegs(double stridePhase) const override;

 private:
  //! True if initialized, false otherwise.
  bool isInitialized_;

  //! Legs information container.
  loco_anymal::LegsAnymal& legs_;

  //! Torso information container.
  TorsoBase& torso_;

  //! Robot model.
  anymal_model::AnymalModel& anymalModel_;

  //! Free gait executor.
  free_gait::Executor& executor_;
};

}  // namespace loco
