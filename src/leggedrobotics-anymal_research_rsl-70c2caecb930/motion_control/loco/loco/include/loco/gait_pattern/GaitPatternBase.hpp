/*!
 * @file     GaitPatternBase.hpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Jun, 2013
 * @version  1.0
 * @ingroup
 * @brief
 */

#pragma once

// stl
#include <string>

// loco
#include "loco/gait_pattern/ContactScheduleLock.hpp"

class TiXmlHandle;

namespace loco {

class GaitPatternBase : public ContactScheduleLock {
 public:
  GaitPatternBase();
  ~GaitPatternBase() override = default;

  /*! @returns the stride duration in seconds
   */
  virtual double getStrideDuration() const = 0;

  /*! Sets the stride duration
   * @param strideDuration  stride duration in seconds
   */
  virtual void setStrideDuration(double strideDuration) = 0;

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg) const = 0;

  /*! @returns the relative phase for the leg whose index is passed in. The number
    returned is always going to be between 0 and 1 (0 meaning it should still be in stance mode,
    1 - it is a stance leg again, anything in between means that it is a swing leg).
    The stridePhase is expected to be between 0 and 1.
  */
  virtual double getSwingPhaseForLeg(int iLeg, double stridePhase) const = 0;

  //! returns the relative stance phase for the leg. If the leg is in swing mode, it returns -1
  virtual double getStancePhaseForLeg(int iLeg) const = 0;

  /*!  @returns the relative stance phase for the leg. If the limb is in swing mode, it returns -1
   */
  virtual double getStancePhaseForLeg(int iLeg, double stridePhase) const = 0;

  //! @returns the total length of the swing phase in seconds for a a given stride duration
  virtual double getSwingDuration(int iLeg, double strideDuration) const = 0;

  //! @returns the total length of the stance phase in seconds for a a given stride duration
  virtual double getStanceDuration(int iLeg, double strideDuration) const = 0;

  //! @returns the total length of the stance phase in seconds
  virtual double getStanceDuration(int iLeg) const = 0;

  /*!  @returns number of gait cycles
   */
  virtual unsigned long int getNGaitCycles() const = 0;

  /*! @returns number of legs that are in stance mode
   * @param stridePhase   stride phase
   */
  virtual int getNumberOfStanceLegs(double stridePhase) const = 0;

  /*! @returns the time left in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns the time left in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns the time spent in stance in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns the time spent in swing in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the current stride
   */
  virtual double getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns time until next stance phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  virtual double getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const = 0;

  /*! @returns time until next swing phase starts in seconds.
   * @param iLeg              index of leg
   * @param strideDuration    stride duration in seconds
   * @param stridePhase       phase of the stride
   */
  virtual double getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const = 0;

  virtual bool addVariablesToLog(bool update);

  virtual bool shouldBeLegGrounded(int iLeg) const = 0;

  /*! @returns stride (cycle) phase, which is between [0, 1].
   */
  virtual double getStridePhase() const = 0;

  /*! Sets the stride (cycle phase), which is between [0, 1].
   * @param stridePhase cycle phase
   */
  virtual void setStridePhase(double stridePhase) = 0;

  /*!
    computed an interpolated version of the two gaits passed in as parameters.
    if t is 0, the current gait is set to gait1, 1 -> gait 2, and values in between
    correspond to interpolated gaits.
  */
  virtual bool setToInterpolated(const GaitPatternBase& gaitPattern1, const GaitPatternBase& gaitPattern2, double t);
};

}  // namespace loco
