/*
 * GaitPatternFlightPhases.hpp
 *
 *  Created on: Mar 14, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/gait_pattern/FootFallPattern.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"

// stl
#include <vector>

namespace loco {

class GaitPatternFlightPhases : public GaitPatternBase {
 public:
  explicit GaitPatternFlightPhases(WholeBody& wholeBody);
  ~GaitPatternFlightPhases() override = default;

  /*! @returns the stride duration in seconds
   */
  double getStrideDuration() const override;

  /*! Sets the stride duration
   * @param strideDuration  stride duration in seconds
   */
  void setStrideDuration(double strideDuration) override;

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
  virtual double getSwingDuration(int iLeg) const;

  //! @returns the total length of the swing phase in seconds for a a given stride duration
  double getSwingDuration(int iLeg, double strideDuration) const override;

  //! @returns the total length of the stance phase in seconds for a a given stride duration
  double getStanceDuration(int iLeg, double strideDuration) const override;

  double getTimeLeftInStance(int iLeg, double strideDuration, double stridePhase) const override;
  double getTimeLeftInSwing(int iLeg, double strideDuration, double stridePhase) const override;

  double getTimeSpentInStance(int iLeg, double strideDuration, double stridePhase) const override;
  double getTimeSpentInSwing(int iLeg, double strideDuration, double stridePhase) const override;

  double getTimeUntilNextStancePhase(int iLeg, double strideDuration, double stridePhase) const override;
  double getTimeUntilNextSwingPhase(int iLeg, double strideDuration, double stridePhase) const override;

  /*! @returns number of legs that are in stance mode
   * @param stridePhase   stride phase
   */
  int getNumberOfStanceLegs(double stridePhase) const override;

  /*!  @returns number of gait cycles
   */
  unsigned long int getNGaitCycles() const override;

  bool initialize(double dt) override;

  bool isInitialized() const;

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
  virtual void setStridePhaseAndUpdate(double stridePhase);

  bool loadParameters(const TiXmlHandle& handle) override;

  bool addVariablesToLog(bool update) override;

  double getFootLiftOffPhase(int iLeg) const;
  double getFootTouchDownPhase(int iLeg) const;

  /*!
    computed an interpolated version of the two gaits passed in as parameters.
    if t is 0, the current gait is set to gait1, 1 -> gait 2, and values in between
    correspond to interpolated gaits.
  */
  bool setToInterpolated(const GaitPatternBase& gaitPattern1, const GaitPatternBase& gaitPattern2, double t) override;

  void clear();

  int getNumberOfLegs() const;

  void addFootFallPattern(int legId, double liftOffPhase, double strikePhase);

  friend std::ostream& operator<<(std::ostream& out, const GaitPatternFlightPhases& gaitPattern);

  void setInitialPhase(double phase);
  double getInitialPhase() const;

 protected:
  virtual void updateTorsoAndLegs();

 protected:
  //! A reference to the torso.
  TorsoBase& torso_;

  //! A reference to the legs.
  Legs& legs_;

  //! A flag which defines if the class was initialized.
  bool isInitialized_;

  //! The current phase of the gait cycle.
  double cyclePhase_;

  //! This is how long it should take from the time one particular foot leaves the ground, until it leaves the ground again next in seconds.
  double strideDuration_;

  //! The gait should start at this stride phase in [0,1]
  double initCyclePhase_;

  //! Number of gait cycles since start
  unsigned long int numGaitCycles_;

  //! A container of contact schedules for the feet.
  std::vector<FootFallPattern> stepPatterns_;

 protected:
  /*!
    Given a "circular" domain:

    (and 0.95 is in the range [-0.1, 0.1], and 0.05 is in the range [0.9, 1.1], for instance).

    start and end must satisfy the condition:
      start >= 0 || end <= 1
    and phase must satisfy:
      phase >= 0 && phase <= 1

    This method returns a number between 0 and 1 if phase is in the range start-end
    It returns 0 if phase is just at the start of the interval, 1 if it's at the end (interpolated
    values in between) and -1 if the phase is not in the range.
  */
  double getRelativePhaseFromAbsolutePhaseInRange(double phase, double start, double end) const;
  virtual void updateGaitPattern(double dt);

  int getStepPatternIndexForLeg(int legId) const;
};

} /* namespace loco */
