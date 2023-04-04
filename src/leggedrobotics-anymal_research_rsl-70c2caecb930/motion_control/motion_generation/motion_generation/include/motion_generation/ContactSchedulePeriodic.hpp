/*
 * ContactSchedulePeriodic.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// motion_generation
#include "motion_generation/GaitDescription.hpp"
#include "loco/gait_pattern/contact_schedules.hpp"
#include "motion_generation/contact_schedule_periodic.hpp"


class TiXmlHandle;

namespace loco {

class ContactSchedulePeriodic : public contact_schedule::ContactScheduleAnymalBase {
public:
  using Base = contact_schedule::ContactScheduleAnymalBase;
  using ListOfGaits = std::vector<contact_schedule::periodic::GaitDescription>;

  ContactSchedulePeriodic();
  ~ContactSchedulePeriodic() override = default;

  /** Initializes the module.
   * @param dt  time step
   * @return true, iff successful
   */
  bool initialize(double dt) override;

  /** Loads parameters from an xml file
   *  @param handle tinyxml handle
   *  @return true, iff successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /** Advances the module.
   * @param dt  time step
   * @return    true, iff successful
   */
  bool advance(double dt) override;

  //! Returns stride duration.
  double getStrideDuration() const noexcept;

  //! Returns nominal stride duration of active gait.
  double getNominalStrideDuration() const noexcept;

  //! Set nominal stride duration and update active gait.
  void setActiveNominalStrideDuration(double nominalStrideDuration) noexcept;

  //! Returns name of the active gait.
  const std::string& getActiveGaitName() const noexcept;

  //! Returns active gait index.
  unsigned int getActiveGaitId() const noexcept;

  //! Get map from gait name to gait index.
  const std::map<std::string, unsigned int>& getMapGaitNameToId() const noexcept;

  //! Given a gait index, return the gait name.
  bool mapGaitIdToName(unsigned int gaitId, std::string& gaitName) const noexcept;

  //! Return true if gaitName is an available gait.
  bool isValidGait(const std::string& gaitName) const noexcept;
  bool isValidGait(unsigned int gaitId) const noexcept;

  //! If true, gait is stopped with balancing gait.
  bool stopActiveGaitWithBalancingGait() const noexcept;

  //! If true, gait is using elevation map.
  double getIsPerceptiveGait(unsigned int gaitId) const;

  //! Retrun true if gait is reversible
  bool isActiveGaitReversible() const noexcept;

  // Get legs which are used for walking and the grounded legs.
  virtual const std::vector<contact_schedule::LegEnumAnymal>& getLegEnumsUsedForWalk() const;
  virtual const std::vector<contact_schedule::LegEnumAnymal>& getLegEnumsUsedForStand() const;
  bool isLegIdGrounded(contact_schedule::LegEnumAnymal legId) const;

protected:
  /*! @Routine that updates stride phase.
   * @param dt  time step
   */
  bool updateStride(double dt);

  /*! @Routine that updates timing vectors
   * @param dt  time step
   */
  virtual bool updateTimeEvents(double dt);

  bool updatePreviousEvent(
      bool shouldLegBeGrounded,
      contact_schedule::LegEnumAnymal legId) noexcept;

  //! Update active gait with gait index.
  bool updateActiveGait(unsigned int activeGaitId);

  //! Update active gait with gait name.
  bool updateActiveGait(const std::string& activeGaitName);

  //! Helper function to load phase events.
  bool loadEvents(
      const std::vector<TiXmlElement*>& eventElements,
      contact_schedule::LegEnumAnymal legId,
      double swingPhase);

  //! Returns gait description corresponding to active gait.
  const contact_schedule::periodic::GaitDescription& getActiveGaitDescription() const;
  contact_schedule::periodic::GaitDescription& getActiveGaitDescription();

  //! Phase in [0, 1).
  double stridePhase_;

  //! Definition of lift-off and touch-down events in phase domain for each periodic gait.
  ListOfGaits gaitDescription_;

  //! Name of the (currently) active gait.
  std::string activeGaitName_;

  //! Active gait index.
  unsigned int activeGaitId_;

  //! A list of names of available gaits.
  std::map<std::string, unsigned int> mapGaitNameToId_;

  //! Default gait index.
  unsigned int defaultGaitId_;

  //! if true, some information are displayed to the console.
  bool verbose_;

  //! Name of the gait used to balance the robot while stance.
  std::string balancingGaitName_;

  //! Velocity is considered to be zero if it is less than the following threshold.
  double zeroVelocityThreshold_;
};

} // namespace loco
