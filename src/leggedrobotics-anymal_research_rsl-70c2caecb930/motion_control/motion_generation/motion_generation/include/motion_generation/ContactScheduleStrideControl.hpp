/*
 * ContactScheduleStrideControl.hpp
 *
 *  Created on: June 07, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// motion_generation
#include "motion_generation/ContactSchedulePeriodicSwitch.hpp"

// basic_filters
#include "basic_filters/filters.hpp"

class TiXmlHandle;

namespace loco {

class ContactScheduleStrideControl : public ContactSchedulePeriodicSwitch {
public:
  using Base = ContactSchedulePeriodicSwitch;

  ContactScheduleStrideControl(WholeBody& wholeBody);
  ~ContactScheduleStrideControl() override = default;

  //! @copydoc ModuleBase::initialize(double dt) Initializes class.
  bool initialize(double dt) override;

  //! Add signals to logger.
  bool addVariablesToLog(const std::string & ns) const override;

  //! Return the stride duration. In case of a gait switch, the stride duration is interpolated between the active and the desired gait.
  double getInterpolatedStrideDuration() const;

  //! Returns filtered velocity error.
  double getLumpedFilteredVelocityError() const;

  //! Returns unfiltered velocity error.
  double getLumpedUnFilteredVelocityError() const noexcept;

  //! Update velocity error and stride feedback signals.
  bool updateVelocityFeedback(const LinearVelocity& linearVelocityDesiredInControlFrame);

protected:
  //! Reference to the legs.
  Legs& legs_;

  //! Velocity error signal used for stride feedback.
  double lumpedUnFilteredVelocityError_;

  //! First order filter to smooth velocity feedback correction.
  basic_filters::FirstOrderFilterD velocityFeedbackFilter_;
};

} // namespace loco
