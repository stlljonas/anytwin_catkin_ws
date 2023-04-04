/*
 * ContactScheduleZmp.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// motion_generation
#include "motion_generation/ContactScheduleStrideControl.hpp"
#include "motion_generation/contact_schedule_zmp.hpp"


class TiXmlHandle;

namespace loco {

class ContactScheduleZmp : public ContactScheduleStrideControl {
public:
  using Base = ContactScheduleStrideControl;

  ContactScheduleZmp(WholeBody& wholeBody);
  ~ContactScheduleZmp() override = default;

  /** Advances the module.
   * @param dt  time step
   * @return    true, iff successful
   */
  bool advance(double dt) override;

  //! Add signals to logger.
  bool addVariablesToLog(const std::string & ns) const override;

  //! Returns contact events (in time domain) for one stride.
  const contact_schedule::ListOfEvents& getListOfEvents() const noexcept;

protected:

  //! Update list of events.
  virtual bool updateListOfEvents(double dt);

  //! Updates members legs.
  bool updateLegs();

  //! A reference to the legs.
  Legs& legs_;

  //! List of future phase events.
  contact_schedule::EventContainer eventContainer_;

  double adaptedStrideDuration_;
  double strideFeedback_;
  double nominalStrideDuration_;
};

} // namespace loco
