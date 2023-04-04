/*
 * ContactScheduleZmpTest.hpp
 *
 *  Created on: Aug 20, 2018
 *      Author: Markus Staeuble
 */

#pragma once

// motion_generation
#include "motion_generation/ContactScheduleZmp.hpp"

namespace loco {

class ContactScheduleZmpTest : public ContactScheduleZmp {
public:
  using Base = ContactScheduleZmp;

  ContactScheduleZmpTest(WholeBody& wholeBody);
  ~ContactScheduleZmpTest() override = default;

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


  bool setDesiredGaitById(const unsigned int desiredGaitId, contact_schedule::ContactScheduleSwitchStatus& status)  override;

    //! Switch from walk to stand (soft stop).
  bool switchToStand(contact_schedule::ContactScheduleSwitchStatus& status) override;

  /*! @rSwitch from stand to walk.
   * @param isGaitForward         true if gait is started in forward motion.
   * @param startGaitForBalancing true if the gait is started to stabilize the robot,
   *                              false, if the gait is started according to high-level commands
   */
  bool switchToWalk(contact_schedule::ContactScheduleSwitchStatus& status, bool isGaitForward, bool startBalancingGait = false) override;


  void setCyclePhase(const double cyclePhase) noexcept { stridePhase_ = cyclePhase; }

  double getCyclePhase() const { return stridePhase_; }
  
protected :
  bool updateLegs(double dt);

  double timeElapsedSinceGaitSwitchStartLocal_;
  double polygonTimeThreshhold_;

};

} // namespace loco
