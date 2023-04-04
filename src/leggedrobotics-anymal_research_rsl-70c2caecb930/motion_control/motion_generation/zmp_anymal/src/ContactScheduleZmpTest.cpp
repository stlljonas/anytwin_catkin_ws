/*
 * ContactScheduleZmpTest.cpp
 *
 *  Created on: Aug 20, 2018
 *      Author: Markus Staeuble
 *
 *      build: catkin build zmp_anymal --no-deps --catkin-make-args run_tests
 *      launch: rostest --reuse-master zmp_anymal zmp_anymal.test -t
 */

// anymal_description
#include <anymal_description/LegEnum.hpp>

// loco
#include "zmp_anymal/ContactScheduleZmpTest.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace loco {
using namespace message_logger::color;

ContactScheduleZmpTest::ContactScheduleZmpTest(WholeBody& wholeBody) :
    Base(wholeBody),
    timeElapsedSinceGaitSwitchStartLocal_(0.0),
    polygonTimeThreshhold_(0.05)

{
}

bool ContactScheduleZmpTest::loadParameters(const TiXmlHandle& handle) {
  const TiXmlHandle comSupportHandle = tinyxml_tools::getChildHandle(handle, "ComSupportControl");
  const TiXmlHandle conditioningHandle = tinyxml_tools::getChildHandle(comSupportHandle, "Conditioning");
  if(!tinyxml_tools::loadParameter(polygonTimeThreshhold_, conditioningHandle, "skip_polygon_if_time_smaller_than")) { return false; }
  return Base::loadParameters(handle);
}

bool ContactScheduleZmpTest::advance(double dt) {
  if (status_ != contact_schedule::Status::Stand &&
      status_ != contact_schedule::Status::Walk) {
    MELO_WARN_STREAM("[ContactScheduleZmpTest::advance]  Wrong status " << contact_schedule::statusMap[status_] <<
                     ". Status must be Stand or Walk!");
    return false;
  }

//  if(!updateMotionStatus()) {
//    MELO_WARN_STREAM("[ContactScheduleZmp::advance] Failed to update motion status.");
//    return false;
//  }

  if(!updateLegs(dt)) {
    MELO_WARN_STREAM("[ContactScheduleZmpTest::advance] Failed to update leg container.");
    return false;
  }

  if(!updateListOfEvents(dt)) {
    MELO_WARN_STREAM("[ContactScheduleZmpTest::advance] Failed to update list of events.");
    return false;
  }

  return true;
}


bool ContactScheduleZmpTest::setDesiredGaitById(const unsigned int desiredGaitId, contact_schedule::ContactScheduleSwitchStatus& status) {
  MELO_INFO_STREAM("[ContactScheduleZmpTest::setDesiredGaitById] Set new gait.");

  // Since we are not walking, we can switch the gait only while standing
  // (otherwise we would never exit the status SwitchGait)
  if (status_ != contact_schedule::Status::Stand) {
    MELO_WARN_STREAM("[ContactScheduleZmpTest::setDesiredGaitById] Wrong status " << contact_schedule::statusMap[status_] <<
                     ". Status must be Stand!");
    return false;
  }

  if(!updateDesiredGait(desiredGaitId)) { return false; }
  if(!updateActiveGait(desiredGaitId)) { return false; }
  status = contact_schedule::ContactScheduleSwitchStatus::Switched;
  return updateLegs(0.0);
}
  

bool ContactScheduleZmpTest::switchToWalk(contact_schedule::ContactScheduleSwitchStatus& status, bool isForwardDirection, bool startBalancingGait) {
  MELO_INFO_STREAM("[ContactScheduleZmpTest::switchToWalk] Switch to walk.");

  if (status_ != contact_schedule::Status::Stand) {
    MELO_WARN_STREAM("[ContactScheduleZmpTest::setDesiredGaitById] Wrong status " << contact_schedule::statusMap[status_] <<
                     ". Status must be Stand!");
    return false;
  }

  stridePhase_ = 0.0; // reset cycle phase
  status_ = contact_schedule::Status::Walk; // switch from stance to walk
  status = contact_schedule::ContactScheduleSwitchStatus::Switched;
  return updateLegs(0.0);
}


bool ContactScheduleZmpTest::switchToStand(contact_schedule::ContactScheduleSwitchStatus& status) {
  MELO_INFO_STREAM("[ContactScheduleZmpTest::switchToStand] Switch to stand.");
  status_ = contact_schedule::Status::Stand; // switch from walk to stance.
  if(!stopGait()) { return false; } // active stance-gait.
  status = contact_schedule::ContactScheduleSwitchStatus::Switched;
  return true;
}

bool ContactScheduleZmpTest::updateLegs(double dt) {
  if (status_ == contact_schedule::Status::Walk) {
    // Update time spend in event.
    if (!ContactSchedulePeriodic::updateTimeEvents(dt)) {
      return false;
    }

    // Set measured leg state equal to desired leg state.
    for (const auto& legId : anymal_description::LegEnumIterator()) {
      auto* leg = legs_.getPtr(static_cast<unsigned int>(legId));
      leg->getContactSchedulePtr()->setShouldBeGrounded(shouldBeLegGrounded(legId));
      leg->getContactSchedulePtr()->setIsGrounded(shouldBeLegGrounded(legId));
    }

    // Check if support polygon duration would be too small.
    if (!eventContainer_.getListOfEvents().empty() && polygonTimeThreshhold_> 0.0) {
      const double firstEventDuration = eventContainer_.getListOfEvents().begin()->first;
      const double strideDuration = getActiveGaitDescription().getStrideDuration();
      if (strideDuration<=0.0) { return false; }
      if (firstEventDuration <= polygonTimeThreshhold_) {
        // Increase stride phase.
        stridePhase_ += polygonTimeThreshhold_ / getActiveGaitDescription().getStrideDuration();
        if (stridePhase_>=1.0) { stridePhase_ -= 1.0; }

        // Recompute event-container.
        if(!updateListOfEvents(dt)) {
          MELO_WARN_STREAM("[ContactScheduleZmpTest::switchToStand] Failed to update list of events.");
          return false;
        }

        // Try again.
        return updateLegs(dt);
      }
    }

  }

  // Set measured leg state equal to desired leg state.
  else if (status_ == contact_schedule::Status::Stand) {
    for (const auto& legId : anymal_description::LegEnumIterator()) {
      auto* leg = legs_.getPtr(static_cast<unsigned int>(legId));
      leg->getContactSchedulePtr()->setShouldBeGrounded(true);
      leg->getContactSchedulePtr()->setIsGrounded(true);
    }
  }

  else {
    MELO_WARN_STREAM("[ContactScheduleZmpTest::updateLegs] Wrong status " << contact_schedule::statusMap[status_] <<
                     ". Status must be Stand or Walk!");
  }

  return true;
}

}
