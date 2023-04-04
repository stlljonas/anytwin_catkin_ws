/*
 * EventDetectorOpt.hpp
 *
 *  Created on: Nov 14, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/event_detection/EventDetectorBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/gait_pattern/contact_schedule_anymal.hpp"

// std utils
#include "std_utils/std_utils.hpp"

class TiXmlHandle;

namespace loco {

class EventDetectorOpt: public EventDetectorBase {
  public:
    explicit EventDetectorOpt(WholeBody& wholeBody);
    ~EventDetectorOpt() override = default;

    bool addVariablesToLog(const std::string & ns = "") const override;
    bool initialize(double dt) override;
    bool loadParameters(const TiXmlHandle& handle) override;
    bool advance(double dt) override;

  protected:
    //! Computes friction modulation depending on friction state.
    virtual bool computeFrictionModulation(double dt);

    //! A reference to the legs.
    Legs& legs_;

    //! Elapsed time (used for VMC).
    double timeSinceInit_;

    //! If true, friction cones are shrunk as the end-effector slip.
    bool enableFrictionModulation_;

    //! Friction modulation will have the following value while slipping.
    double feedbackModulationWhileSlipping_;

    //! Actual friction modulation value for leg.
    std_utils::EnumArray<contact_schedule::LegEnumAnymal, double> frictionModulation_;

    //! Previous slipping state of a leg.
    std_utils::EnumArray<contact_schedule::LegEnumAnymal, bool> wasLegSlipping_;

    //! Time spend in stance for a leg.
    std_utils::EnumArray<contact_schedule::LegEnumAnymal, double> timeSpendInStance_;

    //! Time spend in stance (while not slipping) for a leg.
    std_utils::EnumArray<contact_schedule::LegEnumAnymal, double> timeSpendInStanceNoSlip_;
};

} /* namespace loco */
