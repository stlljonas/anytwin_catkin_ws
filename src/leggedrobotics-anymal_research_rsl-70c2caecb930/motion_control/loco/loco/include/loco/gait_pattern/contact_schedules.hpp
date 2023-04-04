/*
 * contact_schedules.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/gait_pattern/ContactScheduleBase.hpp"

namespace loco {
namespace contact_schedule {
using ContactScheduleAnymalBase = ContactScheduleBase<contact_schedule::LegEnumAnymal>;
}
}  // namespace loco
