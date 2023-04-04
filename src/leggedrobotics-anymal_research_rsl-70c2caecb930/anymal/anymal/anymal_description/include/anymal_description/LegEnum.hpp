/**
 * @authors     S. Caron
 * @affiliation ANYbotics
 * @brief       Leg enumeration and convenience functions.
 */

#pragma once

// std_utils
#include <std_utils/std_utils.hpp>

namespace anymal_description {

/*! Enumeration of the legs:
 *
 *  - LF: left front
 *  - RF: right front
 *  - LH: left hint
 *  - RH: right hind
 *
 * @note This enumeration is similar to those that can be found in AnymalTopology.hpp.
 */
CONSECUTIVE_ENUM(LegEnum, LF, RF, LH, RH)

//! Map from @c LegEnum to corresponding string representations.
static std::map<LegEnum, std::string> mapLegEnumToString = {
    {LegEnum::LF, "LF"}, {LegEnum::RF, "RF"}, {LegEnum::LH, "LH"}, {LegEnum::RH, "RH"}, {LegEnum::SIZE, "SIZE"}};

//! Convenience operator for printing @c LegEnum.
inline std::ostream& operator<<(std::ostream& out, const LegEnum legEnum) {
  return out << mapLegEnumToString[legEnum];
}

/*! Iterator for use in for loops.
 *
 * @example Iterating over each leg goes as follows:
 * @code
 * for (const auto legEnum : LegEnumIterator()) {
 *   // ...
 * }
 * @endcode
 */
using LegEnumIterator = std_utils::enum_iterator<LegEnum>;

/*! Enumeration array.
 *
 * @example A map from @c LegEnum to strings goes as follows:
 * @code
 * LegEnumArray<std::string> stringForLeg;
 * stringForLeg[LegEnum::LF] = "LF touchdown";
 * @endcode
 *
 * @note This type is called "array" to conform with @c EnumArray from @c std_utils, but it is rather a map than an array.
 */
template <typename T>
using LegEnumArray = std_utils::EnumArray<LegEnum, T>;

}  // namespace anymal_description
