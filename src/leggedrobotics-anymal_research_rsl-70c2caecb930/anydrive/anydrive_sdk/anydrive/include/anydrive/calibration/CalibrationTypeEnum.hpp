#pragma once

#include <cstdint>
#include <string>

#include "anydrive/common/ObjectDictionary.hpp"

namespace anydrive {
namespace calibration {

//! Calibration type enumerators for type safe usage.
enum class CalibrationTypeEnum { Custom, Factory, NA };

/*!
 * Convert a calibration type enumerator to an ID.
 * @param calibrationTypeEnum Calibration type enumerator.
 * @return Calibration type ID.
 */
uint16_t calibrationTypeEnumToId(const CalibrationTypeEnum calibrationTypeEnum);

/*!
 * Convert a calibration type ID to an enumerator.
 * @param calibrationTypeId Calibration type ID.
 * @return Calibration type enumerator.
 */
CalibrationTypeEnum calibrationTypeIdToEnum(const uint16_t calibrationTypeId);

/*!
 * Convert a calibration type enumerator to a human readable string (GUI, etc.).
 * @param calibrationTypeEnum Calibration type enumerator.
 * @return Human readable string.
 */
std::string calibrationTypeEnumToName(const CalibrationTypeEnum calibrationTypeEnum);

/*!
 * Convert a human readable string (GUI, etc.) to a calibration type enumerator.
 * @param calibrationTypeName Human readable string.
 * @return Calibration type enumerator.
 */
CalibrationTypeEnum calibrationTypeNameToEnum(const std::string& calibrationTypeName);

}  // namespace calibration
}  // namespace anydrive
