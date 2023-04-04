#include "anydrive/calibration/CalibrationTypeEnum.hpp"

namespace anydrive {
namespace calibration {

uint16_t calibrationTypeEnumToId(const CalibrationTypeEnum calibrationTypeEnum) {
  if (calibrationTypeEnum == CalibrationTypeEnum::Custom) {
    return OD_CALIB_SELECTION_ID_VAL_CUSTOM;
  }
  if (calibrationTypeEnum == CalibrationTypeEnum::Factory) {
    return OD_CALIB_SELECTION_ID_VAL_FACTORY;
  }
  return OD_CALIB_SELECTION_ID_VAL_CUSTOM;
}

CalibrationTypeEnum calibrationTypeIdToEnum(const uint16_t calibrationTypeId) {
  if (calibrationTypeId == OD_CALIB_SELECTION_ID_VAL_CUSTOM) {
    return CalibrationTypeEnum::Custom;
  }
  if (calibrationTypeId == OD_CALIB_SELECTION_ID_VAL_FACTORY) {
    return CalibrationTypeEnum::Factory;
  }
  return CalibrationTypeEnum::NA;
}

std::string calibrationTypeEnumToName(const CalibrationTypeEnum calibrationTypeEnum) {
  if (calibrationTypeEnum == CalibrationTypeEnum::Custom) {
    return "Custom";
  }
  if (calibrationTypeEnum == CalibrationTypeEnum::Factory) {
    return "Factory";
  }
  return "N/A";
}

CalibrationTypeEnum calibrationTypeNameToEnum(const std::string& calibrationTypeName) {
  if (calibrationTypeName == "Custom") {
    return CalibrationTypeEnum::Custom;
  }
  if (calibrationTypeName == "Factory") {
    return CalibrationTypeEnum::Factory;
  }
  return CalibrationTypeEnum::NA;
}

}  // namespace calibration
}  // namespace anydrive
