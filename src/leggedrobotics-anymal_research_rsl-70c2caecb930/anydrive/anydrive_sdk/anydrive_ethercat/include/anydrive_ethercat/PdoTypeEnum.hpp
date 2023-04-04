#pragma once

namespace anydrive_ethercat {

enum class PdoTypeEnum {
  NA,
  A,  // Smallest PDO without IMU.
  B,  // Smallest PDO with IMU.
  C,  // Full PDO with support for Real Time Data Logging (RTDL).
  D   // [DEPRECATED] Full PDO.
};

}  // namespace anydrive_ethercat
