/*
 * BatterySafetyStatus.hpp
 *
 *  Created on: Feb 27, 2017
 *      Author: gech
 */

#pragma once

#include <ostream>

namespace rpsm {


class BatterySafetyStatus {
 public:
  BatterySafetyStatus(): status_(0u) {

  }

  ~BatterySafetyStatus() {}

  bool isOvercharged() const {
    return (status_ & (1 << 20));
  }

  bool isChargeTimeout() const {
    return (status_ & (1 << 19));
  }

  bool isPrechargeTimeout() const {
    return (status_ & (1 << 16));
  }

  bool isUndertemperatureDuringDischarge() const {
    return (status_ & (1 << 11));
  }
  bool isUndertemperatureDuringCharge() const {
    return (status_ & (1 << 10));
  }

  bool isOvertemperatureDuringDischarge() const {
   return (status_ & (1 << 9));
  }

  bool isOvertemperatureDuringCharge() const {
   return (status_ & (1 << 8));
  }

  bool isShortCircuitDuringDischargeLatch() const {
    return (status_ & (1 << 7));
  }

  bool isShortCircuitDuringDischarge() const {
    return (status_ & (1 << 6));
  }

  bool isOverloadDuringDischargeLatch() const {
    return (status_ & (1 << 5));
  }
  bool isOverloadDuringDischarge() const {
    return (status_ & (1 << 4));
  }
  bool isOvercurrentDuringDischarge() const {
    return (status_ & (1 << 3));
  }
  bool isOvercurrentDuringCharge() const {
    return (status_ & (1 << 2));
  }
  bool isCellOvervoltage() const {
    return (status_ & (1 << 1));
  }
  bool isCellUndervoltage() const {
    return (status_ & (1 << 0));
  }

  friend std::ostream & operator << (std::ostream & out, const BatterySafetyStatus& status) {
    out << "Overcharge: " << (status.isOvercharged() ? "detected" : "not detected") << std::endl;
    out << "Charge Timeout: " << (status.isChargeTimeout() ? "detected" : "not detected") << std::endl;
    out << "Precharge Timeout: " << (status.isPrechargeTimeout() ? "detected" : "not detected") << std::endl;
    return out;
  }

  uint32_t status_;
};

} // namespace rpsm
