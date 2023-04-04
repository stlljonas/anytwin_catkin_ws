#pragma once

#include <ostream>

namespace rpsm {

class BatteryStatus {
 public:
  BatteryStatus(): status_(0) {

  }
  virtual ~BatteryStatus() {

  }
  typedef uint16_t Status;
  bool isOverChargedAlarmActive() const {
    return (status_ & (1 << 15));
  }
  bool isTerminateChargeAlarmActive() const {
    return (status_ & (1 << 14));
  }
  bool isOverTemperatureAlarmActive() const {
    return (status_ & (1 << 12));
  }
  bool isTerminateDischargeAlarmActive() const {
    return (status_ & (1 << 11));
  }
  bool isRemainingCapacityAlarmActive() const {
    return (status_ & (1 << 9));
  }
  bool isRemainingTimeAlarmActive() const {
    return (status_ & (1 << 8));
  }
  bool isInitializationActive() const {
    return (status_ & (1 << 7));
  }
  bool isChargeFetTestIsDischarging() const {
    return (status_ & (1 << 6));
  }
  bool isFullyCharged() const {
    return (status_ & (1 << 5));
  }
  bool isFullyDischarged() const {
    return (status_ & (1 << 4));
  }
  friend std::ostream & operator << (std::ostream & out, const BatteryStatus& status) {
    out << "Over charged alarm: " << (status.isOverChargedAlarmActive() ? "active" : "inactive") << std::endl;
    out << "Terminate charge alarm: " << (status.isTerminateChargeAlarmActive() ? "active" : "inactive") << std::endl;
    out << "Over temperature alarm: " << (status.isOverTemperatureAlarmActive() ? "active" : "inactive") << std::endl;
    out << "Terminate discharge alarm: " << (status.isTerminateDischargeAlarmActive() ? "active" : "inactive") << std::endl;
    out << "Remaining capacity alarm: " << (status.isRemainingCapacityAlarmActive() ? "active" : "inactive") << std::endl;
    out << "Remaining time alarm: " << (status.isRemainingTimeAlarmActive() ? "active" : "inactive") << std::endl;
    out << "Initialization: " << (status.isInitializationActive() ? "active" : "inactive") << std::endl;
    out << "FET Test: " << (status.isChargeFetTestIsDischarging() ? " Battery is discharging or at rest." : " Battery Battery is charging.") << std::endl;
    out << "Fully charged: " << (status.isFullyCharged() ? " Battery is fully charged" : " Battery is not fully charged") << std::endl;
    out << "Fully Discharged: " << (status.isFullyDischarged() ? " Battery is fully discharged" : " Battery is ok") << std::endl;
    return out;
  }

  Status status_;
};

} // namespace rpsm
