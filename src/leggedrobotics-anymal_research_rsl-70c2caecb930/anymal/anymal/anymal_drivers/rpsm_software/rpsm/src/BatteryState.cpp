/*
 * BatteryMonitor.cpp
 *
 *  Created on: Feb 22, 2017
 *      Author: gech
 */

#include "rpsm/BatteryState.hpp"
#include <cmath>

namespace rpsm {

BatteryState::BatteryState(int numCells) : voltageFilter_(10.0f*60.0f, 0.01f)
{
  cellVoltages_.resize(numCells);


}

BatteryState::~BatteryState() {
}

void BatteryState::setVoltage(uint16_t voltage_mV) {
  const float current_voltage_V = static_cast<float>(voltage_mV) * 1.0e-3f; // Convert mV to V.
  voltage_V_ = voltageFilter_.filter(current_voltage_V);
}

void BatteryState::populateMsg(sensor_msgs::BatteryState& msg) {
   msg.voltage = voltage_V_;
   msg.current = current_A_;
//    msg.percentage = stateOfCharge_;

   msg.percentage = mapInRange(voltage_V_, 34.8f, 50.0f, 0.0f, 1.0f);


   msg.cell_voltage = cellVoltages_;
   msg.serial_number = std::to_string(serialnumber_);
//
//    if (batterystatus_.isChargeFetTestIsDischarging()) {
//      // is discharging or at rest
//       if (batterystatus_.isFullyCharged()) {
//         msg.power_supply_status = msg.POWER_SUPPLY_STATUS_FULL;
//       }
//       else {
//         msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING;
//       }
//    }
//    else {
//      msg.power_supply_status = msg.POWER_SUPPLY_STATUS_CHARGING;
//    }

   if (voltage_V_ > 50.0) {
     msg.power_supply_status = msg.POWER_SUPPLY_STATUS_FULL;
   }
   else if (current_A_  > -3.0) {
     msg.power_supply_status = msg.POWER_SUPPLY_STATUS_CHARGING;
   }
   else {
     msg.power_supply_status = msg.POWER_SUPPLY_STATUS_DISCHARGING;
   }


   if (safetystatus_.status_ == 0u) {
     msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_GOOD;
   }
   else if (safetystatus_.isOvertemperatureDuringCharge() || safetystatus_.isOvertemperatureDuringDischarge()) {
     msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERHEAT;
   }
   else if (safetystatus_.isCellOvervoltage() ) {
     msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
   }
   else {
     msg.power_supply_health = msg.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
   }

 }

} /* namespace rpsm */
