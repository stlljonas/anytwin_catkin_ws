/*
 * BatteryMonitor.hpp
 *
 *  Created on: Feb 22, 2017
 *      Author: gech
 */

#pragma once

#include <ostream>
#include <sensor_msgs/BatteryState.h>
#include <rpsm/BatteryStatus.hpp>
#include <rpsm/BatterySafetyStatus.hpp>

namespace rpsm {
template <typename T>
class ExponentiallyWeightedMovingAverageFilter {
 public:
  ExponentiallyWeightedMovingAverageFilter(T timeConstant, T samplingTime) {
    timeConstant_ = timeConstant;
    samplingTime_ = samplingTime;
    alpha_ = T(1.0) - std::exp(-samplingTime_/timeConstant_);
  }
  ~ExponentiallyWeightedMovingAverageFilter() {

  }

  T filter(T input) {
    if (!initialized_) {
      initialized_ = true;
      previousValue_ = input;
      return input;
    }
    return input*alpha_ + (T(1.0) - alpha_)*previousValue_;
  }
 protected:
  T timeConstant_ = T(1.0);
  T samplingTime_ = T(1.0);
  T alpha_ = T(1.0);
  bool initialized_ = false;
  T previousValue_;
};


class BatteryState {
 public:
  BatteryState(int numCells);
  virtual ~BatteryState();
  void setBatteryStatus(BatteryStatus::Status status) {
    batterystatus_.status_ = status;
  }

  void setSafetyStatus(uint32_t status) {
    safetystatus_.status_ = status;
  }
  void setSerialnumber(uint16_t serialnumber) {
    serialnumber_ = serialnumber;
  }

  void setTemperature(uint16_t temperature) {
    temperature_ = temperature;
  }
  void setVoltage(uint16_t voltage_mV);

  void setCurrent(int16_t current_mA) {
    current_A_ = static_cast<float>(current_mA) * 1.0e-3f; // Convert mA to A.
  }

  void setStateOfCharge(uint8_t state) {
    stateOfCharge_ = static_cast<float>(state) * 1.0e-2f; // Convert % to 0-1 range
  }

  void setCellVoltage(int index, uint16_t voltage_mV) {
    cellVoltages_[index] = static_cast<float>(voltage_mV);
  }

  template<typename T>
  T mapInRange (T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  void populateMsg(sensor_msgs::BatteryState& msg);

  friend std::ostream & operator << (std::ostream & out, const BatteryState& monitor) {
    out << "Battery status: " << std::endl;
    out << monitor.batterystatus_<< std::endl;
    out << "Safety status: " << std::endl;
    out << monitor.safetystatus_ << std::endl;
    out << "Temperature: " << monitor.temperature_ << " deg C"<< std::endl;
    return out;
  }

 public:
  BatteryStatus batterystatus_; /*< BMS info*/
  BatterySafetyStatus safetystatus_;
  float temperature_ = 0.0f;
  float voltage_V_ = 48.0f; // 40 to accelerate filter convergence at startup
  float current_A_ = 0.0f;
  std::vector<float> cellVoltages_;
  float stateOfCharge_ = 1.0;
  uint16_t serialnumber_ = 0u;


  ExponentiallyWeightedMovingAverageFilter<float> voltageFilter_;
};

} /* namespace rpsm */
