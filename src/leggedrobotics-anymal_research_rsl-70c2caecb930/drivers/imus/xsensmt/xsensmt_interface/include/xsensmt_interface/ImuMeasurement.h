/** \file: ImuMeasurement.h
    \brief: defines custom struct that holds IMU measurement data
  */

#ifndef IMU_MEASUREMENT_H
#define IMU_MEASUREMENT_H

#include <memory>
#include <string>
#include <utility>
#include <cstdint>
#include <chrono>

struct ImuMeasurement {
	std::chrono::steady_clock::time_point timestamp_;
	uint16_t counter_;
	double linearAcceleration_[3];
	double angularVelocity_[3];
	bool triggerIndicator_ = false;
};



#endif
