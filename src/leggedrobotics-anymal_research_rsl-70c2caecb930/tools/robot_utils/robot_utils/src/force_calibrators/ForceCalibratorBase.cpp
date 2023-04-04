/*
 * ForceCalibratorBase.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: gech
 */

#include "robot_utils/force_calibrators/ForceCalibratorBase.hpp"

namespace robot_utils {

ForceCalibratorBase::ForceCalibratorBase(std::string name) : name_(std::move(name)) {}

}  // namespace robot_utils
