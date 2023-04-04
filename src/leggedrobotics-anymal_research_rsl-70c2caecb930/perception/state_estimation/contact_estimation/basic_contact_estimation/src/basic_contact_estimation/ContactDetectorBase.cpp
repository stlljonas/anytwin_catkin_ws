/*
 * ContactDetectorBase.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Christian Gehring
 */

#include "basic_contact_estimation/ContactDetectorBase.hpp"

namespace basic_contact_estimation {

ContactDetectorBase::ContactDetectorBase(std::string name) : name_(std::move(name)), state_(ContactState::OPEN), wrench_() {}

ContactDetectorBase::~ContactDetectorBase() = default;

}  // namespace basic_contact_estimation
