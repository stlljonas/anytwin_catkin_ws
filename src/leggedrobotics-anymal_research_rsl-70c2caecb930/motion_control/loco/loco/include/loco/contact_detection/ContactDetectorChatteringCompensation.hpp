/*
 * ContactDetectorChatteringCompensation.hpp
 *
 *  Created on: Jun 15, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// loco
#include "loco/common/legs/Legs.hpp"
#include "loco/contact_detection/ContactDetectorBase.hpp"

// basic filters
#include "basic_filters/BinaryChatteringCompensator.hpp"

// Tiny XML
#include "tinyxml.h"

namespace loco {

class ContactDetectorChatteringCompensation : public ContactDetectorBase {
 public:
  explicit ContactDetectorChatteringCompensation(Legs& legs);
  ~ContactDetectorChatteringCompensation() override = default;
  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;

 private:
  Legs& legs_;
  std::vector<basic_filters::BinaryChatteringCompensator> contactFilters_;
  double timeBound_;
};

} /* namespace loco */
