/*
 * ContactDetectorFeedThrough.h
 *
 *  Created on: May 26, 2014
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/contact_detection/ContactDetectorBase.hpp"

namespace loco {

class ContactDetectorFeedThrough : public ContactDetectorBase {
 public:
  ContactDetectorFeedThrough() = default;
  ~ContactDetectorFeedThrough() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;
};

} /* namespace loco */
