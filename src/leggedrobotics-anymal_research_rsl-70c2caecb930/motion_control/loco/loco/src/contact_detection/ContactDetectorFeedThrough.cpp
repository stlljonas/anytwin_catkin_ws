/*
 * ContactDetectorFeedThrough.cpp
 *
 *  Created on: May 26, 2014
 *      Author: Christian Gehring
 */

// loco
#include "loco/contact_detection/ContactDetectorFeedThrough.hpp"

namespace loco {

bool ContactDetectorFeedThrough::initialize(double dt) {
  return true;
}

bool ContactDetectorFeedThrough::advance(double dt) {
  return true;
}

bool ContactDetectorFeedThrough::loadParameters(const TiXmlHandle& handle) {
  return true;
}

} /* namespace loco */
