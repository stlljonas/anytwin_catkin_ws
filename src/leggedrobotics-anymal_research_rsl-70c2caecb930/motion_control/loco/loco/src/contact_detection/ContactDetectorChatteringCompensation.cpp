/*
 * ContactDetectorChatteringCompensation.hpp
 *
 *  Created on: Jun 15, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// loco
#include "loco/contact_detection/ContactDetectorChatteringCompensation.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace color = message_logger::color;

namespace loco {

ContactDetectorChatteringCompensation::ContactDetectorChatteringCompensation(Legs& legs)
    : ContactDetectorBase(), legs_(legs), contactFilters_(), timeBound_(0.1) {}

bool ContactDetectorChatteringCompensation::initialize(double dt) {
  contactFilters_.resize(legs_.size());
  for (auto leg : legs_) {
    contactFilters_[leg->getId()].initialize(timeBound_);
  }
  return true;
}

bool ContactDetectorChatteringCompensation::advance(double dt) {
  for (auto leg : legs_) {
    bool state = contactFilters_[leg->getId()].advance(dt, leg->getContactSchedulePtr()->isGrounded());
    leg->getContactSchedulePtr()->setIsGrounded(state);
  }
  return true;
}

bool ContactDetectorChatteringCompensation::loadParameters(const TiXmlHandle& handle) {
  TiXmlElement* element;

  TiXmlHandle cdHandle(handle.FirstChild("ContactDetector"));
  element = cdHandle.Element();
  if (element == nullptr) {
    printf("Could not find ContactDetector\n");
    std::cout << color::magenta << "[ContactDetectorChatteringCompensation/loadParameters] " << color::red << "Warning: " << color::blue
              << "Could not find section 'ContactDetector'" << color::def << std::endl;
  }

  TiXmlElement* child = cdHandle.FirstChild().ToElement();
  for (; child != nullptr; child = child->NextSiblingElement()) {
    if (child->ValueStr() == "ChatteringCompensation") {
      if (child->QueryDoubleAttribute("timeBound", &timeBound_) != TIXML_SUCCESS) {
        std::cout << color::magenta << "[ContactDetectorChatteringCompensation/loadParameters] " << color::red << "Warning: " << color::blue
                  << "Could not find parameter 'timeBound_' in section 'ChatteringCompensation'. Setting to color::default value: "
                  << color::red << timeBound_ << color::def << std::endl;
      } else {
        std::cout << color::magenta << "[ContactDetectorChatteringCompensation/loadParameters] " << color::blue
                  << "TimeBound is set to: " << color::red << timeBound_ << color::def << std::endl;
      }
    }
  }

  return true;
}

} /* namespace loco */
