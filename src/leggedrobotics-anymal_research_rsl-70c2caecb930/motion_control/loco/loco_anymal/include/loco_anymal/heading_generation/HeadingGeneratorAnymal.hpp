/*
 * HeadingGeneratorAnymal.hpp
 *
 *  Created on: Jan 26, 2017
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/heading_generation/HeadingGenerator.hpp"

// loco anymal
#include <loco/common/WholeBody.hpp>

namespace loco_anymal {

class HeadingGeneratorAnymal : public loco::HeadingGenerator {
 public:
  explicit HeadingGeneratorAnymal(const loco::WholeBody& wholeBody);
  ~HeadingGeneratorAnymal() override = default;

  void getTorsoHeadingDirectionInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const override;
  void getLegsHeadingDirectionFromCurrentFeetInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const override;
  void getLegsHeadingDirectionFromLastContactsInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const override;
  void getLegsHeadingDirectionFromPlannedContactsInWorldFrame(loco::Vector& headingDirectionInWorldFrame) const override;

  bool computeLastFootPrintCenterInWorldFrame(loco::Position& footPrintCenterInWorldFrame) const override;
  bool computeCurrentFootPrintCenterInWorldFrame(loco::Position& footPrintCenterInWorldFrame) const override;
  bool computePlannedFootPrintCenterInWorldFrame(loco::Position& footPrintCenterInWorldFrame) const override;

 protected:
  const loco::WholeBody& wholeBody_;
};

} /* namespace loco */
