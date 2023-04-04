/*
 * FootprintGenerator.hpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Dario Bellicoso, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// loco
#include "loco/common/legs/Legs.hpp"
#include "loco/common/typedefs.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"

namespace loco {

class FootprintGenerator {
 public:
  FootprintGenerator();
  virtual ~FootprintGenerator() = default;

  bool update(const Legs& legs, const HeadingGenerator& headingGenerator);

  const Pose& getFootprintPose() const;
  const Vector& getFootprintHeadingDirectionInWorldFrame() const;

 private:
  Pose footprintPose_;
  Vector headingDirectionInWorldFrame_;
};

}  // namespace loco
