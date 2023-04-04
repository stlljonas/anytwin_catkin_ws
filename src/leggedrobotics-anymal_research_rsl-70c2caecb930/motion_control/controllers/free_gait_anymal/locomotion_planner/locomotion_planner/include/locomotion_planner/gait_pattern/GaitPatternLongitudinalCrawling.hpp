/*
 * GaitPatternLongitudinalCrawling.hpp
 *
 *  Created on: Mar 10, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/gait_pattern/GaitPatternBase.hpp"

#include <vector>
#include <map>

namespace locomotion_planner {

class GaitPatternLongitudinalCrawling : public GaitPatternBase
{
 public:
  GaitPatternLongitudinalCrawling();
  virtual ~GaitPatternLongitudinalCrawling();

  void setFirstStep(const LimbEnum& limb);
  void advance();
  const LimbEnum getCurrentLimb() const;
  bool startedNewCycle() const;
  bool endOfCycle() const;

  const std::vector<LimbEnum> getGaitPattern(const Direction& direction);

 private:

  bool isDirectionValid() const;

  //! Definition of the stepping sequence.
  std::map<Direction, std::vector<LimbEnum>> limbSequence_;

  //! Iterator to the current limb.
  std::vector<LimbEnum>::iterator currentLimb_;

};

} /* namespace locomotion_planner */
