/*
 * StairsGeometry.hpp
 *
 *  Created on: Nov 24, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Free Gait
#include <free_gait_core/TypeDefs.hpp>

// STD
#include <string>
#include <memory>

namespace free_gait_stair_climbing {

//                                                   Nr 4 (last step nr)
//                                                +---------------------+
//                                                |
//     Front legs determine                       |
//     the current step number.            Nr 3   |
//                                      +---------+
//                     Run              |
//                  <-------->.         |
//                  .         .  Nr 2   |
//                  .         +---------+
//                  .         |
//                  .         |
//                  .  Nr 1   |
//                  +---------+....
//                  |             ^
//                  ^             | Rise
//            Nr 0  |z            |
// -----------------+-->.x........v
//              Stairs frame

using free_gait::Position;

class StairsGeometry
{
 public:
  StairsGeometry();
  StairsGeometry(const StairsGeometry& other);
  virtual ~StairsGeometry();
  StairsGeometry& operator=(const StairsGeometry& other);

  void setFrameId(const std::string& frameId);
  const std::string& getFrameId() const;
  void setNumberOfSteps(const size_t steps);
  unsigned int getNumberOfSteps() const;

  void setGeneralStepGeometry(const double rise, const double run);
  void setFirstStepGeometry(const double rise, const double run);
  void setLastStepGeometry(const double rise, const double run);

  double getRise(const size_t stepNumber) const;
  double getRun(const size_t stepNumber) const;
  Position getCenterPosition(const size_t stepNumber) const;

  struct Step
  {
    double rise_;
    double run_;
  };

 private:
  std::string frameId_;
  size_t nSteps_;
  Step generalStep_;
  std::unique_ptr<Step> firstStep_;
  std::unique_ptr<Step> lastStep_;
};

} /* namespace free_gait_stair_climbing */
