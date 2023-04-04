/*
 * GaitPatternStaticGait.hpp
 *
 *  Created on: Feb 3, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/gait_pattern/GaitPatternFlightPhases.hpp"

// basic filters
#include "basic_filters/filters.hpp"

namespace loco {

class GaitPatternStaticGait : public GaitPatternFlightPhases {
 public:
  explicit GaitPatternStaticGait(WholeBody& wholeBody);
  ~GaitPatternStaticGait() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  virtual double getDiagonalStanceDuration() const;
  virtual double getLateralStanceDuration() const;
  virtual double getSwingDuration() const;
  virtual double getDiagonalStanceDurationSeconds() const;
  virtual double getLateralStanceDurationSeconds() const;
  virtual double getSwingDurationSeconds() const;

  bool advance(double dt) override;

  void setUseGaitPatternFw(bool setGaitPattern);

  const std::vector<int>& getSwingLegIndexes() const;
  virtual void updateFootfallPattern();

  virtual int getIndexOfSwingLeg() const;
  virtual int getIndexOfSwingLeg(double stridePhase) const;
  virtual int getIndexOfNextSwingLeg(const int currentSwingFoot) const;
  virtual int getIndexOfPreviousSwingLeg(const int currentSwingFoot) const;

  bool isFullStancePhase() const;

  bool addVariablesToLog(bool update) override;

  bool initialize(double dt) override;

  virtual int getNextSwingLeg() const;
  virtual int getLastSwingLeg() const;
  virtual int getOverNextSwingLeg() const;
  virtual int getCurrentSwingLeg() const;

  virtual void resetSwingIndexes(bool wasStanding);
  virtual void updateSwingLegsIndexes();

  friend std::ostream& operator<<(std::ostream& out, const GaitPatternStaticGait& gaitPattern);

 protected:
  double stanceDiagonalDuration_;
  double stanceLateralDuration_;
  double swingDuration_;

  double stanceDiagonalDurationInit_;
  double stanceLateralDurationInit_;
  double strideDurationInit_;

  double stanceDiagonalDurationMin_;
  double stanceLateralDurationMin_;
  double strideDurationMin_;

  double maxHeadingVelocity_;

  basic_filters::FirstOrderFilterD headingVelocityFilter_;

  bool useGaitPatternFw_;

  std::vector<int> swingLegIndexes_;

  int swingLegIndexNow_;
  int swingLegIndexLast_;
  int swingLegIndexNext_;
  int swingLegIndexOverNext_;
};

} /* namespace loco */
