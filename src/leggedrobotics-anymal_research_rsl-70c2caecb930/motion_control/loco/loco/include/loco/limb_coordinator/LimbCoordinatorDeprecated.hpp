/*!
 * @file     LimbCoordinatorDeprecated.hpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

#pragma once

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"

namespace loco {

//! The limb coordinator for dynamic gaits
/*! The limb coordinator updates the state of each leg based
 * on measurements (contact state) an plan (gait pattern).
 * The coordinator further decides if a leg is a support leg, i.e.
 * it supports the torso.
 */
class LimbCoordinatorDeprecated : public LimbCoordinatorBase {
 public:
  LimbCoordinatorDeprecated(WholeBody& wholeBody, GaitPatternBase& gaitPattern, bool isUpdatingStridePhase = true);
  ~LimbCoordinatorDeprecated() override = default;

  void setIsUpdatingStridePhase(bool isUpdatingStridePhase);
  bool isUpdatingStridePhase() const;

  bool initialize(double dt) override;

  /*! Advance in time
   * @param dt  time step [s]
   */
  bool advance(double dt) override;

  const GaitPatternBase& getGaitPattern() const;
  GaitPatternBase* getGaitPatternPtr() override;

  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Computes an interpolated version of the two controllers passed in as parameters.
   *  If t is 0, the current setting is set to limbCoordinator1, 1 -> limbCoordinator2, and values in between
   *  correspond to interpolated parameter set.
   * @param limbCoordinator1
   * @param limbCoordinator2
   * @param t interpolation parameter
   * @returns true if successful
   */
  bool setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2, double t) override;

  /*! Overrides stride phase from gait pattern.
   * @param stridePhase cycle phase in [0, 1]
   */
  void setStridePhase(double stridePhase);

 private:
  void printContactSchedule();

  WholeBody& wholeBody_;
  GaitPatternBase& gaitPattern_;

  bool isUpdatingStridePhase_;
  bool logLimbCoordinatorState_;
};

} /* namespace loco */
