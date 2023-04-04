/*
 * MissionControlSpeedFilter.hpp
 *
 *  Created on: Mar 7, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/mission_control/MissionControlBase.hpp"

// basic filters
#include <basic_filters/FilteredVariable.hpp>
#include <basic_filters/filters.hpp>

namespace loco {

class MissionControlSpeedFilter : public MissionControlBase {
 public:
  MissionControlSpeedFilter();
  ~MissionControlSpeedFilter() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  bool initialize(double dt) override;
  bool advance(double dt) override;

  bool setToInterpolated(const MissionControlBase& missionController1, const MissionControlBase& missionController2, double t) override;

  void setUnfilteredDesiredBaseTwistInControlFrame(const Twist& twist);
  const Twist& getDesiredBaseTwistInControlFrame() const override;

  const Twist& getMaximumBaseTwistInControlFrame() const override;
  virtual void setMaximumBaseTwistInControlFrame(Twist maximumBaseTwistInControlFrame);

  const Pose& getMinimalPoseOffset() const override;
  const Pose& getMaximalPoseOffset() const override;

  void setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(const Position& position);
  const Position& getFilteredPositionOffsetInWorldFrame() const;
  const Position& getMinimalPositionOffsetInWorldFrame() const;
  const Position& getMaximalPositionOffsetInWorldFrame() const;

  const RotationQuaternion& getFilteredDesiredOrientationOffset() const;
  void setUnfilteredDesiredOrientationOffset(const RotationQuaternion& desiredOrientationControlToBase);

  const EulerAnglesZyx& getMinimalDesiredOrientationOffset() const;
  const EulerAnglesZyx& getMaximalDesiredOrientationOffset() const;

  friend std::ostream& operator<<(std::ostream& out, const MissionControlSpeedFilter& speedFilter);

  //! Add variables to the signal logger.
  bool addVariablesToLog(const std::string& ns) const override;

 protected:
  Twist filteredBaseTwistInControlFrame_;
  Twist unfilteredBaseTwistInControlFrame_;
  Twist maximumBaseTwistInControlFrame_;

  Position filteredPositionOffsetInWorldFrame_;
  Position unfilteredPositionOffsetInWorldFrame_;

  Position minimalDesiredPositionOffsetInWorldFrame_;
  Position maximalDesiredPositionOffsetInWorldFrame_;

  RotationQuaternion filteredDesiredOrientationControlToBase_;
  RotationQuaternion unfilteredDesiredOrientationControlToBase_;
  EulerAnglesZyx minimalOrientationControlToBase_;
  EulerAnglesZyx maximalOrientationControlToBase_;

  Pose minimalPoseOffset_;
  Pose maximalPoseOffset_;

  //! filtered speeds [sagittal; coronal; turning]
  std::vector<basic_filters::FilteredDouble> filteredVelocities_;

  //! Feature toggle for rate limited speed
  bool enableRateLimiter_;
  //! Rate limited speeds [sagittal; coronal; turning]
  basic_filters::BrakingRateLimiterEigenVector3d rateLimiterVelocities_;

  //! filtered positions [heading; lateral; height]
  std::vector<basic_filters::FilteredDouble> filteredPositions_;

  //! filtered rotations [roll; pitch; yaw]
  std::vector<basic_filters::FilteredDouble> filteredRotations_;
};

} /* namespace loco */
