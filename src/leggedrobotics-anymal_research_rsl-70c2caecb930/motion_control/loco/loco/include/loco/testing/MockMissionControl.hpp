/**
 * @authors     Stephane Caron
 * @affiliation ANYbotics
 * @brief       Mission control mock for unit testing.
 */

#pragma once

// loco
#include "loco/mission_control/MissionControlBase.hpp"

namespace loco {

class MockMissionControl : public MissionControlBase {
 public:
  bool initialize(double /* dt */) {}

  bool advance(double /* dt */) {}

  //! Get desired base twist in control frame
  const Twist& getDesiredBaseTwistInControlFrame() const override { return desiredBaseTwistInControlFrame_; }

  //! Set desired base twist in control frame
  void setDesiredBaseTwistInControlFrame(const loco::Twist& twist) { desiredBaseTwistInControlFrame_ = twist; }

  //! Mock function
  const Twist& getMaximumBaseTwistInControlFrame() const override { return zeroTwist_; }

  //! Mock function
  const Pose& getMinimalPoseOffset() const override { return zeroPose_; };

  //! Mock function
  const Pose& getMaximalPoseOffset() const override { return zeroPose_; };

 protected:
  //! Desired base twist in control frame
  loco::Twist desiredBaseTwistInControlFrame_;

  //! Twist left to zero
  loco::Twist zeroTwist_;

  //! Pose left to zero
  loco::Pose zeroPose_;
};

}  // namespace loco
