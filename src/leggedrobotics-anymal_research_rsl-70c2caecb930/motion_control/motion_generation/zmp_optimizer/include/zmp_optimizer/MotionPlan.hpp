/*
 * MotionPlan.hpp
 *
 *  Created on: 03.26, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// optimizer.
#include "zmp_optimizer/SupportPolygon.hpp"
#include "zmp_optimizer/VirtualPlaneFrame.hpp"
#include "zmp_optimizer/ZmpOptimizerObjectiveHandler.hpp"
#include "zmp_optimizer/ZmpParameterHandler.hpp"
#include "zmp_optimizer/zmp_optimizer.hpp"

// parameter handler.
#include <parameter_handler/parameter_handler.hpp>

// motion generation.
#include "motion_generation_utils/MotionPlanLinearAngular.hpp"
#include "motion_generation_utils/typedefs.hpp"

// loco.
#include "loco/gait_pattern/contact_schedule.hpp"

namespace zmp {

class MotionPlan : virtual public motion_generation::MotionPlanLinearAngular {
 public:
  MotionPlan();
  ~MotionPlan() override = default;

  bool initialize(double wholeBodyMass, const Eigen::Matrix3d& torsoInertiaTensorInBaseFrame,
                  const motion_generation::anymalLegsVector& vectorsTargetToLimbThighInBaseFrame, double nominalLegExtension);

  bool initialize(const MotionPlan& motionPlan);

  void setLineSearchOptions(const zmp::LineSearchOptions& lineSearchOptions);

  //! returns polygon Id at spline time
  unsigned int computeActivePolygonIdAtTime(double trajectoryTime) const;

  //! Returns zmp location w.r.t. the virtual plane in plane frame.
  bool getPlaneToZmpPositionInPlaneFrame(loco::Position& positionPlaneToZmpInPlaneFrame) const;
  bool getPlaneToZmpPositionInPlaneFrameAtTime(loco::Position& positionPlaneToZmpInPlaneFrame, double time) const;

  //! Returns zmp location w.r.t. the virtual plane in world frame.
  bool getWorldToZmpPositionInWorldFrame(loco::Position& positionWorldToZmpInWorldFrame) const;
  bool getWorldToZmpPositionInWorldFrameAtTime(loco::Position& positionWorldToZmpInWorldFrame, double time) const;

  //! Read access to private members
  const std::vector<SupportPolygon>& getSupportPolygonsInPlaneFrame() const;
  std::vector<SupportPolygon>* getSupportPolygonsInPlaneFramePtr();

  const PathRegularizer& getPathRegularizerInPlaneFrame() const;
  PathRegularizer* getPathRegularizerInPlaneFramePtr();

  const ComStateHandler& getComStateInPlaneFrame() const;
  ComStateHandler* getComStateInPlaneFramePtr();

  const Box& getComFinalBox() const;
  loco::ZmpParameterHandler* getZmpParamsPtr();
  const loco::ZmpParameterHandler& getZmpParams() const;

  //! Set virtual plane frame.
  void setVirtualPlaneFrame(const VirtualPlaneFrame& virtualPlaneFrame);

  //! Set support polygons (expressed in virtual plane frame).
  void setSupportPolygonsInPlaneFrame(const std::vector<SupportPolygon>& supportPolygonsInPlaneFrame, bool isFirstSupportPolygonNew);

  //! Copy parameters for active and desired gait.
  bool setParameters(const loco::ZmpParameterHandler& defaultZmpParams);

  void setComFinalBox(Box&& box);

  //! We copy as much information such that we are able to read motion plan and the zmp plan in world frame.
  void copyMotionPlan(const MotionPlan& motionPlan);

  //! Copy previous solution and update container time.
  bool copyComState(const MotionPlan& previousMotionPlan, double dt);

  //! Set container time of COM state and path regularier.
  bool setContainerTime(double containerTime);

  //! Read access private members.
  bool isFirstSupportPolygonNew() const;

  loco::Position getPositionWorldToComInWorldFrame() const;
  loco::LinearVelocity getLinearVelocityComInWorldFrame() const;
  loco::LinearAcceleration getLinearAccelerationComInWorldFrame() const;

  loco::Position getPositionWorldToComInWorldFrameAtTime(const double tk) const;
  loco::LinearVelocity getLinearVelocityComInWorldFrameAtTime(const double tk) const;
  loco::LinearAcceleration getLinearAccelerationComInWorldFrameAtTime(const double tk) const;

  loco::Position getPositionWorldToPathInWorldFrameAtTime(const double tk) const;
  loco::LinearVelocity getVelocityPathInWorldFrameAtTime(const double tk) const;

  loco::EulerAnglesZyx getAnglesZyxWorldToBase() const;
  loco::LocalAngularVelocity getAngularVelocityBaseInWorldFrame() const;
  loco::AngularAcceleration getAngularAccelerationBaseInWorldFrame() const;

  loco::EulerAnglesZyx getAnglesZyxWorldToBaseAtTime(const double tk) const;
  loco::LocalAngularVelocity getAngularVelocityBaseInWorldFrameAtTime(const double tk) const;
  loco::AngularAcceleration getAngularAccelerationBaseInWorldFrameAtTime(const double tk) const;

  loco::EulerAnglesZyx getAnglesZyxWorldToPathAtTime(const double tk) const;

  // Check constraints (use only for debugging).
  bool checkConstraints(double tol = 1e-5) const;

  //! Print motion plan to console (use only for debugging).
  void print() const;

  bool checkMotionPlan() const;

  const loco::RotationQuaternion& getOrientationWorldToControl() const noexcept;
  void setOrientations(const loco::RotationQuaternion& orientationWorldToControl);

  double getWholeBodyMass() const noexcept;
  Eigen::Matrix3d getTorsoInertiaTensorInBaseFrame() const noexcept;

  //! Returns the an array of vectors that connect target point with limb thigh.
  const motion_generation::anymalLegsVector& getVectorsTargetToLimbThighInBaseFrame() const;

  //! Set foothold-tracking offset indicator.
  void setFootholdTrackingOffsetIndicator(double footholdTrackingOffsetIndicator, loco::contact_schedule::LegEnumAnymal legEnum);

  //! Get foothold-tracking offset indicator.
  const motion_generation::anymalLegsDouble& getFootholdTrackingOffsetIndicator() const;

  //! Set current end-effector position in plane frame.
  void setPositionPlaneToEndEffectorInPlaneFrame(const Eigen::Vector3d& positionPlaneToEndEffectorInPlaneFrame,
                                                 const loco::contact_schedule::LegEnumAnymal& legEnum);

  //! Get current end-effector position in plane frame.
  const motion_generation::anymalLegsVector& getPositionPlaneToEndEffectorInPlaneFrame() const;

  //! Returns desired leg extension length.
  double getNominalLegExtension() const;

  //! Set gravity factor.
  void setGravityFactor(double gravityFactor);

  //! Returns gravity factor.
  double getGravityFactor() const noexcept;

  //! Add variables to the signal logger.
  void addVariablesToLog(const std::string& ns = "/zmp_optimizer/MotionPlan") const;

 protected:
  //! Returns zmp location w.r.t. the virtual plane in plane frame.
  bool getPlaneToZmpPositionInPlaneFrame(Eigen::Vector3d& positionPlaneToZmpInPlaneFrame,
                                         const loco::Position& positionPlaneToComInPlaneFrame,
                                         const loco::LinearAcceleration& linearAccelerationComInPlaneFrame,
                                         const loco::EulerAnglesZyx& eulerAnglesZyxBaseToPlane,
                                         const loco::EulerAnglesZyxDiff& eulerAnglesZyx_dot,
                                         const loco::EulerAnglesZyxDiff& eulerAnglesZyx_ddot) const;

  //! A list of support polygons in plane frame.
  std::vector<SupportPolygon> supportPolygonsInPlaneFrame_;

  //! Path regularizer (High level approximation of CoG trajectory)
  PathRegularizer pathRegularizerInPlaneFrame_;

  //! Optimized CoG motion plan.
  ComStateHandler comStateInPlaneFrame_;

  //! Final box constraints for cog (stored for visualization only).
  Box comFinalBox_;

  //! Container for all gait-varying parameters.
  loco::ZmpParameterHandler zmpParams_;

  //! True if this motion plan starts with a new support polygon (relative to the previous motion plan).
  bool isFirstSupportPolygonNew_;

  //! Orientation world to control (for visualization).
  loco::RotationQuaternion orientationWorldToControl_;

  //! Mass of whole body (torso + limbs)
  double wholeBodyMass_;

  //! Interia of the toros given in base frame.
  Eigen::Matrix3d torsoInertiaTensorInBaseFrame_;

  //! The gravity vector in world frame.
  Eigen::Vector3d gravityVectorInWorldFrame_;

  //! Offset target point to limb thigh.
  motion_generation::anymalLegsVector vectorsTargetToLimbThighInBaseFrame_;

  //! Filtered position error between grounded end-effector and desired foothold.
  motion_generation::anymalLegsDouble footholdTrackingOffsetIndicator_;

  //! Current footprint pattern.
  motion_generation::anymalLegsVector positionPlaneToEndEffectorInPlaneFrame_;

  //! desired leg extension length.
  double nominalLegExtension_;

  //! If 1, robot (approximated as inverted pendulum), will align with gravity vector.
  //! If 0, robot will align with local terrain normal.
  //! If value is in (0,1), the projection vector will be linearly interpolated.
  double gravityFactor_;
};

} /* namespace zmp */
