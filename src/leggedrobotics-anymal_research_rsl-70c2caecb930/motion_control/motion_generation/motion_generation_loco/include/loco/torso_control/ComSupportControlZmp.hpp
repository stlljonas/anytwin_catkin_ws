/*!
* @file     ComSupportControlZmp.hpp
* @author   C. Dario Bellicoso
* @date     Oct 7, 2014
* @brief
*/

#pragma once

// loco
#include "loco/torso_control/ComSupportControlBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/planner/TorsoPlannerZmp.hpp"
#include "loco/common/loco_common.hpp"
#include "loco/heading_generation/HeadingGenerator.hpp"
#include "loco/torso_control/com_support_control_zmp.hpp"
#include "loco/torso_control/TerrainAdapter.hpp"

// motion generation.
#include "motion_generation/ContactScheduleZmp.hpp"
#include "motion_generation_utils/typedefs.hpp"

// zmp optimizer
#include "zmp_optimizer/VirtualPlaneFrame.hpp"
#include "zmp_optimizer/zmp_optimizer.hpp"
#include "zmp_optimizer/MotionPlan.hpp"
#include "zmp_optimizer/SupportPolygon.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// basic filters
#include "basic_filters/filters.hpp"

// curves
#include "curves/PolynomialSplineContainer.hpp"

// boost
#include <boost/thread.hpp>

namespace loco {

class ComSupportControlZmp: public ComSupportControlBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using positionVector = std::vector<Position, Eigen::aligned_allocator<Position>>;
  using velocityVector = std::vector<LinearVelocity, Eigen::aligned_allocator<LinearVelocity>>;
  using accelerationVector = std::vector<LinearAcceleration, Eigen::aligned_allocator<LinearAcceleration>>;
  using eulerAnglesZyxVector = std::vector<EulerAnglesZyx, Eigen::aligned_allocator<EulerAnglesZyx>>;
  using eulerAnglesZyxDiffVector = std::vector<EulerAnglesZyxDiff, Eigen::aligned_allocator<EulerAnglesZyxDiff>>;

  using anymalLegsBool = std_utils::EnumArray<contact_schedule::LegEnumAnymal, bool>;
  using anymalLegsDouble = std_utils::EnumArray<contact_schedule::LegEnumAnymal, double>;
  using anymalLegsPosition = std_utils::EnumArray<contact_schedule::LegEnumAnymal, Position>;
  using anymalLegsVector = std_utils::EnumArray<contact_schedule::LegEnumAnymal, Eigen::Vector3d>;

  ComSupportControlZmp(WholeBody& wholeBody,
                       ContactScheduleZmp& contactSchedule,
                       HeadingGenerator& headingGenerator,
                       TerrainModelBase& terrain,
                       TerrainAdapter& terrainAdapter,
                       bool isRealRobot = false);
  ~ComSupportControlZmp() override = default;

  bool advance(double dt) override;
  bool initialize(double dt) override;
  bool loadParameters(const TiXmlHandle &hParameterSet) override;
  bool stop() override;
  bool setToInterpolated(const ComSupportControlBase& supportPolygon1, const ComSupportControlBase& supportPolygon2, double t) override;

  //! Linear Control signals.
  // ToDo: rename functions in the base class!!!
  virtual const Position& getPositionWorldToDesiredCoMInWorldFrame() const;
  virtual const LinearVelocity& getLinearVelocityDesiredBaseInWorldFrame() const;
  const LinearAcceleration& getLinearAccelerationDesiredTargetInControlFrame() const;

  //! Angular control signals.
  const RotationQuaternion& getOrientationControlToDesiredBase() const;
  const LocalAngularVelocity& getAngularVelocityDesiredBaseInControlFrame() const;
  const AngularAcceleration& getAngularAccelerationDesiredBaseInControlFrame() const;

  //! Get a reference to an object which represents the whole body.
  const WholeBody& getWholeBody() const;

  //! Get a pointer to an object which represents the whole body.
  WholeBody* getWholeBodyPtr() const;

  //! Add variables to the signal logger.
  bool addVariablesToLog(const std::string& ns = "/motion_generation_loco/ComSupportControlZmp") const override;

  //! Add variables to the parameter handler.
  bool addParametersToHandler(const std::string& ns) override;

  //! Remove variables from the parameter handler.
  bool removeParametersFromHandler() override;

  //! Get a reference to an object which represents the torso motion planner.
  const TorsoPlannerZmp& getTorsoPlannerZmp() const;

  //! Get a pointer to an object which represents the torso motion planner.
  TorsoPlannerZmp* getTorsoPlannerZmpPtr();

  //! Copy the motion plane. Note: this method is thread safe.
  void getMotionPlan(zmp::MotionPlan& motionPlan) const;

  //! Copy the pose from the motion plan. Note: this method is thread safe.
  void getPosePlaneToWorldFromMotionPlan(Pose& posePlaneToWorld) const;

  //! Copy the path regularizer from the motion plan. Note: this method is thread safe.
  void getPathRegularizerFromMotionPlan(zmp::PathRegularizer& pathRegularizerInPlaneFrame) const;

  //! Copy the cog trajectory from the motion plan. Note: this method is thread safe.
  void getCogTrajectroyFromMotionPlan(zmp::ComStateHandler& cogTrajectoryInPlaneFrame) const;

  //! Get position of limb thigh at time.
  //! This function uses the path regularizer to make a prediction.
  void getPredictedThighPositionUsingPathRegularizer(
      const contact_schedule::LegEnumAnymal& legId,
      double time,
      Position& positionWorldToLimbThighAtTimeInWorldFrame) const;

  //! Helper function for visualization.
  bool isFirstSupportPolygonNew();

  //! Set desired height above ground on flat terrain.
  void setDesiredHeightAboveGroundOnFlatTerrain(double desiredHeightAboveGroundOnFlatTerrain) noexcept;

  //! Returns nominal desired height above ground on flat terrain.
  double getNominalDesiredHeightAboveGroundOnFlatTerrain() const noexcept;

  /*! Get reference to terrain adapter.
   *
   * @return terrainAdapter Reference to internal terrain adapter.
   *
   */
  const TerrainAdapter& getTerrainAdapter() const {
    return terrainAdapter_;
  }

 protected:
  //! Update virtual plane frame after receiving a new CoM motion plan
  bool updateVirtualPlaneFrame();

  //! Create a list of support polygons based on information from the contact schedule.
  virtual bool computeSupportPolygonSequence(int& polygonIdAtSwitch, bool& isFirstSupportPolygonNew);

  //! Cog motion plan is replaced with splines that connects the current cog state
  // (i.e., position, linear velocity, rotation, angular velocity)
  // with the footprint center at zero rotation, zero linear and angular velocity.
  // The time given to reach the final point is equal to the stride duration of the gait
  // but is at least 0.1 seconds.
  virtual bool resetZmpTrajectory(double dt);

  //! set up dynamic motion data (cog state, rotations, ...)
  virtual bool setMotionPlan(zmp::MotionPlan& optimizedMotionPlan);

  //! Call the motion optimizer.
  bool planCenterOfMassMotion(double dt);

  // Computes unoptimized control signals.
  bool setTrivialControlSignals(double timeToAdvance);

  //! Stop and re-initialize planner.
  bool stopPlanner(double dt);

  //! Alpha filter that merges measurements with reference values for position and velocity.
  bool alphaFilterForInitState(
      Position& position,       const Position& referencePosition,
      LinearVelocity& velocity, const LinearVelocity& referenceVelocity,
      double weightRef) const;

  //! Alpha filter that merges measurements with reference values for angular position and angular velocity.
  bool alphaFilterForInit(
      EulerAnglesZyx& angleZyx,         const EulerAnglesZyx& referenceAngleZyx,
      EulerAnglesZyxDiff& angularRates, const EulerAnglesZyxDiff& referenceAngularRates,
      double weightRef) const;

  //! Add support polygon for current feetconfiguration.
  void emplaceSupportPolygon(
      const anymalLegsPosition& feetConfigurationInWorldFrame,
      double timeLeftInPolygon);

  //! Read position, velocity and acceleration from the motion plan at the current time.
  bool readMotionPlan();

  //! Update motion plan time and set new control signals.
  bool processReferenceSignals(double timeToAdvance, double dt);

  //! get target measurements in plane frame.
  bool getMeasuredTorsoTargetStateInPlaneFrame(
      Position& positionPlaneToMeasuredTargetInPlaneFrame,
      LinearVelocity& linearVelocityMeasuredTargetInPlaneFrame) const;

  bool getMeasuredTorsoTargetRotationInPlaneFrame(
      EulerAnglesZyx& anglesZyxBaseToPlane,
      EulerAnglesZyxDiff& angularRatesZyxBaseInPlaneFrame) const;

  //! get target values in plane frame from previous optimization.
  bool getReferenceTorsoTargetStateInPlaneFrame(
      Position& refPositionPlaneToTargetInPlaneFrame,
      LinearVelocity& refVelocityTargetInPlaneFrame,
      LinearAcceleration& refAccelerationTargetInPlaneFrame) const;

  bool getReferenceTorsoTargetStateInPlaneFrame(
      EulerAnglesZyx& anglesZyxRefBaseToPlane,
      EulerAnglesZyxDiff& refAngularRatesZyxRefBaseInPlaneFrame,
      EulerAnglesZyxDiff& refAngularAccelZyxRefBaseInPlaneFrame) const;

  //! Check if the robot is in flight phase (no legs are grounded)
  bool isFlightPhase() const;

  //! True if leg is in support or contact-invariant mode.
  bool isLegGrounded(unsigned int legId) const;

  virtual bool computeVelocityProjectionInPlaneFrame(
      Position& desiredVelocityProjectionInPlaneFrame,
      LinearVelocity& desiredLinearVelocityTargetInPlaneFrame,
      LinearVelocity& desiredLinearVelocityAtVelocityProjectionInPlaneFrame,
      LinearAcceleration& desiredLinearAccelerationTargetInPlaneFrame,
      LinearAcceleration& desiredLinearAccelerationAtVelocityProjectionInPlaneFrame,
      double optimizationHorizonInSeconds);

    // Debugging tool: print info about the support polygon sequence
  void printSupportPolygonSequence(double rate = 0.05) const;

  virtual bool computePositionWorldToReferencePathInWorldFrame(
      Position& positionWorldToReferencePathInWorldFrame) const;

  virtual bool computePathKnotsInPlaneFrame(
      positionVector& positionKnotInPlaneFrame,
      velocityVector& velocityKnotInPlaneFrame,
      accelerationVector& accelerationKnotInPlaneFrame,
      std::vector<double>& duration,
      double optimizationHorizonInSeconds,
      Position& desiredVelocityProjectionInPlaneFrame);

  bool computePathKnotsInPlaneFrame(
      eulerAnglesZyxVector& anglesZyxPathToPlane,
      eulerAnglesZyxDiffVector& angularRatesZyxPathInPlaneFrame,
      std::vector<double>& duration,
      double optimizationHorizonInSeconds,
      EulerAnglesZyx& orientationDesiredVelocityOffset);

  //! Roll and pitch from control frame, yaw from heading direction of current footprint orientation.
  RotationQuaternion getOrientationFootprintHeadingToPlane() const;

  //! use current endeffector location to compute feet configuration.
  void setCurrentFeetConfigurationInWorldFrame();

  //! Returns desired approach duration to achieve rest position.
  double computeApproachDuration() const;

  //! Updates foothold tracking offset indicator.
  bool updateFootholdTrackingOffsetIndicator();

  //! A reference to the whole body.
  WholeBody& wholeBody_;

  //! A reference to the gait pattern.
  ContactScheduleZmp& contactSchedule_;

  //! A reference to the heading generator.
  const HeadingGenerator& headingGenerator_;

  //! A reference to the terrain.
  const TerrainModelBase& terrain_;

  //! If true, variables are added to signal logger.
  bool enableLogging_;

  //! A container for the motion plan.
  zmp::MotionPlan motionPlan_;

  //! A container for the previous motion plan.
  zmp::MotionPlan motionPlanPrevious_;

  //! A mutex to synchronize access to the motion plan.
  mutable boost::shared_mutex mutexMotionPlan_;

  //! The motion optimizer.
  std::unique_ptr<TorsoPlannerZmp> zmpPlanner_;

  //! The reference acceleration of the main body computed in the control frame.
  LinearAcceleration linearAccelerationDesiredTargetInControlFrame_;

  //! The reference acceleration of the main body computed in the world frame.
  LinearAcceleration linearAccelerationDesiredTargetInWorldFrame_;

  //! The reference orientation of the main body computed in control frame
  RotationQuaternion orientationControlToDesiredBase_;

  // The reference angular velocity of the main body in computed  control frame
  LocalAngularVelocity angularVelocityDesiredBaseInControlFrame_;

  //! The reference angular acceleration of the main body computed in control frame
  AngularAcceleration angularAccelerationDesiredBaseInControlFrame_;

  //! If leg is grounded, the vector element specifies that location.
  // If the leg is swinging, the vector element specifies the desired touch-down position.
  anymalLegsPosition feetConfigurationInWorldFrame_;

  //! A container of support polygons associated to the current contact schedule. The polygons correspond to the previous successful optimization.
  std::vector<zmp::SupportPolygon> supportPolygons_;

  //! Polygons are skipped if phase is smaller than this threshold.
  double polygonTimeThreshold_;

  //! Toggle printouts.
  bool verbose_;

  //! The location of the ZMP in world frame. This is used to store the ZMP location and log it.
  Position positionWorldToZmpInWorldFrame_;

  //! If leg at index legId is stance, then the following vector will be true at legId
  anymalLegsBool isStancedLegId_;

  //! Object that handles the virtual plane frame
  zmp::VirtualPlaneFrame virtualPlaneFrame_;

  //! First order filter to compute average real time discrete time step.
  basic_filters::FirstOrderFilterD averageRealTimeDt_; // ToDo: remove

  //! A watchdog timer.
  std_utils::HighResolutionClockTimer realTimeDt_;

  //! ZMP parameters as read from the config file (these can not be modified online).
  ZmpParameterHandler defaultZmpParams_;

  //! True if physical robot is used, false if simulation is used.
  const bool isRealRobot_;

  //! Desired torso height in world frame on flat ground.
  parameter_handler::Parameter<double> desiredHeightAboveGroundOnFlatTerrain_;

  //! Desired nominal torso height in world frame on flat ground.
  double nominalDesiredHeightAboveGroundOnFlatTerrain_;

  //! Weights for alpha filters.
  double weightPreviousInitState_;

  //! First phase event of the previous optimization.
  std::vector<contact_schedule::Event> previousFirstPhaseEvent_;

  //! Helper variable for visualization (resolves a timing).
  bool isFirstSupportPolygonNewPin_;

  //! True if a motion plan is available.
  bool isOptimizedMotionPlanAvailable_;

  // Method to advance the splines.
  com_support::AdvanceMethod advanceMethod_;

  // Parameters for interpolating previous motion plan with current one.
  unsigned int numOfInterpolationSamples_;
  unsigned int actualInterpolationSample_;

  //! Helper variable to kill optimization.
  bool didPinSafety_;

  //! Filter to smooth velocity measurement used for velocity projection.
  basic_filters::FirstOrderFilter<LinearVelocity> filterVelocityProjection_;

  //! Weight in interval 0 (robot is not following the disturbance) and 1 (robot is following any disturbance).
  double weightDisturbanceFollower_;

  //! Degrees of freedom which are optimized.
  std::vector<zmp::CogDim> optimizationDofs_;

  //! Function of position error between grounded end-effector and desired foothold.
  anymalLegsDouble footholdTrackingOffsetIndicator_;

  //! Low pass filtered foothold tracking offset indicator.
  std_utils::EnumArray<contact_schedule::LegEnumAnymal, basic_filters::FirstOrderFilter<double>> filteredFootholdTrackingOffsetIndicator_;

  //! Previous desired foothold in world frame.
  anymalLegsPosition positionWorldToPreviousDesiredFootholdInWorldFrame_;

  //! Parameters to compute foothold tracking indicator.
  double nominalFootholdTrackingOffset_;
  double footholdTrackingOffsetGain_;

  //! Heading vector in the inertial frame.
  Vector headingInWorldFrame_;

  //! Time in [s] spent in Standing locomotion state. See comments in computePathKnotsInPlaneFrame.
  double timeSpentStanding_;

  //! Time in [s] during which the base yaw target progressively switches to the one from the heading generator.
  double switchToStanceDuration_;

  //! Terrain adaption dictates how the torso adapts to terrain variations.
  TerrainAdapter& terrainAdapter_;

  //! Initial position of the path regularizer in world frame.
  Position positionWorldToPathRegularizerInWorldFrame_;

  //! Initial linear velocity of the path regularizer in the world frame.
  LinearVelocity linearVelocityPathRegularizerInWorldFrame_;
};

} /* namespace loco */
