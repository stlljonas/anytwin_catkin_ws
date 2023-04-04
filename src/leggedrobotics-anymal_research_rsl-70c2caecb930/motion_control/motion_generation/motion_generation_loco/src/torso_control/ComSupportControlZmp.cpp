/*!
* @file     ComSupportControlZmp.cpp
* @author   C. Dario Bellicoso
* @date     Oct 7, 2014
* @brief
*/

// anymal_description
#include <anymal_description/LegEnum.hpp>

// loco
#include "loco/torso_control/ComSupportControlZmp.hpp"

// motion_generation_utils
#include <motion_generation_utils/motion_generation.hpp>

// std utils
#include "std_utils/std_utils.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// boost
#include <boost/thread.hpp>
#include <boost/assert.hpp>

// stl
#include <mutex>

// ToDO: split this class into parts

using namespace message_logger::color;

namespace {

constexpr double MAX_SWITCH_TO_STANCE_DURATION = 100.;  // [s]
constexpr double MIN_SWITCH_TO_STANCE_DURATION = 0.001; // [s]

}

namespace loco {

ComSupportControlZmp::ComSupportControlZmp(
    WholeBody& wholeBody,
    ContactScheduleZmp& contactSchedule,
    HeadingGenerator& headingGenerator,
    TerrainModelBase& terrain,
    TerrainAdapter& terrainAdapter,
    bool isRealRobot)
    : ComSupportControlBase(*wholeBody.getLegsPtr()),
      wholeBody_(wholeBody),
      contactSchedule_(contactSchedule),
      headingGenerator_(headingGenerator),
      terrain_(terrain),
      enableLogging_(false),
      motionPlan_(),
      motionPlanPrevious_(),
      mutexMotionPlan_(),
      zmpPlanner_(new TorsoPlannerZmp()),
      linearAccelerationDesiredTargetInControlFrame_(),
      linearAccelerationDesiredTargetInWorldFrame_(),
      orientationControlToDesiredBase_(),
      angularVelocityDesiredBaseInControlFrame_(),
      angularAccelerationDesiredBaseInControlFrame_(),
      feetConfigurationInWorldFrame_(Position::Zero()),
      supportPolygons_(),
      polygonTimeThreshold_(0.05),
      verbose_(false),
      positionWorldToZmpInWorldFrame_(),
      isStancedLegId_(true),
      virtualPlaneFrame_(zmp::vpf::VirtualPlaneFrameEnum::controlFrame),
      averageRealTimeDt_(),
      realTimeDt_(),
      defaultZmpParams_(),
      isRealRobot_(isRealRobot),
      desiredHeightAboveGroundOnFlatTerrain_(0.47, 0.25, 0.65),
      nominalDesiredHeightAboveGroundOnFlatTerrain_(0.47),
      weightPreviousInitState_(0.2),
      previousFirstPhaseEvent_(),
      isFirstSupportPolygonNewPin_(false),
      isOptimizedMotionPlanAvailable_(false),
      advanceMethod_(com_support::AdvanceMethod::Undefined),
      numOfInterpolationSamples_(0u),
      actualInterpolationSample_(0u),
      didPinSafety_(false),
      filterVelocityProjection_(),
      weightDisturbanceFollower_(0.0),
      optimizationDofs_(),
      footholdTrackingOffsetIndicator_(0.0),
      positionWorldToPreviousDesiredFootholdInWorldFrame_(Position::Zero()),
      filteredFootholdTrackingOffsetIndicator_(),
      nominalFootholdTrackingOffset_(0.04),
      footholdTrackingOffsetGain_(0.05),
      timeSpentStanding_(MAX_SWITCH_TO_STANCE_DURATION + 1.),  // robot is initially standing, no interpolation from walking
      switchToStanceDuration_(2.0),
      terrainAdapter_(terrainAdapter),
      positionWorldToPathRegularizerInWorldFrame_(),
      linearVelocityPathRegularizerInWorldFrame_() {
}

bool ComSupportControlZmp::addVariablesToLog(const std::string& ns) const  {
  if (enableLogging_) {
    signal_logger::add(positionWorldToZmpInWorldFrame_, "positionWorldToZmpInWorldFrame", ns, "m");
    signal_logger::add(positionWorldToDesiredCoMInWorldFrame_, "positionWorldToDesiredCoMInWorldFrame", ns, "m");
    signal_logger::add(orientationControlToDesiredBase_, "orientationControlToDesiredBase", ns);
    signal_logger::add(linearVelocityDesiredBaseInWorldFrame_, "linearVelocityDesiredCoMInWorldFrame", ns, "ms");
    signal_logger::add(linearAccelerationDesiredTargetInWorldFrame_, "linearAccelerationDesiredCoMInWorldFrame", ns, "mss");
    signal_logger::add(positionWorldToPathRegularizerInWorldFrame_, "positionWorldToPathRegularizerInWorld", ns, "m");
    signal_logger::add(linearVelocityPathRegularizerInWorldFrame_, "linearVelocityPathRegularizerInControl", ns, "ms");

    for (const auto legEnum : anymal_description::LegEnumIterator()) {
      signal_logger::add(filteredFootholdTrackingOffsetIndicator_[legEnum].getFilteredValue(), "footholdTrackingOffsetIndicator/filtered/" + anymal_description::mapLegEnumToString[legEnum], ns, "-");
      signal_logger::add(footholdTrackingOffsetIndicator_[legEnum], "footholdTrackingOffsetIndicator/unfiltered/" + anymal_description::mapLegEnumToString[legEnum], ns, "-");
    }

    virtualPlaneFrame_.addVariablesToLog(ns + "/virtualPlaneFrame");
    terrainAdapter_.addVariablesToLog(ns + "/terrainAdapter");
    motionPlan_.addVariablesToLog(ns + "/motionPlan");
  }

  return true;
}

bool ComSupportControlZmp::addParametersToHandler(const std::string& ns) {
  bool success = true;
  success = parameter_handler::handler->addParam(ns + "/com_zmp/torsoHeight", desiredHeightAboveGroundOnFlatTerrain_) && success;
  return success;
}

bool ComSupportControlZmp::removeParametersFromHandler() {
  bool success = true;
  success = parameter_handler::handler->removeParam(desiredHeightAboveGroundOnFlatTerrain_.getName()) && success;
  return success;
}

bool ComSupportControlZmp::initialize(double dt) {
  // Reset parameter handler.
  desiredHeightAboveGroundOnFlatTerrain_.resetToDefault();

  // Optimization related.
  previousFirstPhaseEvent_.clear();
  optimizationDofs_.clear();
  isOptimizedMotionPlanAvailable_= false;

  // Reset Pins.
  isFirstSupportPolygonNewPin_ = false;
  didPinSafety_ = false;

  // Initialize timers.
  realTimeDt_.pinTime();

  // Timing.
  actualInterpolationSample_ = 0u;

  // Feet configuration.
  setCurrentFeetConfigurationInWorldFrame();

  // Initialize the world to plane orientation.
  if(!updateVirtualPlaneFrame()) { return false; }

  // Reset reference quantities.
  Position positionWorldToFootrintCenterInWorldFrame;
  if(!headingGenerator_.computeCurrentFootPrintCenterInWorldFrame(positionWorldToFootrintCenterInWorldFrame)) { return false; }
  positionWorldToDesiredCoMInWorldFrame_ = positionWorldToFootrintCenterInWorldFrame + Position::UnitZ()*desiredHeightAboveGroundOnFlatTerrain_.getValue();
  positionWorldToPathRegularizerInWorldFrame_ = positionWorldToFootrintCenterInWorldFrame + Position::UnitZ()*desiredHeightAboveGroundOnFlatTerrain_.getValue();
  positionWorldToZmpInWorldFrame_ = positionWorldToFootrintCenterInWorldFrame;
  linearVelocityDesiredBaseInWorldFrame_.setZero();
  linearVelocityPathRegularizerInWorldFrame_.setZero();
  linearAccelerationDesiredTargetInControlFrame_.setZero();
  linearAccelerationDesiredTargetInWorldFrame_.setZero();
  orientationControlToDesiredBase_.setIdentity();
  angularVelocityDesiredBaseInControlFrame_.setZero();
  angularAccelerationDesiredBaseInControlFrame_.setZero();

  // Store default parameters.
  if(!defaultZmpParams_.initialize(contactSchedule_.getActiveGaitName(), contactSchedule_.getDesiredGaitName(), -1)) { return false; }

  // Initialize the motion planner.
  if(!zmpPlanner_->stopPlanning()) { return false; }
  if(!zmpPlanner_->initialize(
      dt,
      wholeBody_.getWholeBodyProperties().getTotalMass(),
      wholeBody_.getTorso().getProperties().getInertiaTensorInBaseFrame(),
      false)) { return false; }
  zmpPlanner_->parallelize(true);


  // Initialize motion plan.
  anymalLegsVector vectorsTargetToLimbThighInBaseFrame;
  double maxLimbExtension = 0.0;
  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    const auto legIdInt = static_cast<unsigned int>(legEnum);
    vectorsTargetToLimbThighInBaseFrame[legEnum] = legs_.get(legIdInt).getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame().toImplementation();
    maxLimbExtension += legs_.get(legIdInt).getLegProperties().getMaximumLimbExtension();
  }
  if (static_cast<double>(contact_schedule::LegEnumAnymal::SIZE)>0.0) {
    maxLimbExtension /= static_cast<double>(contact_schedule::LegEnumAnymal::SIZE);
  }

  if(!motionPlan_.initialize(
      wholeBody_.getWholeBodyProperties().getTotalMass(),
      wholeBody_.getTorso().getProperties().getInertiaTensorInBaseFrame(),
      vectorsTargetToLimbThighInBaseFrame,
      maxLimbExtension*0.7)
  ) { return false; }
  if(!motionPlanPrevious_.initialize(motionPlan_)) { return false; }


  // Create first (non-optimized) reference trajectory
  if(!resetZmpTrajectory(dt)) { return false; }
  motionPlanPrevious_.copyMotionPlan(motionPlan_);

  // Initialize and reset filters.
  averageRealTimeDt_.setFilterParameters(dt, dt*40.0, 1.0, dt);
  filterVelocityProjection_.setFilterParameters(dt, 0.1, 1.0, LinearVelocity::Zero());

  // Foothold tracking offset indication.
  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    footholdTrackingOffsetIndicator_[legEnum] = 0.0;
    positionWorldToPreviousDesiredFootholdInWorldFrame_[legEnum] = feetConfigurationInWorldFrame_[legEnum];
    filteredFootholdTrackingOffsetIndicator_[legEnum].setFilterParameters(dt, 0.6, 1.0, 0.0);
  }

  return true;
}

bool  ComSupportControlZmp::stop() {
  return zmpPlanner_->stopPlanning();
}

// ToDo: rename functions in the base class!!!
const Position& ComSupportControlZmp::getPositionWorldToDesiredCoMInWorldFrame() const {
  return positionWorldToDesiredCoMInWorldFrame_;
}

// ToDo: rename functions in the base class!!!
const LinearVelocity& ComSupportControlZmp::getLinearVelocityDesiredBaseInWorldFrame() const {
  return linearVelocityDesiredBaseInWorldFrame_;
}

const LinearAcceleration& ComSupportControlZmp::getLinearAccelerationDesiredTargetInControlFrame() const {
  return linearAccelerationDesiredTargetInControlFrame_;
}

const RotationQuaternion& ComSupportControlZmp::getOrientationControlToDesiredBase() const {
  return orientationControlToDesiredBase_;
}

const LocalAngularVelocity& ComSupportControlZmp::getAngularVelocityDesiredBaseInControlFrame() const {
  return angularVelocityDesiredBaseInControlFrame_;
}

const AngularAcceleration& ComSupportControlZmp::getAngularAccelerationDesiredBaseInControlFrame() const {
  return angularAccelerationDesiredBaseInControlFrame_;
}


bool ComSupportControlZmp::loadParameters(const TiXmlHandle &hParameterSet) {
  MELO_DEBUG_STREAM(magenta << "[ComSupportControlZmp] " << blue << "Load parameters." << def)

  // Com Support, zmp and motion plan handles.
  const TiXmlHandle comSupportHandle = tinyxml_tools::getChildHandle(hParameterSet, "ComSupportControl");
  if(!tinyxml_tools::loadParameter(verbose_, comSupportHandle, "verbose")) { return false; }
  if(!tinyxml_tools::loadParameter(enableLogging_, comSupportHandle, "logging")) { return false; }
  const TiXmlHandle zmpOptimizerHandle = tinyxml_tools::getChildHandle(hParameterSet, "ZmpOptimizer");

  // Timing.
  const TiXmlHandle timingHandle = tinyxml_tools::getChildHandle(comSupportHandle, "Timing");
  std::string advanceMethod = "sampling_time";
  if(!tinyxml_tools::loadParameter(advanceMethod, timingHandle, "advance_method", advanceMethod)) { return false; }
  if (advanceMethod == "sampling_time") {
    advanceMethod_ = com_support::AdvanceMethod::Sampling_Time;
  } else if (advanceMethod == "real_sampling_time") {
    advanceMethod_ = com_support::AdvanceMethod::Real_Sampling_Time;
  } else if (advanceMethod == "average_real_sampling_time") {
    advanceMethod_ = com_support::AdvanceMethod::Average_Real_Sampling_Time;
  } else {
    MELO_FATAL_STREAM("[ComSupportControlZmp::loadParameters] Advance method can be sampling_time, real_sampling_time or average_real_sampling_time");
    return false;
  }

  // Conditioning.
  const TiXmlHandle conditioningHandle = tinyxml_tools::getChildHandle(comSupportHandle, "Conditioning");
  if(!tinyxml_tools::loadParameter(polygonTimeThreshold_, conditioningHandle, "skip_polygon_if_time_smaller_than", 0.05)) { return false; }

  // Initialization
  const TiXmlHandle initHandle = tinyxml_tools::getChildHandle(comSupportHandle, "Initialization");
  if(!tinyxml_tools::loadParameter(weightPreviousInitState_, initHandle, "alpha", 0.2)) { return false; }
  if (weightPreviousInitState_>1.0) {
    MELO_FATAL_STREAM("[ComSupportControlZmp::loadParameters] Alpha filter weight need to be smaller than 1. A negative value indicates that the filter is disabled.");
    return false;
  }

  // Line Search.
  const TiXmlHandle lineSearchHandle = tinyxml_tools::getChildHandle(comSupportHandle, "LineSearch");
  double tol; unsigned int maxIter, maxIterBackTracing; double alpha, beta; bool verbose;
  if(!tinyxml_tools::loadParameter(tol,                lineSearchHandle, "tol", 1e-12)) { return false; }
  if(!tinyxml_tools::loadParameter(maxIter,            lineSearchHandle, "max_iter", 10u)) { return false; }
  if(!tinyxml_tools::loadParameter(maxIterBackTracing, lineSearchHandle, "max_iter_back_tracing", 10u)) { return false; }
  if(!tinyxml_tools::loadParameter(alpha,              lineSearchHandle, "alpha", 0.4)) { return false; }
  if(!tinyxml_tools::loadParameter(beta,               lineSearchHandle, "beta", 0.8)) { return false; }
  if(!tinyxml_tools::loadParameter(verbose,            lineSearchHandle, "verbose", false)) { return false; }

  zmp::LineSearchOptions lineSearchOptions(tol, maxIter, maxIterBackTracing, alpha, beta, verbose, 2.0);
  motionPlan_.setLineSearchOptions(lineSearchOptions);
  motionPlanPrevious_.setLineSearchOptions(lineSearchOptions);

  // Interpolation
  const TiXmlHandle interpolationHandle = tinyxml_tools::getChildHandle(comSupportHandle, "Interpolation");
  if(!tinyxml_tools::loadParameter(numOfInterpolationSamples_, interpolationHandle, "num_of_samples", 0u)) { return false; }

  // Load parameters for zmp planner and objective handler.
  if(!zmpPlanner_->loadParameters(hParameterSet)) { return false; }

  // Torso control.
  double minHeight, maxHeight;
  TiXmlHandle torsoControlHandle = tinyxml_tools::getChildHandle(hParameterSet, "TorsoControl");
  TiXmlHandle torsoConfigurationHandle = tinyxml_tools::getChildHandle(torsoControlHandle, "TorsoConfiguration");
  TiXmlHandle torsoHeightHandle = tinyxml_tools::getChildHandle(torsoConfigurationHandle, "TorsoHeight");
  if(!tinyxml_tools::loadParameter(nominalDesiredHeightAboveGroundOnFlatTerrain_, torsoHeightHandle, "nominal")) { return false; }
  if(!tinyxml_tools::loadParameter(minHeight, torsoHeightHandle, "min")) { return false; }
  if(!tinyxml_tools::loadParameter(maxHeight, torsoHeightHandle, "max")) { return false; }
  desiredHeightAboveGroundOnFlatTerrain_.setValue(nominalDesiredHeightAboveGroundOnFlatTerrain_);
  desiredHeightAboveGroundOnFlatTerrain_.setDefaultValue(nominalDesiredHeightAboveGroundOnFlatTerrain_);
  desiredHeightAboveGroundOnFlatTerrain_.setMinValue(minHeight);
  desiredHeightAboveGroundOnFlatTerrain_.setMaxValue(maxHeight);

  // Load zmp parameters.
  std::vector<TiXmlElement*> gaitElements;
  if(!tinyxml_tools::getChildElements(gaitElements, zmpOptimizerHandle, "Gait")) { return false; }
  if(!contact_schedule::loadGaitParameters(gaitElements, defaultZmpParams_, contactSchedule_.getMapGaitNameToId())) { return false; }

  // Path regularizer.
  TiXmlHandle pathRegHandle = hParameterSet;
  if(!tinyxml_tools::getChildHandle(pathRegHandle, comSupportHandle, "PathRegularizer")) { return false; }
  if(!tinyxml_tools::loadParameter(weightDisturbanceFollower_, pathRegHandle, "weight_disturbance_follower")) { return false; }

  // Terrain adaption.
  terrainAdapter_.loadParameters(comSupportHandle);

  // Foothold Tracking.
  TiXmlHandle footholdTrackingHandle = hParameterSet;
  if(!tinyxml_tools::getChildHandle(footholdTrackingHandle, comSupportHandle, "FootholdTracking")) { return false; }
  if(!tinyxml_tools::loadParameter(nominalFootholdTrackingOffset_, footholdTrackingHandle, "nominal_offset"))  { return false; }
  if(!tinyxml_tools::loadParameter(footholdTrackingOffsetGain_, footholdTrackingHandle, "gain"))  { return false; }

  // Switch to stance.
  TiXmlHandle switchToStanceHandle = hParameterSet;
  if (tinyxml_tools::getChildHandle(switchToStanceHandle, comSupportHandle, "SwitchToStance", false)) {
    const double defaultDuration = switchToStanceDuration_;
    tinyxml_tools::loadParameter(switchToStanceDuration_, switchToStanceHandle, "duration", defaultDuration);
  }
  switchToStanceDuration_ = std::max(MIN_SWITCH_TO_STANCE_DURATION, std::min(MAX_SWITCH_TO_STANCE_DURATION, switchToStanceDuration_));

  return true;
}

bool ComSupportControlZmp::resetZmpTrajectory(double dt) {
  /*
   * This function computes a trajectory that approaches the CoG towards the footprint center.
   * From an implementation point of view, this function fully replaces the zmp optimization!
   * We assume that all legs are grounded. If this is not the case, stability may not be guaranteed.
   * Call this function to initialize the motion plan or to get a trajectory in short time (replace zmp optimizer).
   */

  bool success = true;

  {
    boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);

    motionPlanPrevious_.copyMotionPlan(motionPlan_);
    success &= updateVirtualPlaneFrame();
    motionPlan_.setVirtualPlaneFrame(virtualPlaneFrame_);
    const auto& posePlaneToWorld = virtualPlaneFrame_.getPosePlaneToWorld();


    /**********************
     * Linear Knot States *
     **********************/
    positionVector positionKnotInPlaneFrame(2, Position::Zero());
    velocityVector velocityKnotInPlaneFrame(2, LinearVelocity::Zero());
    accelerationVector accelerationKnotInPlaneFrame(2, LinearAcceleration::Zero());
    constexpr double approachDuration = 0.4;

    // Set initial acceleration.
    if(motionPlanPrevious_.didOptimizationSucceeded()) {
      accelerationKnotInPlaneFrame[0] = posePlaneToWorld.getRotation().inverseRotate(motionPlanPrevious_.getLinearAccelerationComInWorldFrame());
    }

    // Compute initial position and velocity.
    success &= getMeasuredTorsoTargetStateInPlaneFrame(positionKnotInPlaneFrame[0], velocityKnotInPlaneFrame[0]);

    // Compute final position.
    Position positionWorldToFootPrintCenterInWorldFrame;
    success &= headingGenerator_.computeCurrentFootPrintCenterInWorldFrame(positionWorldToFootPrintCenterInWorldFrame);
    const Position positionPlaneToFootPrintCenterInPlaneFrame = posePlaneToWorld.inverseTransform(positionWorldToFootPrintCenterInWorldFrame);

    const Position& positionPlaneToDesiredTargetHeightInPlaneFrame = terrainAdapter_.getPositionPlaneToDesiredTargetHeightInPlaneFrame();
    positionKnotInPlaneFrame[1] = Position(
        positionPlaneToFootPrintCenterInPlaneFrame.x() + positionPlaneToDesiredTargetHeightInPlaneFrame.x(),
        positionPlaneToFootPrintCenterInPlaneFrame.y() + positionPlaneToDesiredTargetHeightInPlaneFrame.y(),
        positionPlaneToDesiredTargetHeightInPlaneFrame.z()
    );
    /**********************/


    /**************************
     * Rotational Knot States *
     **************************/
    eulerAnglesZyxVector anglesZyxBaseToPlane(2);
    eulerAnglesZyxDiffVector angularRatesZyxBaseInPlaneFrame(2);

    // Compute initial orientation and angular rate.
    success &= getMeasuredTorsoTargetRotationInPlaneFrame(anglesZyxBaseToPlane[0], angularRatesZyxBaseInPlaneFrame[0]);

    // Compute final orientation (aligned with heading direction of footprint).
    anglesZyxBaseToPlane[1] = EulerAnglesZyx(getOrientationFootprintHeadingToPlane()).getUnique();
    angularRatesZyxBaseInPlaneFrame[1].setZero();
    /**************************/

    // Set list of support polygons.
    supportPolygons_.clear();
    setCurrentFeetConfigurationInWorldFrame();
    emplaceSupportPolygon(feetConfigurationInWorldFrame_, approachDuration);
    motionPlan_.setSupportPolygonsInPlaneFrame(supportPolygons_, false);

    // Set com state.
    std::vector<double> duration(1, approachDuration);
    if(!motionPlan_.getComStateInPlaneFramePtr()->setMotionPlanLinearAngular(
        positionKnotInPlaneFrame, velocityKnotInPlaneFrame, accelerationKnotInPlaneFrame,
        anglesZyxBaseToPlane, angularRatesZyxBaseInPlaneFrame,
        duration, approachDuration)) { return false; }

    // Set termination state.
    motionPlan_.setTerminationState(zmp::TerminationState::returnInitialGuess);
    isOptimizedMotionPlanAvailable_ = false;
  }

  return success; // return success after mutexMotionPlan_ is unlocked!!
}

bool ComSupportControlZmp::setMotionPlan(zmp::MotionPlan& optimizedMotionPlan) {
  // Compute optimization dofs.
  optimizationDofs_ = zmp::computeUnionsSet(defaultZmpParams_.getActiveParams().optimizationDofs_, defaultZmpParams_.getDesiredParams().optimizationDofs_);
  optimizedMotionPlan.setOptimizationDofs(optimizationDofs_);

  // Update virtual plane frame.
  if(!updateVirtualPlaneFrame()) { return false; }
  optimizedMotionPlan.setVirtualPlaneFrame(virtualPlaneFrame_);

  // Compute the polygon sequence.
  int polygonIdAtSwitch; bool isFirstSupportPolygonNew;
  if (!computeSupportPolygonSequence(polygonIdAtSwitch, isFirstSupportPolygonNew)) {
    // do not display a warning!! Function returns false under usual operation.
    return false;
  }
  optimizedMotionPlan.setSupportPolygonsInPlaneFrame(supportPolygons_, isFirstSupportPolygonNew);
  //printSupportPolygonSequence();

  // Update zmp parameters.
  defaultZmpParams_.setGaitInfo(contactSchedule_.getActiveGaitName(), contactSchedule_.getDesiredGaitName(), polygonIdAtSwitch);
  if(!optimizedMotionPlan.setParameters(defaultZmpParams_)) { return false; }


  // For proper visualization of the orientation we need some additional rotation quaternions.
  optimizedMotionPlan.setOrientations(wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl());

  // Equality constraints options.
  optimizedMotionPlan.setEqualityConstraints(contactSchedule_.getStatus() == contact_schedule::Status::Stand, true);

  /********************************
   * Initial State (Linear State) *
   ********************************/
  // Get measured state.
  Position initPositionPlaneToTargetInPlaneFrame;
  LinearVelocity initLinearVelocityTargetInPlaneFrame;
  if(!getMeasuredTorsoTargetStateInPlaneFrame(
      initPositionPlaneToTargetInPlaneFrame,
      initLinearVelocityTargetInPlaneFrame)) { return false; }

  // Get reference state from previous optimization.
  Position refPositionPlaneToTargetInPlaneFrame;
  LinearVelocity refLinearVelocityTargetInPlaneFrame;
  LinearAcceleration refLinearAccelerationTargetInPlaneFrame;
  if(!getReferenceTorsoTargetStateInPlaneFrame(
      refPositionPlaneToTargetInPlaneFrame,
      refLinearVelocityTargetInPlaneFrame,
      refLinearAccelerationTargetInPlaneFrame)) { return false; }

  // Merge measurements with reference signal.
  if(!alphaFilterForInitState(
      initPositionPlaneToTargetInPlaneFrame,  refPositionPlaneToTargetInPlaneFrame,
      initLinearVelocityTargetInPlaneFrame,   refLinearVelocityTargetInPlaneFrame,
      weightPreviousInitState_)) { return false; }

  // Locally save the current state.
  optimizedMotionPlan.setPlaneToInitialPositionInPlaneFrame(initPositionPlaneToTargetInPlaneFrame);
  optimizedMotionPlan.setInitialVelocityInPlaneFrame(initLinearVelocityTargetInPlaneFrame);
  optimizedMotionPlan.setInitialAccelerationInPlaneFrame(refLinearAccelerationTargetInPlaneFrame);
  /********************************/

  /***********************************
   * Path Regularizer (Linear State) *
   /***********************************/
  // Compute knot points of the path regularizer (initial and final points).
  positionVector positionKnotInPlaneFrame;
  velocityVector velocityKnotInPlaneFrame;
  accelerationVector accelerationKnotInPlaneFrame;
  Position desiredVelocityProjectionInPlaneFrame;
  std::vector<double> duration;

  if(!computePathKnotsInPlaneFrame(
      positionKnotInPlaneFrame, velocityKnotInPlaneFrame, accelerationKnotInPlaneFrame,
      duration,
      optimizedMotionPlan.getOptimizationHorizon(),
      desiredVelocityProjectionInPlaneFrame)) { return false; }

  /***********************************/

  /******************************
   * Final State (Linear State) *
   ******************************/
  // Locally save the final state
  optimizedMotionPlan.setPlaneToFinalPositionInPlaneFrame(positionKnotInPlaneFrame.back());
  optimizedMotionPlan.setFinalVelocityInPlaneFrame(velocityKnotInPlaneFrame.back());
  /******************************/

  /************************************
   * Initial State (Rotational State) *
   ************************************/
  EulerAnglesZyx anglesZyxInitBaseToPlane;
  if (zmp::containsRotation(optimizationDofs_)) {
    EulerAnglesZyxDiff initAngularRatesTargetInPlaneFrame;

    // Get measured state.
    if(!getMeasuredTorsoTargetRotationInPlaneFrame(
        anglesZyxInitBaseToPlane,
        initAngularRatesTargetInPlaneFrame)) { return false; }

    // Get reference state from previous optimization.
    EulerAnglesZyx anglesZyxRefBaseToPlane;
    EulerAnglesZyxDiff refAngularRatesZyxRefBaseInPlaneFrame;
    EulerAnglesZyxDiff refAngularAccelTargetInPlaneFrame;
    if(!getReferenceTorsoTargetStateInPlaneFrame(
        anglesZyxRefBaseToPlane,
        refAngularRatesZyxRefBaseInPlaneFrame,
        refAngularAccelTargetInPlaneFrame)) { return false; }

    // Merge measurements with reference signal.
    if(!alphaFilterForInit(
        anglesZyxInitBaseToPlane,           anglesZyxRefBaseToPlane,
        initAngularRatesTargetInPlaneFrame, refAngularRatesZyxRefBaseInPlaneFrame,
        weightPreviousInitState_)) { return false; }

    // Locally save the current state.
    optimizedMotionPlan.setAnglesZyxInitialBaseToPlane(anglesZyxInitBaseToPlane);
    optimizedMotionPlan.setInitialAngularRatesZyxInPlaneFrame(initAngularRatesTargetInPlaneFrame);
    optimizedMotionPlan.setInitialAngularAccelerationZyxInPlaneFrame(refAngularAccelTargetInPlaneFrame);
  }
  /********************************/


  /***************************************
   * Path Regularizer (Rotational State) *
  /***************************************/
  // Compute knot points of the path regularizer (initial and final points).
  eulerAnglesZyxVector anglesZyxPathToPlane;
  eulerAnglesZyxDiffVector angularRatesZyxPathInPlaneFrame;
  EulerAnglesZyx orientationDesiredVelocityOffset;

  // Note: also compute if we are not optimizing for orientation (we will copy the path regualrizer into the solution)!
  if(!computePathKnotsInPlaneFrame(
      anglesZyxPathToPlane,
      angularRatesZyxPathInPlaneFrame,
      duration,
      optimizedMotionPlan.getOptimizationHorizon(),
      orientationDesiredVelocityOffset)) { return false; }

  // Set path regularizer.
  if(!optimizedMotionPlan.getPathRegularizerInPlaneFramePtr()->setMotionPlanLinearAngular(
      positionKnotInPlaneFrame, velocityKnotInPlaneFrame, accelerationKnotInPlaneFrame,
      anglesZyxPathToPlane, angularRatesZyxPathInPlaneFrame,
      duration, optimizedMotionPlan.getOptimizationHorizon())) { return false; }

  /***********************************/

  /************************************
   * Final State (Rotational State) *
   ************************************/
  if (zmp::containsRotation(optimizationDofs_)) {
    if (contactSchedule_.getStatus() == contact_schedule::Status::Stand) {
      optimizedMotionPlan.setAnglesZyxFinalBaseToPlane(anglesZyxPathToPlane.back());
    } else {
      const auto anglesZyxFinalBaseToPlane = (anglesZyxInitBaseToPlane*orientationDesiredVelocityOffset).getUnique();
      optimizedMotionPlan.setAnglesZyxFinalBaseToPlane(anglesZyxFinalBaseToPlane);
    }
    optimizedMotionPlan.setFinalAngularRatesZyxInPlaneFrame(angularRatesZyxPathInPlaneFrame.back());
  }
  /********************************/

  // Others.
  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    optimizedMotionPlan.setFootholdTrackingOffsetIndicator(filteredFootholdTrackingOffsetIndicator_[legEnum].getFilteredValue(), legEnum);

    const Position& positionWorldToEndEffectorInWorldFrame = legs_.get(static_cast<int>(legEnum)).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    const Position positionPlaneToEndEffectorInPlaneFrame = virtualPlaneFrame_.getPosePlaneToWorld().inverseTransform(positionWorldToEndEffectorInWorldFrame);
    optimizedMotionPlan.setPositionPlaneToEndEffectorInPlaneFrame(positionPlaneToEndEffectorInPlaneFrame.toImplementation(), legEnum);
  }
  optimizedMotionPlan.setGravityFactor(terrainAdapter_.getGravityFactor());

  return true;
}


bool ComSupportControlZmp::planCenterOfMassMotion(double dt) {
  zmp::MotionPlan optimizedMotionPlan;
  {
    // Initialize motion plan using the previous motion plan.
    bool success = true;
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
      success &= optimizedMotionPlan.initialize(motionPlan_);
      success &= optimizedMotionPlan.copyComState(motionPlan_, 0.0);
    }

    if (!success) { return false; }
  }

  // Add initial and final states as well as the list of support polygons.
  if (!setMotionPlan(optimizedMotionPlan)) {
    return true; // If fails, try once again (do not return false!!)
  }

  // Update.
  zmpPlanner_->setMotionPlan(optimizedMotionPlan);
  zmpPlanner_->startPlanning();
  return true;
}

bool ComSupportControlZmp::setToInterpolated(const ComSupportControlBase& supportPolygon1, const ComSupportControlBase& supportPolygon2, double t) {
  return false;
}

bool ComSupportControlZmp::computeSupportPolygonSequence(int& polygonIdAtSwitch, bool& isFirstSupportPolygonNew) {
  supportPolygons_.clear();
  isFirstSupportPolygonNew = false;
  polygonIdAtSwitch        = -1;
  const auto& listOfEvents = contactSchedule_.getListOfEvents();

  // If the robot should be standing, just send the first support polygon to the optimizer.
  if (listOfEvents.empty()) {
    setCurrentFeetConfigurationInWorldFrame();
    std::fill(isStancedLegId_.begin(), isStancedLegId_.end(), true);
    emplaceSupportPolygon(feetConfigurationInWorldFrame_, computeApproachDuration());
    previousFirstPhaseEvent_.clear();
    return true;
  }

  // Update stance leg id for current situation.
  // ToDo: consider limb strategy for the first support polygon (in particular early touch down)
  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    const auto legIdInt =static_cast<unsigned int>(legEnum);
    if (contactSchedule_.shouldBeLegGrounded(legEnum)) {
      isStancedLegId_[legEnum] = true;
      feetConfigurationInWorldFrame_[legEnum] = legs_.get(legIdInt).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    } else {
      isStancedLegId_[legEnum] = false;
      feetConfigurationInWorldFrame_[legEnum] = legs_.get(legIdInt).getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();
    }
  }

  // Helper variables.
  double previousEventTime = 0.0;

  // Iterate over all future events.
  for (auto eventIt = listOfEvents.begin() ; eventIt != listOfEvents.end(); ++eventIt) {

    // Compute support polygon time.
    double timeLeftInPolygon = eventIt->first - previousEventTime;
    previousEventTime = eventIt->first ;
    assert(timeLeftInPolygon>=0.0);

    // Handling of small support polygon durations.
    bool skipSupportPolygon = false;
    if (timeLeftInPolygon < polygonTimeThreshold_) {
      if (eventIt == listOfEvents.begin()) { return false; }
      else { skipSupportPolygon = true; }
    }

    // Add support polygon.
    if (!skipSupportPolygon) {
      emplaceSupportPolygon(feetConfigurationInWorldFrame_, timeLeftInPolygon);
    }

    // Find polygon index at which switch between active and desired gait happens.
    if (contactSchedule_.getStatus() == contact_schedule::Status::SwitchGait && polygonIdAtSwitch == -1) {
      for (const auto& event : eventIt->second) {
        if (event.isDesiredGait_) {
          polygonIdAtSwitch = supportPolygons_.size();

          // Check if we are already in the active gait.
          if (polygonIdAtSwitch == 1 && contactSchedule_.isDesiredGait()) { polygonIdAtSwitch = 0; }
          break;
        }
      }
    }

    // Update stance legs and feet configuration for next support polygon.
    for (const auto& event : eventIt->second) {
      switch(event.contactEvent_) {

        case contact_schedule::ContactEvent::LiftOff : {
          if (!isStancedLegId_[event.legId_]) {
            MELO_WARN_STREAM("[ComSupportControlZmp::computeSupportPolygonSequence] Leg is swinging before lift-off (support id = " <<
                supportPolygons_.size() << ", leg = " << event.legId_ << ").");
            return false;
          }
          isStancedLegId_[event.legId_] = false;
          feetConfigurationInWorldFrame_[event.legId_] =
              legs_.get(static_cast<unsigned int>(event.legId_)).getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();
        } break;

        case contact_schedule::ContactEvent::TouchDown : {
          if (isStancedLegId_[event.legId_]) {
            MELO_WARN_STREAM("[ComSupportControlZmp::computeSupportPolygonSequence] Leg is grounded before touch-down (support id = " <<
                supportPolygons_.size() << ", leg = " << event.legId_ << ").");
            return false;
          }
          isStancedLegId_[event.legId_] = true;
        } break;

        case contact_schedule::ContactEvent::EndOfGait : {
          supportPolygons_.back().setDuration(computeApproachDuration());
        } break;

        default: break;
      }
    }
  } // end for eventIt

  // If we are using a periodic gait -> close the loop.
  if (contactSchedule_.isGaitPeriodic()) {
    const double timeToCloseTheGait = contactSchedule_.getStrideDuration() - std::prev(listOfEvents.end())->first;
    if (timeToCloseTheGait>polygonTimeThreshold_) {
      emplaceSupportPolygon(feetConfigurationInWorldFrame_, timeToCloseTheGait);
    }
  }

  // Check if a new phase event has appeared at the beginning of the gait.
  isFirstSupportPolygonNew  = !contact_schedule::areEventsIdentical(previousFirstPhaseEvent_, listOfEvents.begin()->second);
  previousFirstPhaseEvent_  = listOfEvents.begin()->second;

  return true;
}

void ComSupportControlZmp::emplaceSupportPolygon(
    const anymalLegsPosition& feetConfigurationInWorldFrame,
    double timeLeftInPolygon) {
  supportPolygons_.emplace_back(
      feetConfigurationInWorldFrame,
      timeLeftInPolygon,
      isStancedLegId_,
      virtualPlaneFrame_.getPosePlaneToWorld());
}

bool ComSupportControlZmp::advance(double dt) {
  if(!updateFootholdTrackingOffsetIndicator()) {
    return false;
  }

  // Timers and time scaling.
  const double realTimeDt = realTimeDt_.getElapsedTimeSec();                  // time passed since the last advance step.
  realTimeDt_.pinTime();                                                      // Pin the timer to store the real time control loop update time.
  averageRealTimeDt_.advance(realTimeDt);                                     // compute mean real-time-dt online
  const double timeScale =  (isRealRobot_ ? 1.0 : dt/averageRealTimeDt_.getFilteredValue());  // time conversion from simulation to real world.

  // Keep track of time spent standing for base yaw interpolation (see computePathKnotsInPlaneFrame)
  if (contactSchedule_.getStatus() == contact_schedule::Status::Stand) {
    timeSpentStanding_ += realTimeDt;
  }

  // Compute time for which we advance on the splines.
  double timeToAdvance = 0.0;
  switch(advanceMethod_) {
    case com_support::AdvanceMethod::Sampling_Time :              { timeToAdvance = dt; } break;
    case com_support::AdvanceMethod::Real_Sampling_Time :         { timeToAdvance = realTimeDt; } break;
    case com_support::AdvanceMethod::Average_Real_Sampling_Time : { timeToAdvance = averageRealTimeDt_.getFilteredValue(); } break;
    default : { MELO_FATAL_STREAM("[ComSupportControlZmp::advance] Unknown advance method."); } break;
  }

  // Check if planning has finished.
  if (zmpPlanner_->hasStartedPlanning() && zmpPlanner_->hasFinishedPlanning()) {
    zmpPlanner_->setHasStartedPlanning(false);

    if (zmpPlanner_->getDidOptimizationSuceeded()) {
      isOptimizedMotionPlanAvailable_ = true;
      didPinSafety_                   = false;
      actualInterpolationSample_      = 0u;
      bool success = true;

      {
        boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);

        // Store the old motion plan and get the new motion plan.
        motionPlanPrevious_.copyMotionPlan(motionPlan_);
        zmpPlanner_->getMotionPlan(motionPlan_);

        // Get measured initial state.
        const auto& posePlaneToWorld = motionPlan_.getVirtualPlaneFrame().getPosePlaneToWorld();
        const auto positionPlaneToTargetInPlaneFrame = posePlaneToWorld.inverseTransform(
            getPositionWorldToTargetInWorldFrame(wholeBody_));

        const auto orientationBaseToPlane = EulerAnglesZyx(
            wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase()*
            posePlaneToWorld.getRotation()
        ).inverted().getUnique();

        // Find optimal spline time.
        const double initGuess = zmpPlanner_->getLastComputationDuration()*timeScale+dt;
        const double optimalContainerTime = motionPlan_.getComStateInPlaneFramePtr()->lineSearch(
            initGuess, positionPlaneToTargetInPlaneFrame, orientationBaseToPlane,
            0.0, 0.45*motionPlan_.getOptimizationHorizon()
        );

        // Advance on the splines by the optimal container time.
        success &= motionPlan_.setContainerTime(optimalContainerTime);
      }

      if (!success) { return false; }

    // Optimization failed.
    } else {
      zmp::MotionPlan motionPlan;
      zmpPlanner_->getMotionPlan(motionPlan);

      // Check if optimization failed due to invalid motion plan.
      MELO_INFO_STREAM("[ComSupportControlZmp::advance] Check previous motion plan...");
      if (!motionPlan.checkMotionPlan()) { motionPlan.print(); }
      MELO_INFO_STREAM("[ComSupportControlZmp::advance] Done.");

      // Safety.
      if (motionPlan.getTerminationState() == zmp::TerminationState::failedToReadMotionPlan) {
        return false;
      }
    }
  }

  // Extract solution.
  if(!processReferenceSignals(timeToAdvance, dt)) {
    MELO_FATAL_STREAM("[ComSupportControlZmp::advance] Failed to advance reference signals.");
    return false;
  }

  // Debugging output.
  if (verbose_) {
    std::stringstream msg;
    msg << "comp. duration [ms]: "   << std::to_string(zmpPlanner_->getLastComputationDuration()*1000.0)
        << " interp. samples: "      << std::to_string(actualInterpolationSample_);
    MELO_INFO_THROTTLE_STREAM(0.5, msg.str());
  }

  // If previous optimization has finished we are ready for a new update.
  if (!zmpPlanner_->hasStartedPlanning() && zmpPlanner_->hasFinishedPlanning()) {
    if(!planCenterOfMassMotion(dt)) {
      MELO_FATAL_STREAM("[ComSupportControlZmp::advance] Failed to plan COG trajectory.");
      return false;
    }
  }

  return true;
}

bool ComSupportControlZmp::readMotionPlan() {
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  const auto& orientationWorldToControl = wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();

  // Linear States.
  positionWorldToDesiredCoMInWorldFrame_         = motionPlan_.getPositionWorldToComInWorldFrame();
  linearVelocityDesiredBaseInWorldFrame_         = motionPlan_.getLinearVelocityComInWorldFrame();
  linearAccelerationDesiredTargetInWorldFrame_   = motionPlan_.getLinearAccelerationComInWorldFrame();
  linearAccelerationDesiredTargetInControlFrame_ = orientationWorldToControl.rotate(linearAccelerationDesiredTargetInWorldFrame_);

  // Zero-tilting moment point position in world frame
  motionPlan_.getWorldToZmpPositionInWorldFrame(positionWorldToZmpInWorldFrame_);

  // Path regularizer
  positionWorldToPathRegularizerInWorldFrame_ = motionPlan_.getPositionWorldToPathInWorldFrameAtTime(0);
  linearVelocityPathRegularizerInWorldFrame_ = motionPlan_.getVelocityPathInWorldFrameAtTime(0);

  //  Angular state (Note: even if we do not optimize for orientation, we copy back the path path regularizer).
  orientationControlToDesiredBase_              = RotationQuaternion(motionPlan_.getAnglesZyxWorldToBase())*orientationWorldToControl.inverted();
  angularVelocityDesiredBaseInControlFrame_     = orientationWorldToControl.rotate(motionPlan_.getAngularVelocityBaseInWorldFrame());
  angularAccelerationDesiredBaseInControlFrame_ = orientationWorldToControl.rotate(motionPlan_.getAngularAccelerationBaseInWorldFrame());

  return true;
}

bool ComSupportControlZmp::processReferenceSignals(double timeToAdvance, double dt) {
  if (isOptimizedMotionPlanAvailable_) {

    // (1) Advance the current motion plan.
    bool success = true;
    {
      boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);
      success &= motionPlan_.getComStateInPlaneFramePtr()->advance(timeToAdvance, true);
    }

    // (2) If failed, try again.
    if (!success) {
      MELO_WARN_STREAM("[ComSupportControlZmp::processReferenceSignals] Failed to advance optimized motion plan. Stop gait.");
      isOptimizedMotionPlanAvailable_ = false;            // make sure we don't pass through this section again.
      if(!stopPlanner(dt)) { return false; }              // kill optimization to start a new one.
      if(!contactSchedule_.stopGait()) {return false; }   // no motion-plan -> no motion.
      if(!setTrivialControlSignals(dt)) {return false;}   // we still need some reference signals.
    }

    // (3) Check if optimization is stuck (cycling, locked mutex, etc.).
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);

      constexpr double ratio = 0.5;
      const double expectedMaxComputationTime = ratio *motionPlan_.getComStateInPlaneFrame().getContainerDuration();

      if (!didPinSafety_ && motionPlan_.getComStateInPlaneFrame().getContainerTime()>expectedMaxComputationTime  ) {
        MELO_WARN_STREAM("[ComSupportControlZmp::processReferenceSignals] Container time reached << " << ratio*100.0 << "% of container duration.");
        success &= stopPlanner(dt);     // kill optimization to start a new one.
        didPinSafety_ = true;           // make sure we kill the planner not again until we get a new solution.

        // Check if optimization stuck due to invalid motion plan.
        zmp::MotionPlan motionPlan;
        zmpPlanner_->getMotionPlan(motionPlan);
        MELO_INFO_STREAM("[ComSupportControlZmp::processReferenceSignals] Check previous motion plan...");
        if (!motionPlan.checkMotionPlan()) { motionPlan.print(); }
        MELO_INFO_STREAM("[ComSupportControlZmp::processReferenceSignals] Done.");
      }
    }

    if (!success) { return false; }

    // (4) Read current reference state.
    if(!readMotionPlan()) {
      MELO_FATAL_STREAM("[ComSupportControlZmp::processReferenceSignals] Failed to read motion plan!");
      return false;
    }

    // (5) Merge with previous motion plan.
    if (actualInterpolationSample_ < numOfInterpolationSamples_ && motionPlanPrevious_.didOptimizationSucceeded()) {
      // Advance the old motion plan if needed.
      if(!motionPlanPrevious_.getComStateInPlaneFramePtr()->advance(timeToAdvance, false)) {
        MELO_WARN_STREAM("[ComSupportControlZmp::processReferenceSignals] Failed to advance previous optimized motion plan.");
        actualInterpolationSample_ = numOfInterpolationSamples_; // make sure we don't pass through this section again.
      }

      // weight for current motion plan
      const double weightCurrent  = static_cast<double>(actualInterpolationSample_) / static_cast<double>(numOfInterpolationSamples_);
      const double weightPrevious = 1.0 - weightCurrent;
      const RotationQuaternion& orientationWorldToControl = wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();

      // Interpolate between current and previous motion plan.
      robot_utils::alphaFilter(
          positionWorldToDesiredCoMInWorldFrame_.toImplementation(),
          motionPlanPrevious_.getPositionWorldToComInWorldFrame().toImplementation(),
          weightCurrent, weightPrevious);

      robot_utils::alphaFilter(
          linearVelocityDesiredBaseInWorldFrame_.toImplementation(),
          motionPlanPrevious_.getLinearVelocityComInWorldFrame().toImplementation(),
          weightCurrent, weightPrevious);

      robot_utils::alphaFilter(
          linearAccelerationDesiredTargetInControlFrame_.toImplementation(),
          orientationWorldToControl.rotate(motionPlanPrevious_.getLinearAccelerationComInWorldFrame()).toImplementation(),
          weightCurrent, weightPrevious);

      robot_utils::alphaFilter(
          orientationControlToDesiredBase_,
          RotationQuaternion(motionPlanPrevious_.getAnglesZyxWorldToBase())*orientationWorldToControl.inverted(),
          weightCurrent, weightPrevious);

      robot_utils::alphaFilter(
          angularVelocityDesiredBaseInControlFrame_.toImplementation(),
          orientationWorldToControl.rotate(motionPlanPrevious_.getAngularVelocityBaseInWorldFrame()).toImplementation(),
          weightCurrent, weightPrevious);

      robot_utils::alphaFilter(
          angularAccelerationDesiredBaseInControlFrame_.toImplementation(),
          orientationWorldToControl.rotate(motionPlanPrevious_.getAngularAccelerationBaseInWorldFrame()).toImplementation(),
          weightCurrent, weightPrevious);

      // Advance timing.
      ++actualInterpolationSample_;
    }
  }

  // We don't have an optimized motion plan available. Let's make an approximation...
  else {
    MELO_INFO_THROTTLE_STREAM(0.2, "[ComSupportControlZmp::processReferenceSignals] Compute trivial control signals.");
    if(!setTrivialControlSignals(dt)) {return false;}
  }

  return true;
}

bool ComSupportControlZmp::setTrivialControlSignals(double timeToAdvance) {
  bool success = true;
  success &= resetZmpTrajectory(timeToAdvance);
  {
    boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);
    success &= motionPlan_.getComStateInPlaneFramePtr()->advance(timeToAdvance, false);
  }
  success &= readMotionPlan();
  return success;
}

bool ComSupportControlZmp::stopPlanner(double dt) {
  MELO_INFO_STREAM("[ComSupportControlZmp::stopPlanner] Stop planner.");
  if(!zmpPlanner_->stopPlanning()) { return false; }
  if(!zmpPlanner_->initialize(
      dt,
      wholeBody_.getWholeBodyProperties().getTotalMass(),
      wholeBody_.getTorso().getProperties().getInertiaTensorInBaseFrame(),
      false)) { return false; }
  return true;
}

bool ComSupportControlZmp::alphaFilterForInitState(
    Position& position,       const Position& referencePosition,
    LinearVelocity& velocity, const LinearVelocity& referenceVelocity,
    double weightRef) const {

  const double weight = 1.0-weightRef;
  robot_utils::alphaFilter(position.toImplementation(), referencePosition.toImplementation(), weight, weightRef);
  robot_utils::alphaFilter(velocity.toImplementation(), referenceVelocity.toImplementation(), weight, weightRef);
  return true;
}

bool ComSupportControlZmp::alphaFilterForInit(
    EulerAnglesZyx& angleZyx,         const EulerAnglesZyx& referenceAngleZyx,
    EulerAnglesZyxDiff& angularRates, const EulerAnglesZyxDiff& referenceAngularRates,
    double weightRef) const {

  // Safety (in case that a frame jumps).
  constexpr double tol = M_PI/4.0;
  if (!robot_utils::areNear(angleZyx, referenceAngleZyx, tol)) {
    MELO_WARN_STREAM("[ComSupportControlZmp::alphaFilterForInit] Angles are too far apart. Cannot merge.");
    return false;
  }

  const double weight = 1.0-weightRef;
  robot_utils::alphaFilter(angleZyx.toImplementation(),     referenceAngleZyx.toImplementation(),     weight, weightRef);
  robot_utils::alphaFilter(angularRates.toImplementation(), referenceAngularRates.toImplementation(), weight, weightRef);
  angleZyx.setUnique();

  return true;
}


bool ComSupportControlZmp::getMeasuredTorsoTargetStateInPlaneFrame(
    Position& positionPlaneToMeasuredTargetInPlaneFrame,
    LinearVelocity& linearVelocityMeasuredTargetInPlaneFrame) const {
  positionPlaneToMeasuredTargetInPlaneFrame = virtualPlaneFrame_.getPosePlaneToWorld().inverseTransform(getPositionWorldToTargetInWorldFrame(wholeBody_));
  linearVelocityMeasuredTargetInPlaneFrame  = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().inverseRotate(getLinearVelocityTargetInWorldFrame(wholeBody_));
  return true;
}

bool ComSupportControlZmp::getMeasuredTorsoTargetRotationInPlaneFrame(
    EulerAnglesZyx& anglesZyxBaseToPlane,
    EulerAnglesZyxDiff& angularRatesZyxBaseInPlaneFrame) const {

  // Compute orientation.
  const auto orientationPlaneToBase =
      wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase()*
      virtualPlaneFrame_.getPosePlaneToWorld().getRotation();
  anglesZyxBaseToPlane = EulerAnglesZyx(orientationPlaneToBase.inverted()).getUnique();

  // Compute angular rates.
  angularRatesZyxBaseInPlaneFrame = EulerAnglesZyxDiff(
      zmp::getMatrixAngularVelocityInBodyFrameToAngularRatesZyx(anglesZyxBaseToPlane) *
      wholeBody_.getTorso().getMeasuredState().getAngularVelocityBaseInBaseFrame().toImplementation()
  );

  return true;
}


bool ComSupportControlZmp::getReferenceTorsoTargetStateInPlaneFrame(
    Position& refPositionPlaneToTargetInPlaneFrame,
    LinearVelocity& refVelocityTargetInPlaneFrame,
    LinearAcceleration& refAccelerationTargetInPlaneFrame) const {
  /*
   * The motion plan stored in the class contains the previously optimized reference motion. The quantities that are
   * being read from it have to be projected to the current virtual plane frame, which can be different from the one
   * stored in the motion plan class.
   */
  const auto& posePlaneToWorld = virtualPlaneFrame_.getPosePlaneToWorld();
  const auto& orientationPlaneToWorld  = posePlaneToWorld.getRotation();
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  {
    refPositionPlaneToTargetInPlaneFrame = posePlaneToWorld.inverseTransform(motionPlan_.getPositionWorldToComInWorldFrame());
    refVelocityTargetInPlaneFrame        = orientationPlaneToWorld.inverseRotate(motionPlan_.getLinearVelocityComInWorldFrame());
    refAccelerationTargetInPlaneFrame    = orientationPlaneToWorld.inverseRotate(motionPlan_.getLinearAccelerationComInWorldFrame());
  }
  return true;
}

bool ComSupportControlZmp::getReferenceTorsoTargetStateInPlaneFrame(
    EulerAnglesZyx& anglesZyxRefBaseToPlane,
    EulerAnglesZyxDiff& refAngularRatesZyxRefBaseInPlaneFrame,
    EulerAnglesZyxDiff& refAngularAccelZyxRefBaseInPlaneFrame) const{
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);

  const auto& orientationPlaneToWorld      = virtualPlaneFrame_.getPosePlaneToWorld().getRotation();
  const auto orientationWorldToDesiredBase = RotationQuaternion(motionPlan_.getAnglesZyxWorldToBase());

  // Compute orientation.
  anglesZyxRefBaseToPlane = EulerAnglesZyx(orientationWorldToDesiredBase*orientationPlaneToWorld).inverted().getUnique();

  // Compute transformations matrix.
  const auto velToRates = zmp::getMatrixAngularVelocityInInertialFrameToAngularRatesZyx(anglesZyxRefBaseToPlane);

  // Compute angular rates (*).
  const auto angularVelocityBaseInPlaneFrame = orientationPlaneToWorld.inverseRotate(motionPlan_.getAngularVelocityBaseInWorldFrame());
  refAngularRatesZyxRefBaseInPlaneFrame = EulerAnglesZyxDiff(velToRates*angularVelocityBaseInPlaneFrame.toImplementation());

  // Compute transformations matrix time derivative.
  const auto velToRatesDiff = zmp::getMatrixAngularVelocityInInertialFrameToAngularRatesZyxTimeDerivative(
      anglesZyxRefBaseToPlane, refAngularRatesZyxRefBaseInPlaneFrame);

  // Compute angular acceleration (time derivative of the above formula).
  const auto angularAccelerationBaseInPlaneFrame = orientationPlaneToWorld.inverseRotate(motionPlan_.getAngularAccelerationBaseInWorldFrame());

  // Derivative of (*)
  refAngularAccelZyxRefBaseInPlaneFrame = EulerAnglesZyxDiff(
      velToRatesDiff * angularVelocityBaseInPlaneFrame.toImplementation() +
      velToRates     * angularAccelerationBaseInPlaneFrame.toImplementation()
   );

  return true;
}

bool ComSupportControlZmp::updateVirtualPlaneFrame() {
  // Update virtual plane frame.
  if(!virtualPlaneFrame_.computeVirtualPlaneFrame(headingGenerator_, wholeBody_, terrain_, contactSchedule_)) {
    MELO_WARN_STREAM("[ComSupportControlZmp::updateVirtualPlaneFrame] Failed to compute virtual plane frame.");
    return false;
  }
  bool isWalking = (contactSchedule_.getStatus() == contact_schedule::Status::Walk ||
      contactSchedule_.getStatus() == contact_schedule::Status::SwitchGait ||
      contactSchedule_.getStatus() == contact_schedule::Status::SwitchToWalk);
  const RotationQuaternion& orientationPlaneToWorld = virtualPlaneFrame_.getPosePlaneToWorld().getRotation();
  terrainAdapter_.update(desiredHeightAboveGroundOnFlatTerrain_.getValue(), orientationPlaneToWorld, headingGenerator_, terrain_, isWalking);
  return true;
}

const WholeBody& ComSupportControlZmp::getWholeBody() const {
  return wholeBody_;
}

WholeBody* ComSupportControlZmp::getWholeBodyPtr() const {
  return &wholeBody_;
}

const TorsoPlannerZmp& ComSupportControlZmp::getTorsoPlannerZmp() const {
  return *zmpPlanner_;
}

TorsoPlannerZmp* ComSupportControlZmp::getTorsoPlannerZmpPtr() {
  return zmpPlanner_.get();
}

void ComSupportControlZmp::getMotionPlan(zmp::MotionPlan& motionPlan) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  motionPlan = motionPlan_;
}

void ComSupportControlZmp::getPosePlaneToWorldFromMotionPlan(Pose& posePlaneToWorld) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  posePlaneToWorld = motionPlan_.getVirtualPlaneFrame().getPosePlaneToWorld();
}

void ComSupportControlZmp::getPathRegularizerFromMotionPlan(zmp::PathRegularizer& pathRegularizerInPlaneFrame) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  pathRegularizerInPlaneFrame = motionPlan_.getPathRegularizerInPlaneFrame();
}

void ComSupportControlZmp::getCogTrajectroyFromMotionPlan(zmp::ComStateHandler& cogTrajectoryInPlaneFrame) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  cogTrajectoryInPlaneFrame = motionPlan_.getComStateInPlaneFrame();
}

void ComSupportControlZmp::getPredictedThighPositionUsingPathRegularizer(
    const contact_schedule::LegEnumAnymal& legId,
    double time,
    Position& positionWorldToLimbThighAtTimeInWorldFrame) const {
  const auto& leg = wholeBody_.getLegs().get(static_cast<unsigned int>(legId));

  // Get desired velocity twist.
  const LinearVelocity linearVelocityTargetInControlFrame(
      wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame().x(),
      wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame().y(),
      0.0
   );

  const LocalAngularVelocity angularVelocityTargetInControlFrame(
      0.0,
      0.0,
      wholeBody_.getTorso().getDesiredState().getAngularVelocityBaseInControlFrame().z()
  );

  // Rotations.
  const auto& orientationWorldToControl = wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const auto& orientationWorldToBase = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase();
  const auto orientationBaseToControl = orientationWorldToControl * orientationWorldToBase.inverted();

  // Get most recent path regularizer from motion plan.
  zmp::PathRegularizer pathRegularizerInPlaneFrame;
  Pose posePlaneToWorld;
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
    pathRegularizerInPlaneFrame = motionPlan_.getPathRegularizerInPlaneFrame();
    posePlaneToWorld = motionPlan_.getVirtualPlaneFrame().getPosePlaneToWorld();
  }
  const double optInitTime = pathRegularizerInPlaneFrame.getContainerTime();
  const double optFinalTime = pathRegularizerInPlaneFrame.getContainerDuration();

  // Velocity projection.
  const Position& positionBaseToThighInBaseFrame = leg.getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame();
  Position positionBaseToThighInControlFrame = orientationBaseToControl.rotate(positionBaseToThighInBaseFrame);
  positionBaseToThighInControlFrame.z() = 0.0;
  const Position desiredLinearVelocityInBaseFrame = orientationBaseToControl.inverseRotate(Position(
      linearVelocityTargetInControlFrame.toImplementation() +
      angularVelocityTargetInControlFrame.toImplementation().cross(positionBaseToThighInControlFrame.toImplementation())
  ));

  // Add time offset.
  time += optInitTime;

  // Clip to constraints.
  const double timeUnpredicted = std::fmax(time - optFinalTime, 0.0);
  robot_utils::boundToRange(&time, 0.0, optFinalTime);

  // Predict orientations.
  const RotationQuaternion orientationBaseAtTimeToPlane = static_cast<RotationQuaternion>(pathRegularizerInPlaneFrame.getAnglesZyxBaseToPlaneAtTime(time));
  const RotationQuaternion orientationBaseAtTimeToWorld = posePlaneToWorld.getRotation() * orientationBaseAtTimeToPlane;

  // Predict target position (Notice that path regularizer is added to the target location).
  Position positionBaseToTargetInWorldFrame;
  getPositionBaseToTargetPointInWorldFrame(wholeBody_, positionBaseToTargetInWorldFrame);
  const Position positionWorldToTargetAtTimeInWorldFrame = posePlaneToWorld.transform(pathRegularizerInPlaneFrame.getPositionPlaneToComInPlaneFrameAtTime(time));
  const Position positionWorldToBaseAtTimeInWorldFrame = positionWorldToTargetAtTimeInWorldFrame - positionBaseToTargetInWorldFrame;

  // Predicted thigh position at time.
  positionWorldToLimbThighAtTimeInWorldFrame =
      positionWorldToBaseAtTimeInWorldFrame +
      orientationBaseAtTimeToWorld.rotate(positionBaseToThighInBaseFrame + desiredLinearVelocityInBaseFrame * timeUnpredicted);
}

bool ComSupportControlZmp::isFirstSupportPolygonNew() {
  /*
   * Returns true if the first support polygon in the list of support polygon has appeared
   * for the very first time.
   */

  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  if (isFirstSupportPolygonNewPin_) {
    if (!motionPlan_.isFirstSupportPolygonNew()) { isFirstSupportPolygonNewPin_ = false; }
    return false;
  }

  else {
    if (motionPlan_.isFirstSupportPolygonNew()) { isFirstSupportPolygonNewPin_ = true; }
    return motionPlan_.isFirstSupportPolygonNew();
  }
}

bool ComSupportControlZmp::isFlightPhase() const {
  for (unsigned int legId=0u; legId<legs_.size(); ++legId) {
    if(isLegGrounded(legId))  { return false; }
  }
  return true;
}


bool ComSupportControlZmp::isLegGrounded(unsigned int legId) const {
  return ( (legs_.get(legId).getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
           (legs_.get(legId).getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant) );
}

bool ComSupportControlZmp::computeVelocityProjectionInPlaneFrame(
    Position& desiredVelocityProjectionInPlaneFrame,
    LinearVelocity& desiredLinearVelocityTargetInPlaneFrame,
    LinearVelocity& desiredLinearVelocityAtVelocityProjectionInPlaneFrame,
    LinearAcceleration& desiredLinearAccelerationTargetInPlaneFrame,
    LinearAcceleration& desiredLinearAccelerationAtVelocityProjectionInPlaneFrame,
    double optimizationHorizonInSeconds) {
  /*
   * Note: Computations are done in virtual plane frame, i.e.
   * the x axis is aligned with the desired velocity vector.
   */

  const auto& desiredLinearVelocityInControlFrame  = wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame();
  const auto& desiredAngularVelocityInControlFrame = wholeBody_.getTorso().getDesiredState().getAngularVelocityBaseInControlFrame();
  const double desiredLinearVelocityNorm = desiredLinearVelocityInControlFrame.toImplementation().norm();
  const double desiredAngularVelocityNorm = desiredAngularVelocityInControlFrame.z();

  // ToDO: on slopes this does not work!!
  // ToDO: Improve this function.

  // If the robot should be standing but is disturbed: Walk in direction of the disturbance.
  if (desiredLinearVelocityNorm<=1e-7 && desiredAngularVelocityNorm<=1e-7 &&
      contactSchedule_.getStatus() == contact_schedule::Status::Walk
      && weightDisturbanceFollower_ > 0.0) {

    // Get Base velocity and rotate to world frame.
    auto linearVelocityInControlFrame = wholeBody_.getTorso().getMeasuredState().inControlFrame().getLinearVelocityBaseInControlFrame();
    linearVelocityInControlFrame.y() *= 0.5;
    linearVelocityInControlFrame.z() = 0.0;
    const auto& orientationWorldToControl = wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
    const LinearVelocity linearVelocityBaseInWorldFrame = orientationWorldToControl.inverseRotate(linearVelocityInControlFrame);

    // Scale and smooth in world frame.
    const LinearVelocity filteredLinearVelocityBaseInWorldFrame = filterVelocityProjection_.advance(
        linearVelocityBaseInWorldFrame*weightDisturbanceFollower_);

    // Rotate to plane frame.
    desiredLinearVelocityTargetInPlaneFrame = virtualPlaneFrame_.getPosePlaneToWorld().getRotation().inverseRotate(filteredLinearVelocityBaseInWorldFrame);
    desiredLinearVelocityTargetInPlaneFrame.z() = 0.0;

    // Velocity projection.
    desiredLinearVelocityAtVelocityProjectionInPlaneFrame.setZero();
    desiredVelocityProjectionInPlaneFrame = optimizationHorizonInSeconds * static_cast<Position>(0.5*desiredLinearVelocityTargetInPlaneFrame);

    // Acceleration.
    desiredLinearAccelerationTargetInPlaneFrame.setZero();
    desiredLinearAccelerationAtVelocityProjectionInPlaneFrame.setZero();

    return true;
  } else { filterVelocityProjection_.reset(LinearVelocity::Zero()); }


  const auto orientationPlaneToControl =
      wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl()*
      virtualPlaneFrame_.getPosePlaneToWorld().getRotation();
  desiredLinearVelocityTargetInPlaneFrame = orientationPlaneToControl.inverseRotate(desiredLinearVelocityInControlFrame);



  // ToDo: tune this properly. Might be responsible for vibrations at while going back to stand.
  if(desiredLinearVelocityNorm>1e-7 && desiredAngularVelocityNorm>1e-7) {
    if (desiredAngularVelocityInControlFrame.z() == 0.0) { return false; }
    const double yawAngle = desiredAngularVelocityInControlFrame.z()*optimizationHorizonInSeconds;
    const RotationQuaternion orientationNextFootprintToPlane(EulerAnglesZyx(yawAngle, 0.0, 0.0));

    // Velocity.
    desiredLinearVelocityAtVelocityProjectionInPlaneFrame = orientationNextFootprintToPlane.rotate(desiredLinearVelocityTargetInPlaneFrame);

    // Computations are done in a frame that is aligned with the reference linear velocity.
    RotationQuaternion orientationControlToComputation;
    try {
      orientationControlToComputation.setFromVectors<Eigen::Vector3d>(
          desiredLinearVelocityInControlFrame.toImplementation().normalized(),
          Eigen::Vector3d::UnitX()
      );
    } catch (const std::runtime_error& error) {
      MELO_WARN_STREAM(error.what() << "setFromVectors in ComSupportControlZmp::computeVelocityProjectionInPlaneFrame()." << std::endl);
      orientationControlToComputation.setIdentity();
    }


    const auto desiredLinearVelocityInComputationFrame = orientationControlToComputation.rotate(desiredLinearVelocityInControlFrame);


    // Velocity projection: v = w*r
    const double radius = desiredLinearVelocityInComputationFrame.x() / desiredAngularVelocityInControlFrame.z();
    const Position desiredVelocityProjectionInComputationFrame(
        radius * std::sin(yawAngle),
        radius * (1.0 - std::cos(yawAngle)),
        0.0
    );
    const Position desiredVelocityProjectionInControlFrame = orientationControlToComputation.inverseRotate(desiredVelocityProjectionInComputationFrame);
    desiredVelocityProjectionInPlaneFrame = orientationPlaneToControl.inverseRotate(desiredVelocityProjectionInControlFrame);

    // Acceleration: a = v^2/r = w^2*r
    const double accelNorm = boost::math::pow<2>(desiredAngularVelocityInControlFrame.z()) * radius;
    const auto desiredLinearAccelerationTargetInComputationFrame = LinearAcceleration(0.0, accelNorm, 0.0);
    const LinearAcceleration desiredLinearAccelerationTargetInControlFrame = orientationControlToComputation.inverseRotate(desiredLinearAccelerationTargetInComputationFrame);
    desiredLinearAccelerationTargetInPlaneFrame = orientationPlaneToControl.inverseRotate(desiredLinearAccelerationTargetInControlFrame);
    desiredLinearAccelerationAtVelocityProjectionInPlaneFrame = orientationNextFootprintToPlane.rotate(desiredLinearAccelerationTargetInPlaneFrame);
  }

  else {
    desiredLinearVelocityAtVelocityProjectionInPlaneFrame = desiredLinearVelocityTargetInPlaneFrame;
    desiredVelocityProjectionInPlaneFrame =  static_cast<Position>(desiredLinearVelocityTargetInPlaneFrame)*optimizationHorizonInSeconds;
    desiredLinearAccelerationTargetInPlaneFrame.setZero();
    desiredLinearAccelerationAtVelocityProjectionInPlaneFrame.setZero();
  }

  return true;
}

bool ComSupportControlZmp::computePositionWorldToReferencePathInWorldFrame(
  Position& positionWorldToReferencePathInWorldFrame) const {
  if (contactSchedule_.getStatus() == contact_schedule::Status::SwitchToStand ||
      contactSchedule_.getStatus() == contact_schedule::Status::ExecuteOneCycle ||
      contactSchedule_.getStatus() == contact_schedule::Status::Stand) {
    if(!headingGenerator_.computeCurrentFootPrintCenterInWorldFrame(positionWorldToReferencePathInWorldFrame)) { return false; }
  } else {
    positionWorldToReferencePathInWorldFrame = getPositionWorldToTargetInWorldFrame(wholeBody_);
  }
  return true;
}


bool ComSupportControlZmp::computePathKnotsInPlaneFrame(
    positionVector& positionKnotInPlaneFrame,
    velocityVector& velocityKnotInPlaneFrame,
    accelerationVector& accelerationKnotInPlaneFrame,
    std::vector<double>& duration,
    double optimizationHorizonInSeconds,
    Position& desiredVelocityProjectionInPlaneFrame) {

  // Compute velocity projection.
  LinearVelocity desiredLinearVelocityTargetInPlaneFrame;
  LinearVelocity desiredLinearVelocityAtVelocityProjectionInPlaneFrame;
  LinearAcceleration desiredLinearAccelerationTargetInPlaneFrame;
  LinearAcceleration desiredLinearAccelerationAtVelocityProjectionInPlaneFrame;
  if(!computeVelocityProjectionInPlaneFrame(
      desiredVelocityProjectionInPlaneFrame,
      desiredLinearVelocityTargetInPlaneFrame,
      desiredLinearVelocityAtVelocityProjectionInPlaneFrame,
      desiredLinearAccelerationTargetInPlaneFrame,
      desiredLinearAccelerationAtVelocityProjectionInPlaneFrame,
      optimizationHorizonInSeconds)) { return false; }

  // Resize.
  positionKnotInPlaneFrame.resize(2); velocityKnotInPlaneFrame.resize(2);
  accelerationKnotInPlaneFrame.resize(2); duration.resize(1);

  // Set acceleration knots.
  duration.front()                = optimizationHorizonInSeconds;
  accelerationKnotInPlaneFrame[0] = desiredLinearAccelerationTargetInPlaneFrame;
  accelerationKnotInPlaneFrame[1] = desiredLinearAccelerationAtVelocityProjectionInPlaneFrame;

  // Set velocity knots.
  velocityKnotInPlaneFrame[0] = desiredLinearVelocityTargetInPlaneFrame;
  velocityKnotInPlaneFrame[1] = desiredLinearVelocityAtVelocityProjectionInPlaneFrame;

  // Get desired state
  const auto& posePlaneToWorld = virtualPlaneFrame_.getPosePlaneToWorld();
  const auto& positionOffsetInWorldFrame = wholeBody_.getTorso().getDesiredState().getDesiredPositionOffsetInWorldFrame();
  const auto positionOffsetInPlaneFrame = posePlaneToWorld.getRotation().inverseRotate(positionOffsetInWorldFrame);

  // Set initial position knot.
  Position positionWorldToReferencePathInWorldFrame;
  const Position& positionPlaneToDesiredTargetHeightInPlaneFrame = terrainAdapter_.getPositionPlaneToDesiredTargetHeightInPlaneFrame();
  if(!computePositionWorldToReferencePathInWorldFrame(positionWorldToReferencePathInWorldFrame)) { return false; }
  Position positionPlaneToReferencePathOnPlaneInPlaneFrame = posePlaneToWorld.inverseTransform(positionWorldToReferencePathInWorldFrame);
  positionPlaneToReferencePathOnPlaneInPlaneFrame.z() = 0.0;
  positionKnotInPlaneFrame[0] =
      positionPlaneToReferencePathOnPlaneInPlaneFrame +
      positionPlaneToDesiredTargetHeightInPlaneFrame +
      positionOffsetInPlaneFrame;

  // Set final position and velocity knots.
  positionKnotInPlaneFrame[1] = positionKnotInPlaneFrame.front() + desiredVelocityProjectionInPlaneFrame;

  return true;
}

bool ComSupportControlZmp::computePathKnotsInPlaneFrame(
    eulerAnglesZyxVector& anglesZyxPathToPlane,
    eulerAnglesZyxDiffVector& angularRatesZyxPathInPlaneFrame,
    std::vector<double>& duration,
    double optimizationHorizonInSeconds,
    EulerAnglesZyx& orientationDesiredVelocityOffset) {

  // Get desired state.
  const auto& desiredAngularVelocityBaseInControlFrame = wholeBody_.getTorso().getDesiredState().getAngularVelocityBaseInControlFrame();
  const auto& orientationDesiredAngleOffset = EulerAnglesZyx(wholeBody_.getTorso().getDesiredState().getDesiredOrientationOffset().inverted()).getUnique();
  orientationDesiredVelocityOffset = EulerAnglesZyx(
      desiredAngularVelocityBaseInControlFrame.z()*optimizationHorizonInSeconds,
      desiredAngularVelocityBaseInControlFrame.y()*optimizationHorizonInSeconds,
      desiredAngularVelocityBaseInControlFrame.x()*optimizationHorizonInSeconds
  );

  // Resize.
  anglesZyxPathToPlane.resize(2); angularRatesZyxPathInPlaneFrame.resize(2);
  duration.resize(1); duration.front() = optimizationHorizonInSeconds;

  // Compute orientation control to plane.
  const auto orientationControlToPlane =  EulerAnglesZyx(
      wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl()*
      virtualPlaneFrame_.getPosePlaneToWorld().getRotation()
  ).inverted().getUnique();

  const auto orientationFinalControlToPlane = (
      orientationControlToPlane*
      orientationDesiredAngleOffset*
      orientationDesiredVelocityOffset
  ).getUnique();

  // Set orientation.
  /*
   * This orientation is currently forwarded to torso->desiredOrientationBaseToWorld.
   *
   * The behavior is particular for the YAW component:
   * - While walking, yaw is set from orientationControlToPlane, which is itself read from the measured torso orientation, therefore
   *   ensuring that desiredYaw == measuredYaw. Then, the whole-body controller has (artificially) zero error, and won't try to control the
   *   torso's yaw.
   * - While standing, yaw is read from the heading "generator", which itself computes it from all four measured foothold positions. Then
   *   desiredYaw != measuredYaw and the whole-body controller controls the yaw orientation as expected.
   *
   */
  const RotationQuaternion standingOrientationPathToPlane = getOrientationFootprintHeadingToPlane();
  const RotationQuaternion walkingOrientationPathToPlane = RotationQuaternion(orientationControlToPlane);
  RotationQuaternion orientationPathToPlaneRQ;
  switch (contactSchedule_.getStatus()) {
    case contact_schedule::Status::Stand: {
      double interpRatio = std::max(0., std::min(1., timeSpentStanding_ / switchToStanceDuration_));
      orientationPathToPlaneRQ = walkingOrientationPathToPlane.interpolate(standingOrientationPathToPlane, interpRatio);
    } break;
    case contact_schedule::Status::SwitchToStand: {
      timeSpentStanding_ = 0.;
      orientationPathToPlaneRQ = walkingOrientationPathToPlane;
    } break;
    case contact_schedule::Status::Walk:
    default:
      orientationPathToPlaneRQ = walkingOrientationPathToPlane;
      break;
  }
  EulerAnglesZyx orientationPathToPlane = EulerAnglesZyx(orientationPathToPlaneRQ).getUnique();

  anglesZyxPathToPlane[0] = (orientationPathToPlane * orientationDesiredAngleOffset).getUnique();
  anglesZyxPathToPlane[1] = (anglesZyxPathToPlane[0] * orientationDesiredVelocityOffset).getUnique();  // TODO(scaron): make homogeneous

  // Set angular rates.
  angularRatesZyxPathInPlaneFrame[0] = EulerAnglesZyxDiff(
      zmp::getMatrixAngularVelocityInInertialFrameToAngularRatesZyx(orientationControlToPlane) *
      orientationControlToPlane.rotate(desiredAngularVelocityBaseInControlFrame).toImplementation()
   );

  angularRatesZyxPathInPlaneFrame[1] = EulerAnglesZyxDiff(
      zmp::getMatrixAngularVelocityInInertialFrameToAngularRatesZyx(orientationFinalControlToPlane) *
      orientationFinalControlToPlane.rotate(desiredAngularVelocityBaseInControlFrame).toImplementation()
   );

  return true;
}

RotationQuaternion ComSupportControlZmp::getOrientationFootprintHeadingToPlane() const {
  // Compute orientation plane to control.
  const auto& orientationWorldToControl = wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();
  const auto eulerAnglesZyxPlaneToControl = EulerAnglesZyx(
      orientationWorldToControl *
      virtualPlaneFrame_.getPosePlaneToWorld().getRotation()
  ).getUnique();

  // Compute yaw angle between plane frame and heading direction of the footprint.
  RotationQuaternion orientationWorldToHeading;
  headingGenerator_.getOrientationWorldToCurrentFootprintHeading(orientationWorldToHeading);
  const auto eulerAnglesZyxPlaneToHeading = EulerAnglesZyx(
      orientationWorldToHeading *
      virtualPlaneFrame_.getPosePlaneToWorld().getRotation()
  ).getUnique();

  // Compute orientation plane to heading on terrain plane.
  const auto eulerAnglesZyxPlaneToHeadingOnTerrain = EulerAnglesZyx(
      eulerAnglesZyxPlaneToHeading.yaw(),
      eulerAnglesZyxPlaneToControl.pitch(),
      eulerAnglesZyxPlaneToControl.roll()
  );

  return RotationQuaternion(eulerAnglesZyxPlaneToHeadingOnTerrain).inverted();
}

void ComSupportControlZmp::setCurrentFeetConfigurationInWorldFrame() {
  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    const auto legIdInt = static_cast<unsigned int>(legEnum);
    feetConfigurationInWorldFrame_[legEnum] = legs_.get(legIdInt).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  }
}

void ComSupportControlZmp::printSupportPolygonSequence(double rate) const {
  std::stringstream msg;
  msg << std::left << "List of support polygons:" << std::endl;
  msg << std::setw(10) << "id"
      << std::setw(10) << "d[s]"
      << std::setw(10) << "type"
      << std::setw(10) << "vertices"
      << std::endl;
  unsigned int polId = 0;
  for (const auto& polygon : supportPolygons_) {
    msg << std::setw(10) << std::to_string(polId)
        << std::setw(10) << std::to_string(polygon.getDuration())
        << std::setw(10) << zmp::PolygonTypeNamesMap[polygon.getPolygonType()]
        << std::setw(10) << std::to_string(polygon.getPolygon().getNumVertices())
        << std::endl;
    polId++;
  }

  if (rate>0.0) { MELO_INFO_THROTTLE_STREAM(rate, msg.str()); }
  else { MELO_INFO_STREAM(msg.str()); }
}

double ComSupportControlZmp::computeApproachDuration() const {
  return 0.8;
}

void ComSupportControlZmp::setDesiredHeightAboveGroundOnFlatTerrain(double desiredHeightAboveGroundOnFlatTerrain) noexcept {
  desiredHeightAboveGroundOnFlatTerrain_.setValue(desiredHeightAboveGroundOnFlatTerrain);
}

double ComSupportControlZmp::getNominalDesiredHeightAboveGroundOnFlatTerrain() const noexcept {
  return nominalDesiredHeightAboveGroundOnFlatTerrain_;
}

bool ComSupportControlZmp::updateFootholdTrackingOffsetIndicator() {
  const double desVelocityNorm = robot_utils::computeNorm(
      wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame().x(),
      wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame().y(),
      wholeBody_.getTorso().getDesiredState().getAngularVelocityBaseInControlFrame().z()
  );
  const double maxAllowedTrackingOffset = nominalFootholdTrackingOffset_ + footholdTrackingOffsetGain_ * desVelocityNorm;

  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    const auto& leg = legs_.get(static_cast<unsigned int>(legEnum));

    if(leg.didTouchDownAtLeastOnceDuringStance()) {
      /*
       * Compute tracking offset of footholds during stance phase:
       * Error e_xy is computed in xy plane in world frame. If a leg is regaining or touches down too early, e_xy is typically
       * large. To avoid to trigger the tip over objective in this case, we subtract the vertical error e_z. This means, the tip over objective
       * can only be triggered, if the horizontal error is larger than the vertical error.
       */
      const Eigen::Vector3d vectorPreviousDesFootholdToMeasFootholdInWorldFrame = (
          leg.getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() -
          positionWorldToPreviousDesiredFootholdInWorldFrame_[legEnum]).toImplementation();
      const double footholdTrackingOffset = vectorPreviousDesFootholdToMeasFootholdInWorldFrame.head<2>().norm() - std::fabs(vectorPreviousDesFootholdToMeasFootholdInWorldFrame.z());

      // Transform into offset indicator. The larger the des velocity, the more we penalize the tracking offset.
      footholdTrackingOffsetIndicator_[legEnum] =  (1.0 + desVelocityNorm) * std::fmax(footholdTrackingOffset - maxAllowedTrackingOffset, 0.0);
    }

    // Update previous desired foothold.
    else if(leg.didLiftOffAtLeastOnceDuringSwing()) {
      positionWorldToPreviousDesiredFootholdInWorldFrame_[legEnum] = leg.getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();
    }

    // Filter tracking offset.
    filteredFootholdTrackingOffsetIndicator_[legEnum].advance(footholdTrackingOffsetIndicator_[legEnum]);
  }

  return true;
}


} /* namespace loco */
