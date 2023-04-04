/*
 * ZmpInteractiveTest.cpp
 *
 *  Created on: Aug 14, 2018
 *  Author: Markus Staeuble
 */

#include <kindr/common/gtest_eigen.hpp>
#include <cosmo_ros/cosmo_ros.hpp>


#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <kindr_ros/kindr_ros.hpp>

#include <boost/bind.hpp>
#include <string>

#include <motion_generation_utils/typedefs.hpp>

#include <anymal_model/AnymalModel.hpp>
#include <anymal_model/AnymalState.hpp>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <any_measurements/Pose.hpp>
#include <any_measurements/Twist.hpp>

#include <zmp_optimizer/ZmpOptimizerDynamicWalk.hpp>
#include <zmp_optimizer/QPReader.hpp>

#include "motion_generation_ros/SupportPolygonSequence.hpp"
#include <motion_generation_ros/MotionPlanVisualizer.hpp>

#include <loco_anymal/loco_anymal.hpp>
#include <loco_anymal/common/WholeBodyAnymal.hpp>

#include <param_io/get_param.hpp>

#include <loco/torso_control/ComSupportControlZmp.hpp>
#include <loco/common/TerrainModelFreePlane.hpp>
#include <loco/heading_generation/HeadingGenerator.hpp>
#include <loco_anymal/heading_generation/HeadingGeneratorAnymal.hpp>
#include <loco/common/ParameterSet.hpp>

#include <anymal_model_ros/conversions.hpp>
#include <anymal_model_ros/initializations.hpp>

#include <loco/torso_control/TorsoControlZmp.hpp>
#include <loco/limb_coordinator/LimbCoordinatorBase.hpp>
#include <loco/foot_placement_strategy/FootPlacementStrategyBase.hpp>
#include <loco/mission_control/MissionControlZmp.hpp>
#include <loco/swing_trajectory_generation/SwingTrajectoryGeneratorModule.hpp>
#include <loco/foothold_generation/FootholdGeneratorInvertedPendulumBase.hpp>
#include <loco/foothold_generation/FootholdGeneratorOptimizedQPBase.hpp>
#include <loco/foot_placement_strategy/FootPlacementStrategyOptimized.hpp>
#include <loco/foothold_generation/FootholdGeneratorInvertedPendulumMotionGen.hpp>
#include <loco/contact_force_distribution/constraints/ForceLimitsConstraint.hpp>
#include "loco/terrain_perception/TerrainPerceptionFreePlane.hpp"
#include "loco/contact_detection/ContactDetectorFeedThrough.hpp"
#include <loco/contact_force_distribution/ContactForceDistribution.hpp>
#include <loco/contact_force_distribution/ContactForceDistributionDummy.hpp>
#include <loco/contact_force_distribution/constraints/ForceLimitsConstraint.hpp>
#include <loco/contact_force_distribution/constraints/FrictionConstraint.hpp>
#include <loco/locomotion_controller/LocomotionController.hpp>
#include "loco_anymal/motion_control/ImpedanceAndVirtualModelController.hpp"
#include "anymal_ctrl_dynamic_gaits/modules/LimbCoordinatorOpt.hpp"
#include "loco_anymal/motion_control/WholeBodyController.hpp"
#include "anymal_ctrl_dynamic_gaits/modules/SwingTrajectoryGeneratorSplineModule.hpp"
#include "swing_trajectory_generation/SwingTrajectoryGeneratorSplineOptimized.hpp"

#include <motion_generation_msgs/GetAvailableGaits.h>
#include <motion_generation_msgs/SwitchGait.h>

#include "loco_ros_anymal/visualization/GaitPatterns.hpp"
#include "motion_generation_ros/FootholdPlanVisualizer.hpp"
#include <zmp_anymal/ContactScheduleZmpTest.hpp>

#include <parameter_handler/parameter_handler.hpp>
#include "parameter_handler_ros/parameter_handler_ros.hpp"

using AD = anymal_description::AnymalDescription;

class FootMarker : public visualization_msgs::InteractiveMarker
{
 public:
  FootMarker(const std::string& limbName, const motion_generation::Position& position)
      : scale_(0.1),
        footRadius_(0.06),
        frameId_("odom")
  {
    color_.r = 1.0;
    color_.a = 1.0;
    setup(limbName, position);
  }

  ~FootMarker() = default;

 private:

  void setup(const std::string& limbName, const motion_generation::Position& position)
  {
    controls.clear();
    menu_entries.clear();

    header.frame_id = frameId_;
    name = limbName;
    scale = scale_;
    motion_generation::Pose initialPose;
    initialPose.getPosition() = position;
    kindr_ros::convertToRosGeometryMsg(initialPose, pose);

    visualization_msgs::InteractiveMarkerControl mainControl;
    mainControl.name = name + "_menu";
    mainControl.always_visible = true;
    mainControl.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
    mainControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

    visualization_msgs::Marker sphereMarker;
    sphereMarker.type = visualization_msgs::Marker::SPHERE;
    sphereMarker.scale.x = footRadius_;
    sphereMarker.scale.y = footRadius_;
    sphereMarker.scale.z = footRadius_;
    sphereMarker.color = color_;
    mainControl.markers.push_back(sphereMarker);
    controls.push_back(mainControl);

    // Add interactive controls.
    visualization_msgs::InteractiveMarkerControl positionControl;
    positionControl.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

    // Move about x-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 1;
    positionControl.orientation.y = 0;
    positionControl.orientation.z = 0;
    positionControl.name = "move_x";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(positionControl);

    // Move about y-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 0;
    positionControl.orientation.y = 1;
    positionControl.orientation.z = 0;
    positionControl.name = "move_y";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(positionControl);

    // Move about z-axis.
    positionControl.orientation.w = 1;
    positionControl.orientation.x = 0;
    positionControl.orientation.y = 0;
    positionControl.orientation.z = 1;
    positionControl.name = "move_z";
    positionControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    controls.push_back(positionControl);
  }

  double scale_;
  double footRadius_;
  std_msgs::ColorRGBA color_;
  std::string frameId_;
};

class PoseOptimizationHelper
{
public:
  static constexpr double dt = 0.01;

  struct EnumClassHash
  {
    template<typename T>
    std::size_t operator()(T t) const
    {
      return static_cast<std::size_t>(AD::mapKeyEnumToKeyId(t));
    }
  };

  using Stance = std::unordered_map<AD::LimbEnum, motion_generation::Position, EnumClassHash>;
  using TwistShm = any_measurements::Twist;
  using TwistRos = geometry_msgs::TwistStamped;
  using PoseShm = any_measurements::Pose;
  using PoseRos = geometry_msgs::PoseStamped;

  PoseOptimizationHelper()
      : nodeHandle_("~"),
        virtualPlaneFrame_(zmp::vpf::VirtualPlaneFrameEnum::alignedWithRefVel) {
    anymalStatePublisher_ = nodeHandle_.advertise<anymal_msgs::AnymalState>("/state_estimator/anymal_state", 1);

    nominalStanceInBaseFrame_[AD::LimbEnum::LF_LEG] = motion_generation::Position(0.33, 0.22, -0.48);
    nominalStanceInBaseFrame_[AD::LimbEnum::RF_LEG] = motion_generation::Position(0.33, -0.22, -0.48);
    nominalStanceInBaseFrame_[AD::LimbEnum::LH_LEG] = motion_generation::Position(-0.33, 0.22, -0.48);
    nominalStanceInBaseFrame_[AD::LimbEnum::RH_LEG] = motion_generation::Position(-0.33, -0.22, -0.48);

    std::vector<loco::Vector, Eigen::aligned_allocator<loco::Vector>> colors_;
    colors_.reserve(18);
    colors_.emplace_back(loco::Vector(230.0/255.0, 25.0/255.0,  75.0/255.0));    // Red
    colors_.emplace_back(loco::Vector(60.0/255.0,  180.0/255.0, 75.0/255.0));    // Green
    colors_.emplace_back(loco::Vector(255.0/255.0, 225.0/255.0, 25.0/255.0));    // Yellow
    colors_.emplace_back(loco::Vector(0.0/255.0,   130.0/255.0, 200.0/255.0));   // Blue
    colors_.emplace_back(loco::Vector(245.0/255.0, 130.0/255.0, 48.0/255.0));    // Orange
    colors_.emplace_back(loco::Vector(145.0/255.0, 30.0/255.0,  180.0/255.0));   // Purple
    colors_.emplace_back(loco::Vector(70.0/255.0,  240.0/255.0, 240.0/255.0));   // Cyan
    colors_.emplace_back(loco::Vector(240.0/255.0, 50.0/255.0,  230.0/255.0));   // Maganta
    colors_.emplace_back(loco::Vector(210.0/255.0, 245.0/255.0, 60.0/255.0));    // Lime
    colors_.emplace_back(loco::Vector(0.0/255.0,   128.0/255.0, 128.0/255.0));   // Teal
    colors_.emplace_back(loco::Vector(230.0/255.0, 190.0/255.0, 255.0/255.0));   // Lavender
    colors_.emplace_back(loco::Vector(170.0/255.0, 110.0/255.0, 40.0/255.0));    // Brow
    colors_.emplace_back(loco::Vector(255.0/255.0, 250.0/255.0, 200.0/255.0));   // Beige
    colors_.emplace_back(loco::Vector(128.0/255.0, 0.0/255.0,   0.0/255.0));     // Maroon
    colors_.emplace_back(loco::Vector(170.0/255.0, 255.0/255.0, 195.0/255.0));   // Mint
    colors_.emplace_back(loco::Vector(128.0/255.0, 128.0/255.0, 0.0/255.0));     // Olive
    colors_.emplace_back(loco::Vector(255.0/255.0, 215.0/255.0, 180.0/255.0));   // Coral
    colors_.emplace_back(loco::Vector(0.0/255.0,   0.0/255.0,   128.0/255.0));   // Navy

  // Set color vectors.
    supportPolygonSequence_.setColorVector(colors_);
    motionPlanVisualizer_.setColorVector(colors_);
    motionPlanVisualizer_.initialize(nodeHandle_);
    supportPolygonSequence_.initialize(nodeHandle_, "/dynamic_gaits_ros/support_polygons");

    desiredRobotTwist_.setZero();
    desiredRobotTwist_.getTranslationalVelocity().toImplementation() = Eigen::Vector3d(0.2, 0.0, 0.0);

    anymal_model_ros::initialize(anymalStateRos_);
    footholdPlanVisualizer_.setColorVector(colors_);

    anymalModel_ = new anymal_model::AnymalModel();
    anymalModelDesired_ = new anymal_model::AnymalModel();

    std::string anymalUrdfDescription = param_io::param<std::string>(nodeHandle_, "/anymal_description", "");

    anymalModel_->initializeFromUrdf(anymalUrdfDescription);
    anymalModelDesired_->initializeFromUrdf(anymalUrdfDescription);

    legs_ = loco_anymal::make_unique_leg_group(*anymalModel_, *anymalModelDesired_, false);
    torso_.reset(new loco_anymal::TorsoAnymal("torso", *anymalModel_));
    wholeBody_.reset(new loco_anymal::WholeBodyAnymal(*anymalModel_, *torso_, *legs_, true));

    contactSchedule_.reset(new loco::ContactScheduleZmpTest(*wholeBody_));

    // Create the heading generator.
    headingGenerator_.reset(new loco_anymal::HeadingGeneratorAnymal(*wholeBody_));

    terrainModel_.reset(new loco::TerrainModelFreePlane());

    // Create the center of mass controller.
    modulesParameterSet_.reset(new loco::ParameterSet());

    bool isRealRobot = false;
    std::string location = param_io::param<std::string>(nodeHandle_, "crawling_config_path", "");

    const std::string parameterFile = location + "/DefaultParams" + (isRealRobot ? "" : "Sim") + ".xml";

    if (!modulesParameterSet_->loadXmlDocument(parameterFile)) {
      MELO_ERROR_STREAM("Could not load parameter file: " << parameterFile);
    } else {
      MELO_INFO_STREAM("Loaded file: " << parameterFile);
    }

    terrainAdapter_.reset(new loco::TerrainAdapter());

    comControl_.reset(new loco::ComSupportControlZmp(
      *wholeBody_, *contactSchedule_, *headingGenerator_, *terrainModel_, *terrainAdapter_, isRealRobot));

    // Create the torso controller.
    torsoController_.reset(new loco::TorsoControlZmp(*wholeBody_, *terrainModel_, *comControl_));

    bool useOptimizedSwingTrajectory_ = true;
    // Create the swing foot motion generator.
    if (useOptimizedSwingTrajectory_) {
      swingTrajectoryGenerator_.reset(new loco::SwingTrajectoryGeneratorSplineOptimized(*wholeBody_, *terrainModel_, *contactSchedule_));
    } else {
      swingTrajectoryGenerator_.reset(new loco::SwingTrajectoryGeneratorSplineModule(*wholeBody_, *terrainModel_));
    }

    footholdGenerator_.reset(new loco::FootholdGeneratorInvertedPendulumMotionGen(*wholeBody_, *terrainModel_, *contactSchedule_));
    footholdGeneratorOptimized_.reset(new loco::FootholdGeneratorOptimizedInvPend(*wholeBody_));
    footPlacementStrategy_.reset(new loco::FootPlacementStrategyOptimized<loco::FootholdGeneratorOptimizedInvPend, loco::foothold_generator::FootholdPlanInvPend>(
      *wholeBody_, *terrainModel_, *swingTrajectoryGenerator_, *dynamic_cast<loco::FootholdGeneratorOptimizedInvPend*>(footholdGeneratorOptimized_.get()),
      *contactSchedule_, *headingGenerator_, *terrainAdapter_, *footholdGenerator_));

    contactDetector_.reset(new loco::ContactDetectorFeedThrough());
    limbCoordinator_.reset(new loco::LimbCoordinatorOpt(*wholeBody_));

    terrainPerception_.reset(
        new loco::TerrainPerceptionFreePlane(
            *terrainModel_, *wholeBody_, *headingGenerator_,
            loco::TerrainPerceptionFreePlane::EstimatePlaneInFrame::World,
            loco::TerrainPerceptionFreePlane::ControlFrameHeading::Hips));

    bool useWholeBodyController_ = true;

    if (useWholeBodyController_) {
      contactForceDistribution_.reset(new loco::ContactForceDistributionDummy());
    } else {
      std::unique_ptr<numopt_common::QuadraticProblemSolver> qpSolver(new numopt_quadprog::ActiveSetFunctionMinimizer());
      contactForceDistribution_.reset(new loco::ContactForceDistribution(*wholeBody_, *terrainModel_, std::move(qpSolver)));
      contactForceDistribution_->addConstraint(loco::ConstraintInterfacePtr(new loco::FrictionConstraint(*wholeBody_, *terrainModel_)));
      contactForceDistribution_->addConstraint(loco::ConstraintInterfacePtr(new loco::ForceLimitsConstraint<>(*wholeBody_, *terrainModel_)));
    }

    // Create the motion controller.
    if (useWholeBodyController_) {
      motionControl_.reset(new loco_anymal::WholeBodyController(
          *wholeBody_, *anymalModel_, *terrainModel_));
    } else {
      motionControl_.reset(
          new loco::ImpedanceAndVirtualModelController(
              dynamic_cast<loco_anymal::WholeBodyAnymal&>(*wholeBody_), *anymalModel_,
              *anymalModelDesired_, *contactForceDistribution_));
    }

    locomotionController_.reset(new loco::LocomotionController(
        *wholeBody_->getLegsPtr(), *wholeBody_->getTorsoPtr(), *terrainPerception_,
        *contactDetector_, *limbCoordinator_, *footPlacementStrategy_,
        *torsoController_, *motionControl_,
        modulesParameterSet_.get(), *contactSchedule_, *terrainModel_,
        *wholeBody_, *headingGenerator_));


    missionController_.reset(new loco::MissionControlZmp(
        &desiredRobotTwist_, &desiredRobotPose_, *wholeBody_, *contactSchedule_));


    // Get the parameter handle.
    const TiXmlHandle locomotionControllerHandle(modulesParameterSet_->getHandle().FirstChild("LocomotionController"));


    locomotionController_->setParameterSet(modulesParameterSet_.get());
    speedFilter_.loadParameters(modulesParameterSet_->getHandle());

    loco::Twist maxBaseTwist;
    maxBaseTwist.setVector(Eigen::Matrix<double, 6, 1>::Ones());
    speedFilter_.setMaximumBaseTwistInControlFrame(maxBaseTwist);

    anymal_model::AnymalState state;
    state.setZero();
    motion_generation::Pose startPose;
    startPose.setIdentity();
    startPose.getPosition().z() = 0.46;
    state.setPoseBaseToWorld(startPose);
    setDefaultJointPosition(state);

    anymalModel_->setState(state, true, false, false);
    anymalModelDesired_->setState(state, true, false, false);

    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      anymalModel_->getContactContainer()[contactEnum]->setState(AD::ContactStateEnum::CLOSED);
    }

    anymalStatePublisher_.publish(createRosMsg(state));

    locomotionController_->initialize(dt);
    missionController_->loadParameters(locomotionControllerHandle);
    missionController_->initialize(dt);

    velocityCommandsSubscriber_ = nodeHandle_.subscribe("/joy_manager/twist", 1, &PoseOptimizationHelper::velocityCommandsCallback, this);
    poseCommandsSubscriber_ = nodeHandle_.subscribe("/joy_manager/pose", 1, &PoseOptimizationHelper::poseCommandsCallback, this);

    missionController_->setTargetLocomotionMode(loco::MissionControlZmp::LocomotionMode::ModeWalking);

    getAvailableGatisService_ = nodeHandle_.advertiseService("/dynamic_gaits_ros/get_available_gaits", &PoseOptimizationHelper::getAvailableGaitsCallback, this);
    switchGaitService_ = nodeHandle_.advertiseService("/dynamic_gaits_ros/switch_gait", &PoseOptimizationHelper::switchGaitCallback, this);
    gaitPatternVisualizer_.initialize(nodeHandle_, "/loco_ros/gait_patterns");
    footholdPlanVisualizer_.initialize(nodeHandle_, "/dynamic_gaits_ros/foothold_plan");

    parameter_handler_ros::setParameterHandlerRos(&nodeHandle_);
    stridePhase_.setName("StridePhase");
    stridePhase_.setValue(0.0);
    stridePhase_.setDefaultValue(0.0);
    stridePhase_.setMinValue(0.0);
    stridePhase_.setMaxValue(1.0);
    parameter_handler::handler->addParam(stridePhase_);
  }

  bool getAvailableGaitsCallback(
      motion_generation_msgs::GetAvailableGaits::Request &req,
      motion_generation_msgs::GetAvailableGaits::Response &res) {
    try {
      const auto& contactSchedule = dynamic_cast<const loco::ContactScheduleZmp&>(locomotionController_->getContactSchedule());

      res.available_gaits_names.clear();
      res.available_gaits_ids.clear();
      const auto gaitNamesSize = contactSchedule.getMapGaitNameToId().size();
      res.available_gaits_names.reserve(gaitNamesSize);
      res.available_gaits_ids.reserve(gaitNamesSize);
      {
        for (const auto& pair : contactSchedule.getMapGaitNameToId()) {
          if (contactSchedule.isValidGait(pair.first)) {
            res.available_gaits_names.push_back(pair.first);
            res.available_gaits_ids.push_back(pair.second);
          }
        }
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  bool switchGaitCallback(
      motion_generation_msgs::SwitchGait::Request &req,
      motion_generation_msgs::SwitchGait::Response &res) {
    try {
      auto& gaitPatternRef = dynamic_cast<loco::ContactScheduleZmp&>(*locomotionController_->getContactSchedulePtr());
      const auto& map = gaitPatternRef.getMapGaitNameToId();

      loco::contact_schedule::ContactScheduleSwitchStatus status;
      gaitPatternRef.setDesiredGaitById(map.at(req.name), status);
      res.status = motion_generation_msgs::SwitchGait::Response::STATUS_SWITCHED;
      return true;
    } catch (...) {
      return false;
    }
  }

  void poseCommandsCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    kindr_ros::convertFromRosGeometryMsg(msg->pose, desiredRobotPose_);
  }

  void velocityCommandsCallback(const geometry_msgs::TwistStampedConstPtr& msg) {
    kindr_ros::convertFromRosGeometryMsg(msg->twist, desiredRobotTwist_);
    optimize();
  }

  anymal_msgs::AnymalState createRosMsg(const anymal_model::AnymalState& stateRomo) {
    anymal_model_ros::toRos(stateRomo, anymalStateRos_);

    anymalStateRos_.state = 1;
    const auto timeNow = ros::Time::now();
    anymalStateRos_.header.stamp = timeNow;
    anymalStateRos_.joints.header.stamp = timeNow;
    anymalStateRos_.pose.header.stamp = timeNow;
    anymalStateRos_.twist.header.stamp = timeNow;
    for(auto& tf : anymalStateRos_.frame_transforms) {
      tf.header.stamp = timeNow;
    }
    return anymalStateRos_;
  }

  virtual ~PoseOptimizationHelper() {
    gaitPatternVisualizer_.shutdown();
    footholdPlanVisualizer_.shutdown();
    getAvailableGatisService_.shutdown();
    switchGaitService_.shutdown();
    supportPolygonSequence_.shutdown();
    parameter_handler::handler->cleanup();
    delete anymalModel_;
    delete anymalModelDesired_;
  }

  bool optimize() {
    auto& contactScheduleRef = dynamic_cast<loco::ContactScheduleZmpTest&>(*locomotionController_->getContactSchedulePtr());
    contactScheduleRef.setCyclePhase(stridePhase_.getValue());
    // Advance measurements
    locomotionController_->advanceMeasurements(dt);
    missionController_->advance(dt);
    // Advance setpoints
    locomotionController_->advanceSetPoints(dt);
    //
    comControl_->getMotionPlan(motionPlan_);

    motionPlanVisualizer_.update(motionPlan_);
    motionPlanVisualizer_.publish();
    supportPolygonSequence_.update(motionPlan_);
    supportPolygonSequence_.publish();

    const auto* gait = dynamic_cast<const loco::contact_schedule::ContactScheduleAnymalBase*>(locomotionController_->getContactSchedulePtr());
    gaitPatternVisualizer_.updateContactSchedule(gait, 1.5);
    gaitPatternVisualizer_.updateStridePhase(contactScheduleRef.getCyclePhase());
    gaitPatternVisualizer_.publish();
    const auto* fhGen = dynamic_cast<const loco::FootPlacementStrategyOptimized<loco::FootholdGeneratorOptimizedInvPend, loco::foothold_generator::FootholdPlanInvPend>*>(
      &locomotionController_->getFootPlacementStrategy());
    fhGen->getFootholdPlan(footholdPlan_);
    footholdPlanVisualizer_.update(footholdPlan_);
    footholdPlanVisualizer_.publish();
    return true;
  }

  void setDefaultJointPosition(anymal_model::AnymalState& state)
  {
    state.getJointPositions().setSegment<AD::getNumDofLimb()>(
        AD::getLimbStartIndexInJ(AD::LimbEnum::LF_LEG), anymal_model::JointPositionsLimb(-0.1, 0.7, -1.0));

    state.getJointPositions().setSegment<AD::getNumDofLimb()>(
        AD::getLimbStartIndexInJ(AD::LimbEnum::RF_LEG), anymal_model::JointPositionsLimb(0.1, 0.7, -1.0));

    state.getJointPositions().setSegment<AD::getNumDofLimb()>(
        AD::getLimbStartIndexInJ(AD::LimbEnum::LH_LEG), anymal_model::JointPositionsLimb(-0.1, -0.7, 1.0));

    state.getJointPositions().setSegment<AD::getNumDofLimb()>(
        AD::getLimbStartIndexInJ(AD::LimbEnum::RH_LEG), anymal_model::JointPositionsLimb(0.1, -0.7, 1.0));
  }

 protected:
  anymal_model::AnymalModel* anymalModel_{nullptr};
  anymal_model::AnymalModel* anymalModelDesired_{nullptr};
  std::unique_ptr<loco_anymal::LegsAnymal> legs_{nullptr};

  std::unique_ptr<loco::TerrainAdapter> terrainAdapter_{nullptr};
  std::unique_ptr<loco::ComSupportControlZmp> comControl_{nullptr};
  std::unique_ptr<loco::TorsoControlZmp> torsoController_{nullptr};
  std::unique_ptr<loco::MissionControlZmp> missionController_{nullptr};

  std::unique_ptr<loco::TerrainPerceptionFreePlane> terrainPerception_{nullptr};
  std::unique_ptr<loco::ContactForceDistributionInterface> contactForceDistribution_{nullptr};
  std::unique_ptr<loco::MotionControllerBase> motionControl_{nullptr};
  std::unique_ptr<loco::LocomotionController> locomotionController_{nullptr};
  // body
  std::unique_ptr<loco_anymal::TorsoAnymal> torso_{nullptr};

  std::unique_ptr<loco::WholeBody> wholeBody_{nullptr};
  std::unique_ptr<loco::HeadingGenerator> headingGenerator_{nullptr};

  std::unique_ptr<loco::LimbCoordinatorBase> limbCoordinator_{nullptr};
  std::unique_ptr<loco::FootPlacementStrategyBase> footPlacementStrategy_{nullptr};
  std::unique_ptr<loco::ContactDetectorBase> contactDetector_{nullptr};

  zmp::VirtualPlaneFrame virtualPlaneFrame_;
  std::unique_ptr<loco::SwingTrajectoryGeneratorModule> swingTrajectoryGenerator_{nullptr};
  std::unique_ptr<loco::FootholdGeneratorOptimizedQPBase> footholdGeneratorOptimized_{nullptr};
  std::unique_ptr<loco::FootholdGeneratorInvertedPendulumBase> footholdGenerator_{nullptr};
  std::unique_ptr<loco::TerrainModelFreePlane> terrainModel_{nullptr};
  std::unique_ptr<loco::ParameterSet> modulesParameterSet_{nullptr};
  anymal_msgs::AnymalState anymalStateRos_;

  loco::MissionControlSpeedFilter speedFilter_;
  loco::Pose desiredRobotPose_;
  loco::Twist desiredRobotTwist_;
  ros::Subscriber poseCommandsSubscriber_;
  ros::Subscriber velocityCommandsSubscriber_;

  Stance nominalStanceInBaseFrame_;
  bool hidePreviousResult_;

  zmp::MotionPlan motionPlan_;
  loco::foothold_generator::FootholdPlanInvPend footholdPlan_;
  anymal_ctrl_dynamic_gaits_ros::MotionPlanVisualizer motionPlanVisualizer_;
  anymal_ctrl_dynamic_gaits_ros::SupportPolygonSequence supportPolygonSequence_;
  motion_generation::Pose initialTorsoPoseInWorldFrame_;
  std::unique_ptr<loco::ContactScheduleZmp> contactSchedule_{nullptr};

  loco_ros_anymal::GaitPatterns gaitPatternVisualizer_;
  anymal_ctrl_dynamic_gaits_ros::FootholdPlanVisualizer footholdPlanVisualizer_;

  ros::ServiceServer getAvailableGatisService_;
  ros::ServiceServer switchGaitService_;
  ros::NodeHandle nodeHandle_;
  ros::Publisher anymalStatePublisher_;

  parameter_handler::Parameter<double> stridePhase_;
};


class PoseOptimizationInteractiveHelper : PoseOptimizationHelper
{
 public:
  PoseOptimizationInteractiveHelper()
      : PoseOptimizationHelper(),
        markerServer_("marker_server")
  {
    hidePreviousResult_ = false;

    for (const auto limbKey : AD::getLimbKeys()) {
      const auto limbEnum = limbKey.getEnum();
      auto position = nominalStanceInBaseFrame_[limbEnum];
      position.z() = 0.0;
      initialFootholdsInWorldFrame_[limbEnum] = position;
    }

    // Initialize a contact marker for each contact pointer of the robot
    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      const auto limbEnum = AD::mapEnums<AD::LimbEnum> (contactEnum);
      FootMarker marker(contactKey.getName(), initialFootholdsInWorldFrame_[limbEnum]);
      markerServer_.insert(marker, boost::bind(&PoseOptimizationInteractiveHelper::positionFeedback, this, _1));
      // menuHandler_.apply(markerServer_, marker.name);
    }

    motion_generation::Position position(0.0, 0.0, 0.42);
    FootMarker markerBase("torso", position);
    markerServer_.insert(markerBase, boost::bind(&PoseOptimizationInteractiveHelper::torsoPositionFeedback, this, _1));

    markerServer_.applyChanges();

    // First run at initial position.
    visualization_msgs::InteractiveMarkerFeedbackConstPtr emptyFeedback(
        new visualization_msgs::InteractiveMarkerFeedback());
    positionFeedback(emptyFeedback);
  }

 protected:

  void torsoPositionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    kindr_ros::convertFromRosGeometryMsg(feedback->pose, initialTorsoPoseInWorldFrame_);
    auto currState = anymalModel_->getState();
    currState.setPositionWorldToBaseInWorldFrame(initialTorsoPoseInWorldFrame_.getPosition());
    setDefaultJointPosition(currState);
    anymalModel_->setState(currState, true, false, false);

  // Reset contact flags after moving the torso
    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      anymalModel_->getContactContainer()[contactEnum]->setState(AD::ContactStateEnum::CLOSED);
    }

    geometry_msgs::Pose pose;
    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      const auto limbEnum = AD::mapEnums<AD::LimbEnum>(contactEnum);
      const auto branchEnum = AD::mapEnums<AD::BranchEnum>(limbEnum);
      const auto limbEndBody = AD::getBranchEndBody(branchEnum);
      auto footPosition = anymalModel_->getPositionWorldToBody(limbEndBody, AD::CoordinateFrameEnum::WORLD);
      kindr_ros::convertToRosGeometryMsg(loco::Position(footPosition), pose.position);
      markerServer_.setPose(contactKey.getName(), pose);
    }
    markerServer_.applyChanges();
    anymalStatePublisher_.publish(createRosMsg(currState));
  }

  void positionFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (!feedback->marker_name.empty()) {
      const auto contactEnum = AD::mapKeyNameToKeyEnum<AD::ContactEnum>(feedback->marker_name);
      const auto limbEnum = AD::mapEnums<AD::LimbEnum>(contactEnum);
      motion_generation::Position position;
      kindr_ros::convertFromRosGeometryMsg(feedback->pose.position, position);
      initialFootholdsInWorldFrame_[limbEnum] = position;

      auto currState = anymalModel_->getState();
      const auto initialFootHoldInBaseFrame =
        currState.getOrientationBaseToWorld().inverseRotate(initialFootholdsInWorldFrame_[limbEnum] - currState.getPositionWorldToBaseInWorldFrame());

      const auto limbJointPositions = legs_->getLegPtrById(AD::mapKeyEnumToKeyId<AD::LimbEnum>(limbEnum))->
        getFootPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrameIteratively(initialFootHoldInBaseFrame);

      currState.getJointPositions().toImplementation().segment<3>(3 * AD::mapKeyEnumToKeyId<AD::LimbEnum>(limbEnum)) = limbJointPositions;
      anymalModel_->setState(currState, true, false, false);

      // Open the contact enum when moving the feet
      anymalModel_->getContactContainer()[contactEnum]->setState(AD::ContactStateEnum::OPEN);

      anymalStatePublisher_.publish(createRosMsg(currState));
    }
  }

 private:
  interactive_markers::InteractiveMarkerServer markerServer_;
  Stance initialFootholdsInWorldFrame_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zmp_anymal_interactive_test");
  PoseOptimizationInteractiveHelper optimizationHelper;
  ros::spin();
  return 0;
}

