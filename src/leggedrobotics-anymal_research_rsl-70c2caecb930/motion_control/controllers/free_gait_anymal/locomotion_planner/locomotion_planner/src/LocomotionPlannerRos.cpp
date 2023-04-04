/**
 * @authors     Peter Fankhauser, Francisco Giraldez Gamez, Valentin Yuryev
 * @affiliation ANYbotics
 * @brief       Implementation of LocomotionPlannerRos class
 */

#include "locomotion_planner/LocomotionPlannerRos.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <grid_map_msgs/GetGridMap.h>
#include <kindr_ros/kindr_ros.hpp>
#include <locomotion_planner/common/type_defs.hpp>
#include <message_logger/message_logger.hpp>
#include <visualization_msgs/MarkerArray.h>

#include <functional>

namespace locomotion_planner {

LocomotionPlannerRos::LocomotionPlannerRos(ros::NodeHandle& nodeHandle, free_gait::AdapterBase& adapter)
    : nodeHandle_(nodeHandle),
      actionClient_(nodeHandle_),
      adapter_(adapter),
      rosConverter_(adapter_),
      frameConverter_(adapter_),
      locomotionPlanner_(adapter_, parameters_),
      elevationMapFilterChain_("grid_map::GridMap"),
      goalPoseServer_(nodeHandle_,"/locomotion_planner/goal_pose_action_server", false)
{
  // Read default configuration parameters
  readParameters();

  // Register callbacks.
  actionClient_.registerCallback(
      std::bind(&LocomotionPlannerRos::executeStepsActiveCallback, this),
      std::bind(&LocomotionPlannerRos::executeStepsFeedbackCallback, this, std::placeholders::_1),
      std::bind(&LocomotionPlannerRos::executeStepsDoneCallback, this, std::placeholders::_1, std::placeholders::_2));

  locomotionPlanner_.registerSendGoalCallback(
      std::bind(&LocomotionPlannerRos::sendGoalCallback, this, std::placeholders::_1));
  locomotionPlanner_.registerGetActionStateCallback(
      std::bind(&LocomotionPlannerRos::getActionStateCallback, this, std::placeholders::_1, std::placeholders::_2));
  locomotionPlanner_.registerGetTransformCallback(
      std::bind(&LocomotionPlannerRos::getTransform, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  locomotionPlanner_.registerVisualizePlanningDataCallback(
      std::bind(&LocomotionPlannerRos::visualizePlanningData, this, std::placeholders::_1));

  goalPoseServer_.registerGoalCallback(std::bind(&LocomotionPlannerRos::goalPoseGoalCallback, this));
  goalPoseServer_.registerPreemptCallback(std::bind(&LocomotionPlannerRos::goalPosePreemptCallback, this));

  // Publish twist limits
  minTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("/commands/twist_min", 10, true);
  maxTwistPublisher_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>("/commands/twist_max", 10, true);
  twistCommandSubscriber_ = nodeHandle_.subscribe(twistCommandTopic_, 1, &LocomotionPlannerRos::twistCommandCallback, this);
  getElevationMapService_ = nodeHandle_.serviceClient<grid_map_msgs::GetGridMap>(elevationMapServiceName_);
  elevationMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/locomotion_planner/elevation_map", 1, true);
  footstepVisualizationPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("/locomotion_planner/footsteps", 1, true);
}

void LocomotionPlannerRos::twistCommandCallback(const geometry_msgs::TwistStamped& message)
{
  if (isActive() && (inputType_ == LocomotionPlannerInputType::TWIST)) {
    Twist twist;
    kindr_ros::convertFromRosGeometryMsg(message.twist, twist);
    locomotionPlanner_.setNewTwistCommand(twist);
  }
}

bool LocomotionPlannerRos::readParameters()
{
  // TODO: Read parameters using yaml_tools
  // Goal topics.
  if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/input_processing/twist_command_topic", twistCommandTopic_)) {
    MELO_ERROR("Could not load twist command topic.");
    return false;
  }

  // Default goal frame
  std::string defaultGoalFrameId;
  if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/input_processing/default_goal_frame_id", defaultGoalFrameId)) {
    MELO_ERROR("Could not load default goal frame id.");
    return false;
  }
  if (defaultGoalFrameId.empty()) {
    MELO_ERROR("Default goal frame id cannot be empty!");
    return false;
  }
  parameters_.setDefaultGoalFrameId(defaultGoalFrameId);

  // Perception parameters
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters")) {
    // Elevation mapping parameters.
    if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping")) {

      std::string elevationLayer, footholdScoreLayer, collisionLayer;
      if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/layers/elevation", elevationLayer)) {
        MELO_ERROR("Could not load elevation map elevation layer.");
        return false;
      }
      if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/layers/foothold_score", footholdScoreLayer)) {
        MELO_ERROR("Could not load elevation map foothold score layer.");
        return false;
      }
      if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/layers/collision", collisionLayer)) {
        MELO_ERROR("Could not load elevation map collision layer.");
        return false;
      }
      parameters_.setElevationMapLayers(elevationLayer, footholdScoreLayer, collisionLayer);

      if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/service_name", elevationMapServiceName_)) {
        MELO_ERROR("Could not load elevation map service name.");
        return false;
      }

      double elevationMapUpdateRate;
      if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/update_rate", elevationMapUpdateRate)) {
        MELO_ERROR("Could not load elevation map update rate.");
        return false;
      }
      if (elevationMapUpdateRate == 0.0) {
        elevationMapUpdateDuration_.fromSec(0.0);
      } else {
        elevationMapUpdateDuration_.fromSec(1.0 / elevationMapUpdateRate);
      }

      double maxAgeOfLastElevationMapUpdate;
      if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/max_age_of_last_update", maxAgeOfLastElevationMapUpdate)) {
        MELO_ERROR("Could not load elevation map max. age of last update.");
        return false;
      }
      maxAgeOfLastElevationMapUpdate_.fromSec(maxAgeOfLastElevationMapUpdate);

      double mapAreaFootprintRadius, mapAreaMaxDistance;
      if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/map_area/footprint_radius", mapAreaFootprintRadius)) {
        MELO_ERROR("Could not load elevation map area footprint radius.");
        return false;
      }
      if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/map_area/max_distance", mapAreaMaxDistance)) {
        MELO_ERROR("Could not load elevation map area max. distance.");
        return false;
      }
      parameters_.setElevationMapRegionParameters(mapAreaFootprintRadius, mapAreaMaxDistance);

      // Configure elevation map filters.
      if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/elevation_map_filters")) {
        if (!elevationMapFilterChain_.configure(
            "locomotion_planner/configurations/" + configuration_ + "/perception_parameters/elevation_mapping/elevation_map_filters", nodeHandle_)) {
          MELO_ERROR("Could not configure the elevation map filter chain!");
          return false;
        }
      } else {
        elevationMapFilterChain_.clear();
      }
      runPerception_ = true;
    }
  } else {
    runPerception_ = false;
    elevationMapFilterChain_.clear();
    parameters_.setElevationMapRegionParameters(0.0, 0.0);
  }

  // Nominal planar stance list.
  XmlRpc::XmlRpcValue nominalPlanarStanceList;
  if (!nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/nominal_planar_stance", nominalPlanarStanceList)) {
    MELO_ERROR("Could not load nominal planar stance parameters.");
    return false;
  }

  if (nominalPlanarStanceList.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    MELO_ERROR("The nominal planar stance parameters must be a list.");
    return false;
  }

  for (size_t i = 0; i < nominalPlanarStanceList.size(); ++i) {
    if (nominalPlanarStanceList[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      MELO_ERROR("Nominal planar stance parameters must be maps!");
      return false;
    }

    XmlRpc::XmlRpcValue footPositionParameter;
    if (nominalPlanarStanceList[i].hasMember("foot_position")) {
      footPositionParameter = nominalPlanarStanceList[i]["foot_position"];
    } else {
      MELO_ERROR("Did not find a foot position for nominal planar stance parameter entry.");
      return false;
    }

    std::string name;
    if (footPositionParameter.hasMember("name")) {
      name = std::string(footPositionParameter["name"]);
    } else {
      MELO_ERROR("Did not find a limb name for nominal planar stance parameter entry.");
      return false;
    }

    Position2 position;
    if (footPositionParameter.hasMember("position")) {
      position.toImplementation().x() = (double) footPositionParameter["position"][0];
      position.toImplementation().y() = (double) footPositionParameter["position"][1];
    } else {
      MELO_ERROR("Did not find a position for nominal planar stance parameter entry.");
      return false;
    }

    parameters_.setNominalPlanarStanceForLimb(adapter_.getLimbEnumFromLimbString(name), position);
  }

  // Max pose difference for gait cycle.
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/max_pose_difference_for_gait_cycle")) {
    PlanarPose maxPoseDifferenceForGaitCycle;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/max_pose_difference_for_gait_cycle/slow/x", maxPoseDifferenceForGaitCycle.x());
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/max_pose_difference_for_gait_cycle/slow/y", maxPoseDifferenceForGaitCycle.y());
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/max_pose_difference_for_gait_cycle/slow/yaw", maxPoseDifferenceForGaitCycle.z());
    parameters_.setMaxPoseDifferenceForGaitCycle(Parameters::Speed::SLOW, maxPoseDifferenceForGaitCycle);
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/max_pose_difference_for_gait_cycle/fast/x", maxPoseDifferenceForGaitCycle.x());
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/max_pose_difference_for_gait_cycle/fast/y", maxPoseDifferenceForGaitCycle.y());
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/max_pose_difference_for_gait_cycle/fast/yaw", maxPoseDifferenceForGaitCycle.z());
    parameters_.setMaxPoseDifferenceForGaitCycle(Parameters::Speed::FAST, maxPoseDifferenceForGaitCycle);
  } else {
    MELO_ERROR("Did not find parameters for max. pose difference for gait cycle.");
  }

  // Default speed factor.
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/default_speed_factor")) {
    double defaultSpeedFactor;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/default_speed_factor", defaultSpeedFactor);
    parameters_.setDefaultSpeedFactor(defaultSpeedFactor);
  } else {
    MELO_ERROR("Did not find parameter for default speed factor.");
  }

  // Intermediate poses parameters.
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/intermediate_poses/max_unpreferred_direction_distance")) {
    double maxUnpreferredDirectionDistance;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/intermediate_poses/max_unpreferred_direction_distance", maxUnpreferredDirectionDistance);
    parameters_.setMaxUnpreferredDirectionDistance(maxUnpreferredDirectionDistance);
  } else {
    MELO_ERROR("Did not find parameter for max unpreferred direction distance.");
  }
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/intermediate_poses/turn_and_walk_distance")) {
    double turnAndWalkDistance;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/intermediate_poses/turn_and_walk_distance", turnAndWalkDistance);
    parameters_.setTurnAndWalkDistance(turnAndWalkDistance);
  } else {
    MELO_ERROR("Did not find parameter for turn and walk distance.");
  }

  // Footstep parameters.
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps")) {
    free_gait::Footstep footstep(LimbEnum::LF_LEG); // Limb does not matter.
    double profileHeight;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/slow/profile_height", profileHeight);
    footstep.setProfileHeight(profileHeight);
    // If footstep profile type is not set, default it to 'triangle'
    std::string profileType = "triangle";
    if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/slow/profile_type")) {
      nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/slow/profile_type", profileType);
    } else {
      MELO_WARN("Did not find parameter for slow footstep profile type. Defaulting to triangular profile");
    }
    footstep.setProfileType(profileType);
    double averageVelocity;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/slow/average_velocity", averageVelocity);
    footstep.setAverageVelocity(averageVelocity);
    parameters_.setFoostepParameters(Parameters::Speed::SLOW, footstep);

    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/fast/profile_height", profileHeight);
    footstep.setProfileHeight(profileHeight);
    // If footstep profile type is not set, default it to 'triangle'
    profileType = "triangle";
    if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/fast/profile_type")) {
      nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/fast/profile_type", profileType);
    } else {
      MELO_WARN("Did not find parameter for fast footstep profile type. Defaulting to triangular profile");
    }
    footstep.setProfileType(profileType);
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/fast/average_velocity", averageVelocity);
    footstep.setAverageVelocity(averageVelocity);
    parameters_.setFoostepParameters(Parameters::Speed::FAST, footstep);

    double skipStepThreshold;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/footsteps/skip_step_threshold", skipStepThreshold);
    parameters_.setSkipStepThreshold(skipStepThreshold);
  } else {
    MELO_ERROR("Did not find parameters for footsteps.");
  }

  // Base motion parameters.
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions")) {
    free_gait::BaseAuto baseAutoCommon;
    double centerOfMassTolerance;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/center_of_mass_tolerance", centerOfMassTolerance);
    baseAutoCommon.setCenterOfMassTolerance(centerOfMassTolerance);
    double legLengthTolerance;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/leg_length_tolerance", legLengthTolerance);
    baseAutoCommon.setLegLengthTolerance(legLengthTolerance);
    double minLimbLengthScale;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/min_limb_length_scale", minLimbLengthScale);
    baseAutoCommon.setMinLimbLengthScale(minLimbLengthScale);
    double maxLimbLengthAtClosingContactScale;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/max_limb_length_at_closing_contact_scale", maxLimbLengthAtClosingContactScale);
    baseAutoCommon.setMaxLimbLengthAtClosingContactScale(maxLimbLengthAtClosingContactScale);
    double maxLimbLengthAtOpeningContactScale;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/max_limb_length_at_opening_contact_scale", maxLimbLengthAtOpeningContactScale);
    baseAutoCommon.setMaxLimbLengthAtOpeningContactScale(maxLimbLengthAtOpeningContactScale);
    parameters_.setBaseMotionCommonParameters(baseAutoCommon);

    free_gait::BaseAuto baseAuto;
    double baseHeight;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/slow/base_height", baseHeight);
    baseAuto.setHeight(baseHeight);
    double averageLinearVelocity;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/slow/average_linear_velocity", averageLinearVelocity);
    baseAuto.setAverageLinearVelocity(averageLinearVelocity);
    double averageAngularVelocity;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/slow/average_angular_velocity", averageAngularVelocity);
    baseAuto.setAverageAngularVelocity(averageAngularVelocity);
    double supportMargin;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/slow/support_margin", supportMargin);
    baseAuto.setSupportMargin(supportMargin);
    parameters_.setBaseMotionParameters(Parameters::Speed::SLOW, baseAuto);

    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/fast/base_height", baseHeight);
    baseAuto.setHeight(baseHeight);
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/fast/average_linear_velocity", averageLinearVelocity);
    baseAuto.setAverageLinearVelocity(averageLinearVelocity);
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/fast/average_angular_velocity", averageAngularVelocity);
    baseAuto.setAverageAngularVelocity(averageAngularVelocity);
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/base_motions/fast/support_margin", supportMargin);
    baseAuto.setSupportMargin(supportMargin);
    parameters_.setBaseMotionParameters(Parameters::Speed::FAST, baseAuto);
  } else {
    MELO_ERROR("Did not find parameters for base motions.");
  }

  // Foothold optimizer parameters.
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/foothold_optimizer")) {
    double footCenterHeight;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/foothold_optimizer/foot_center_height", footCenterHeight);
    parameters_.setFootCenterHeight(footCenterHeight);
    double footholdSize;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/foothold_optimizer/foothold_size", footholdSize);
    parameters_.setFootholdSize(footholdSize);
    double footholdSearchAreaSize;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/foothold_optimizer/search_area_size", footholdSearchAreaSize);
    parameters_.setFootholdSearchAreaSize(footholdSearchAreaSize);
  } else {
    MELO_ERROR("Did not find parameters for foothold optimizer.");
  }

  // Swing trajectory parameters.
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/swing_trajectory")) {
    double swingTrajectoryMinClearance, swingTrajectoryMaxHeight;
    bool stopOnFootTrajectoryFailure, usePrimarySwingTrajectory;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/swing_trajectory/use_primary_trajectory", usePrimarySwingTrajectory);
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/swing_trajectory/stop_on_failure", stopOnFootTrajectoryFailure);
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/swing_trajectory/min_clearance", swingTrajectoryMinClearance);
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/locomotion_parameters/swing_trajectory/max_height", swingTrajectoryMaxHeight);
    parameters_.setSwingTrajectoryParameters(swingTrajectoryMinClearance, swingTrajectoryMaxHeight);
    parameters_.setStopOnFootTrajectoryFailure(stopOnFootTrajectoryFailure);
    parameters_.setUsePrimarySwingTrajectory(usePrimarySwingTrajectory);
  } else {
    MELO_ERROR("Did not find parameters for swing trajectory.");
  }

  // Twist scaling parameter
  if (nodeHandle_.hasParam("locomotion_planner/configurations/" + configuration_ + "/input_processing/twist_scaling")) {
    double twistScaling;
    nodeHandle_.getParam("locomotion_planner/configurations/" + configuration_ + "/input_processing/twist_scaling", twistScaling);
    parameters_.setTwistScaling(twistScaling);
  } else {
    MELO_ERROR("Did not find parameter twistScaling.");
  }

  MELO_INFO_STREAM("Loaded Locomotion Planner parameters for configuration " << configuration_);
  return true;
}

void LocomotionPlannerRos::resultCallback(const LocomotionPlanner::GoalPoseResult& result) {
  MELO_DEBUG_STREAM("Received result " << static_cast<int>(result) << "from Locomotion Planner");
  switch(result) {
    case LocomotionPlanner::GoalPoseResult::SUCCESS :
      result_.status = locomotion_planner_msgs::NavigateToGoalPoseResult::GOAL_REACHED;
      goalPoseServer_.setSucceeded(result_, "Goal pose reached");
      break;
    case LocomotionPlanner::GoalPoseResult::PREEMPTED :
      break;
    default:
      result_.status = locomotion_planner_msgs::NavigateToGoalPoseResult::EXECUTION_ERROR;
      goalPoseServer_.setAborted(result_, "Walking to goal pose failed.");
      break;
    }
}


void LocomotionPlannerRos::sendGoalCallback(const free_gait::StepQueue& goal)
{
  MELO_DEBUG("Sending Free Gait action.");
  free_gait_msgs::ExecuteStepsGoal goalMessage;
  goalMessage.preempt = free_gait_msgs::ExecuteStepsGoal::PREEMPT_STEP;
  rosConverter_.toMessage(goal, goalMessage.steps);
  actionClient_.sendGoal(goalMessage);
}

bool LocomotionPlannerRos::getActionStateCallback(
    free_gait::FreeGaitActionClient::ActionState& actionState,
    free_gait::ExecutorState& executorState)
{
  actionState = actionClient_.getState();
  executorState = executorState_;
  return true;
}

bool LocomotionPlannerRos::getTransform(const std::string& sourceFrameId, const std::string& targetFrameId,
                                        Transform& transform)
{
  return frameConverter_.getTransform(sourceFrameId, targetFrameId, Transform(), transform);
}

void LocomotionPlannerRos::updateElevationMap(const ros::TimerEvent& timerEvent)
{
  MELO_DEBUG("Locomotion planner updates elevation map.");
  grid_map_msgs::GetGridMap serviceCall;
  serviceCall.request.frame_id = adapter_.getWorldFrameId();
  grid_map::Position position;
  grid_map::Length length;
  locomotionPlanner_.getElevationMapRegion(position, length);
  serviceCall.request.position_x = position.x();
  serviceCall.request.position_y = position.y();
  serviceCall.request.length_x = length.x();
  serviceCall.request.length_y = length.y();

  const auto elevMapServiceTimeout = ros::Duration(0.01);
  if (!getElevationMapService_.waitForExistence(elevMapServiceTimeout) || !getElevationMapService_.call(serviceCall)) {
    const ros::Duration durationSinceLastUpdate = timerEvent.current_real - lastSuccessfulElevationMapUpdate_;
    if (durationSinceLastUpdate < maxAgeOfLastElevationMapUpdate_) {
      MELO_WARN_THROTTLE(5.0, "Failed to call service get elevation map, using old map.");
      return;
    } else {
      MELO_WARN_THROTTLE(5.0, "Failed to call service get elevation map, map is not used.");
      locomotionPlanner_.clearElevationMap();
      grid_map_msgs::GridMap message;
      elevationMapPublisher_.publish(message);
      return;
    }
  }

  grid_map::GridMap mapIn, mapOut;
  grid_map::GridMapRosConverter::fromMessage(serviceCall.response.map, mapIn);
  if (!elevationMapFilterChain_.update(mapIn, mapOut)) {
    MELO_ERROR("Could not update the elevation map filter chain!");
    return;
  }
  locomotionPlanner_.setElevationMap(mapOut);
  lastSuccessfulElevationMapUpdate_.fromNSec(mapOut.getTimestamp());

  // Publish elevation map for debugging.
  if (elevationMapPublisher_.getNumSubscribers() > 0u) {
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(mapOut, message);
    elevationMapPublisher_.publish(message);
  }
}

void LocomotionPlannerRos::visualizePlanningData(const PlanningData& planningData)
{
  if (footstepVisualizationPublisher_.getNumSubscribers() < 1) return;
  MELO_DEBUG("Visualizing locomotion planning data.");

  visualization_msgs::MarkerArray markerArray;

  {
    // Clear.
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);
  }

  {
    // Nominal footholds.
    visualization_msgs::Marker marker;
    marker.header.frame_id = adapter_.getWorldFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = "Nominal Footholds";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.035;
    marker.scale.y = 0.035;
    marker.scale.z = 0.035;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    for (const auto& footstep : planningData.getNominalFootholds()) {
      geometry_msgs::Point point;
      kindr_ros::convertToRosGeometryMsg(std::get<1>(footstep), point);
      marker.points.push_back(point);
    }
    markerArray.markers.push_back(marker);
  }

  {
    // Optimized footholds.
    visualization_msgs::Marker marker;
    marker.header.frame_id = adapter_.getWorldFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = "Optimized Footholds";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.035;
    marker.scale.y = 0.035;
    marker.scale.z = 0.035;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.2;
    for (const auto& footstep : planningData.getOptimizedFootholds()) {
      geometry_msgs::Point point;
      kindr_ros::convertToRosGeometryMsg(std::get<1>(footstep), point);
      marker.points.push_back(point);
    }
    markerArray.markers.push_back(marker);
  }

  {
    // Invalid footholds.
    visualization_msgs::Marker marker;
    marker.header.frame_id = adapter_.getWorldFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = "Invalid Footholds";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.015;
    marker.color.a = 1.0;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.8;
    for (const auto& footstep : planningData.getCandidateFootholds()) {
      const std::vector<PlanningData::FootholdValidityTypes>& validityType = std::get<2>(footstep);
      if (!validityType.empty()) continue;
      geometry_msgs::Point point;
      kindr_ros::convertToRosGeometryMsg(std::get<1>(footstep), point);
      marker.points.push_back(point);
    }
    markerArray.markers.push_back(marker);
  }

  {
    // Candidate footholds for terrain.
    visualization_msgs::Marker marker;
    marker.header.frame_id = adapter_.getWorldFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = "Valid Terrain Footholds";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.015;
    marker.color.a = 1.0;
    marker.color.r = 0.2;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    for (const auto& footstep : planningData.getCandidateFootholds()) {
      const std::vector<PlanningData::FootholdValidityTypes>& validityType = std::get<2>(footstep);
      if (std::find(validityType.begin(), validityType.end(), PlanningData::FootholdValidityTypes::Terrain) == validityType.end()) continue;
      geometry_msgs::Point point;
      kindr_ros::convertToRosGeometryMsg(std::get<1>(footstep), point);
      marker.points.push_back(point);
    }
    markerArray.markers.push_back(marker);
  }

  {
    // Candidate footholds for kinematics.
    visualization_msgs::Marker marker;
    marker.header.frame_id = adapter_.getWorldFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = "Valid Kinematics Foothold";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.015;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (const auto& footstep : planningData.getCandidateFootholds()) {
      const std::vector<PlanningData::FootholdValidityTypes>& validityType = std::get<2>(footstep);
      if (std::find(validityType.begin(), validityType.end(), PlanningData::FootholdValidityTypes::Kinematic) == validityType.end()) continue;
      geometry_msgs::Point point;
      kindr_ros::convertToRosGeometryMsg(std::get<1>(footstep), point);
      marker.points.push_back(point);
    }
    markerArray.markers.push_back(marker);
  }

  {
    // Candidate footholds for terrain and kinematics.
    visualization_msgs::Marker marker;
    marker.header.frame_id = adapter_.getWorldFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = "Valid Terrain & Kinematics Foothold";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.015;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.2;
    for (const auto& footstep : planningData.getCandidateFootholds()) {
      const std::vector<PlanningData::FootholdValidityTypes>& validityType = std::get<2>(footstep);
      if (std::find(validityType.begin(), validityType.end(), PlanningData::FootholdValidityTypes::Terrain) == validityType.end()) continue;
      if (std::find(validityType.begin(), validityType.end(), PlanningData::FootholdValidityTypes::Kinematic) == validityType.end()) continue;
      geometry_msgs::Point point;
      kindr_ros::convertToRosGeometryMsg(std::get<1>(footstep), point);
      marker.points.push_back(point);
    }
    markerArray.markers.push_back(marker);
  }

  footstepVisualizationPublisher_.publish(markerArray);
}

void LocomotionPlannerRos::executeStepsActiveCallback()
{
  MELO_DEBUG("Free Gait action server reported state active.");
}

void LocomotionPlannerRos::executeStepsFeedbackCallback(const free_gait_msgs::ExecuteStepsFeedbackConstPtr& feedback)
{
  executorState_.setState(feedback->step_number, feedback->phase);
}

void LocomotionPlannerRos::executeStepsDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                    const free_gait_msgs::ExecuteStepsResult& result)
{
  MELO_INFO("Free Gait action server reported state done.");
}

void LocomotionPlannerRos::goalPoseGoalCallback() {
  // Check if goal pose is available.
  if (!goalPoseServer_.isNewGoalAvailable()) {
    MELO_ERROR("New goal is not available.");
    return;
  }

  // Accept goal pose
  const auto& goalPosePtr = goalPoseServer_.acceptNewGoal();

  // Send goal pose to locomotion planner.
  const geometry_msgs::PoseStamped goalPoseStamped = goalPosePtr->goal_pose;
  MELO_DEBUG_STREAM("Locomotion planner action server received new goal pose:" << std::endl << goalPoseStamped);
  Pose goalPose;
  kindr_ros::convertFromRosGeometryMsg(goalPoseStamped.pose, goalPose);
  LocomotionPlanner::GoalPoseStatus status;
  if (goalPosePtr->is_relative_pose) {
    status = locomotionPlanner_.setNewRelativeGoalPose(
        goalPoseStamped.header.frame_id, goalPose);
  } else {
    status = locomotionPlanner_.setNewGoalPose(
        goalPoseStamped.header.frame_id, goalPose);
  }
  // Receive status of goal sent
  MELO_DEBUG_STREAM("Got goal pose status " << static_cast<int>(status));
  switch (status) {
    case LocomotionPlanner::GoalPoseStatus::ACCEPTED_NEW_GOAL:
      locomotionPlanner_.start();
      return;
    case LocomotionPlanner::GoalPoseStatus::ALREADY_THERE:
      result_.status = locomotion_planner_msgs::NavigateToGoalPoseResult::GOAL_REACHED;
      goalPoseServer_.setSucceeded(result_, "Robot was already at goal pose.");
      return;
    case LocomotionPlanner::GoalPoseStatus::IGNORED_SAME_AS_CURRENT:
    case LocomotionPlanner::GoalPoseStatus::OVERWRITE_CURRENT_GOAL:
      // We assume Locomotion planner did not stop.
      return;
    case LocomotionPlanner::GoalPoseStatus::REJECTED_FRAME_ERROR:
      MELO_ERROR_STREAM("Invalid frame for new goal pose:" << std::endl << goalPoseStamped);
      result_.status = locomotion_planner_msgs::NavigateToGoalPoseResult::INVALID_GOAL_FRAME;
      goalPoseServer_.setAborted(result_, "Transformation to goal pose frame could not be found, or is outdated. Goal pose rejected.");
      return;
    case LocomotionPlanner::GoalPoseStatus::REJECTED_INVALID_TRANSFORM:
      MELO_ERROR_STREAM("Invalid transform for new goal pose:" << std::endl << goalPoseStamped);
      result_.status = locomotion_planner_msgs::NavigateToGoalPoseResult::INVALID_TRANSFORM;
      goalPoseServer_.setAborted(result_, "Transform is mathematically inconsistent. Goal pose rejected.");
      return;
    default:
      MELO_ERROR_STREAM("Unknown status received when setting new goal pose:" << std::endl << goalPoseStamped);
      result_.status = locomotion_planner_msgs::NavigateToGoalPoseResult::EXECUTION_ERROR;
      goalPoseServer_.setAborted(result_, "Unknown status received when sending goal pose.");
      return;
  }
}

void LocomotionPlannerRos::runElevationMapUpdateThread()
{
  static const double elevationMapPublishRate = 30; // TODO (fgiraldez): parameter if needed
  static const double timeout = 0.01;
  while (updateElevationMap_) {
    elevationMapUpdateQueue_.callAvailable(ros::WallDuration(timeout));
    ros::Rate(elevationMapPublishRate).sleep();
  }
}

void LocomotionPlannerRos::shutdownPerception() {
  MELO_DEBUG("Shutting down Locomotion Planner perception.");
  updateElevationMap_ = false;
  elevationMapUpdateTimer_.stop();
  elevationMapUpdateQueue_.clear();
  elevationMapUpdateQueue_.disable();
  if (elevationMapUpdateThread_.joinable()) {
    MELO_DEBUG("Joining Elevation Map thread.");
    elevationMapUpdateThread_.join();
  }
  locomotionPlanner_.clearElevationMap();
  grid_map_msgs::GridMap message;
  elevationMapPublisher_.publish(message);
}

void LocomotionPlannerRos::setConfiguration(const std::string& configuration) {
  configuration_ = configuration;
}

std::string LocomotionPlannerRos::getConfiguration() {
  return configuration_;
}

bool LocomotionPlannerRos::initialize() {
  // Read parameters
  if (!readParameters()) return false;

  // Initialize locomotion_planner depending on input type
  locomotionPlanner_.initialize();
  if (inputType_ == LocomotionPlannerInputType::TWIST) {
    publishTwistLimits();
    // Register null callback
    locomotionPlanner_.registerResultCallback(std::function<void(const LocomotionPlanner::GoalPoseResult&)>{});
    // Auto-start
    locomotionPlanner_.start();
  } else if (inputType_ == LocomotionPlannerInputType::POSE) {
    locomotionPlanner_.registerResultCallback(std::bind(&LocomotionPlannerRos::resultCallback, this, std::placeholders::_1));
    goalPoseServer_.start();
  } else {
    MELO_WARN("Undefined input type");
    return false;
  }

  // Multi-threading for elevation map updates because they shouldn't block.
  if (runPerception_) {
    MELO_DEBUG("Locomotion Planner will use perception.");
    updateElevationMap_ = true;
    if (!elevationMapUpdateDuration_.isZero()) {
      MELO_DEBUG("Starting new thread to update elevation map");
      elevationMapUpdateQueue_.enable();
      ros::TimerOptions timerOptions(elevationMapUpdateDuration_,
                                     boost::bind(&LocomotionPlannerRos::updateElevationMap, this, _1),
                                     &elevationMapUpdateQueue_, false, true);
      elevationMapUpdateTimer_ = nodeHandle_.createTimer(timerOptions);
      elevationMapUpdateThread_ = std::thread(std::bind(&LocomotionPlannerRos::runElevationMapUpdateThread, this));
    }
  } else if (!runPerception_){
    shutdownPerception();
  }

  isActive_ = true;
  return true;
}

void LocomotionPlannerRos::setInputToPose() {
  inputType_ = LocomotionPlannerInputType::POSE;
}

void LocomotionPlannerRos::setInputToTwist() {
  inputType_ = LocomotionPlannerInputType::TWIST;
}

void LocomotionPlannerRos::shutdown() {
  goalPoseServer_.shutdown();
  stopLocomotionPlannerExecution();
  shutdownPerception();
  isActive_ = false;
}

PlanarTwist LocomotionPlannerRos::getMaxTwistCommandFast() {
  free_gait::BaseAuto baseFastMotion;
  parameters_.populateBaseAutoParameters(1.0, baseFastMotion);
  return {baseFastMotion.getAverageLinearVelocity(), baseFastMotion.getAverageLinearVelocity(), baseFastMotion.getAverageAngularVelocity()};
}

PlanarTwist LocomotionPlannerRos::getMaxTwistCommandSlow() {
  free_gait::BaseAuto baseSlowMotion;
  parameters_.populateBaseAutoParameters(parameters_.getDefaultSpeedFactor(), baseSlowMotion);
  return {baseSlowMotion.getAverageLinearVelocity(), baseSlowMotion.getAverageLinearVelocity(), baseSlowMotion.getAverageAngularVelocity()};
}

bool LocomotionPlannerRos::isActive() {
  return isActive_;
}

void LocomotionPlannerRos::goalPosePreemptCallback() {
  MELO_DEBUG("Locomotion Planner has been requested to preempt.");
  if (!goalPoseServer_.isNewGoalAvailable()) {
    if (!stopLocomotionPlannerExecution()) {
      result_.status = locomotion_planner_msgs::NavigateToGoalPoseResult::EXECUTION_ERROR;
      goalPoseServer_.setAborted(result_, "Locomotion Planner step execution could not be stopped.");
      return;
    }
  }
  // If goal is overwriten, preempt current goal without stopping the planner.
  if (goalPoseServer_.isActive()) {
    result_.status = locomotion_planner_msgs::NavigateToGoalPoseResult::PREEMPTED;
    goalPoseServer_.setPreempted(result_, "Goal pose was preempted before the step has been finished. Goal pose was not reached.");
    MELO_DEBUG("Preempted successfully");
  }
}

bool LocomotionPlannerRos::stopLocomotionPlannerExecution() {
  locomotionPlanner_.stop();
  if (actionClient_.getState() == free_gait::FreeGaitActionClient::ActionState::ACTIVE) {
    MELO_DEBUG("Waiting for step action to finish.");
    actionClient_.cancelGoal();
    if (!actionClient_.waitForResult(5.0)) {
      MELO_ERROR("Last Free Gait step could not be preempted within 5 seconds!");
      return false;
    }
  }
  return true;
}

void LocomotionPlannerRos::publishTwistLimits() {
  geometry_msgs::TwistStamped maxMsg{};
  maxMsg.header.stamp = ros::Time::now();
  maxMsg.twist.linear.x = getMaxTwistCommandFast().x();
  maxMsg.twist.linear.y = getMaxTwistCommandFast().y();
  maxMsg.twist.angular.z = getMaxTwistCommandFast().z();
  maxTwistPublisher_.publish(maxMsg);
  geometry_msgs::TwistStamped minMsg{};
  minMsg.header.stamp = ros::Time::now();
  minMsg.twist.linear.x = -getMaxTwistCommandFast().x();
  minMsg.twist.linear.y = -getMaxTwistCommandFast().y();
  minMsg.twist.angular.z = -getMaxTwistCommandFast().z();
  minTwistPublisher_.publish(minMsg);
}

} /* namespace */
