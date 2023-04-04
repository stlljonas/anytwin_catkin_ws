/*
 * PoseOptimizationSqpInteractiveTest.cpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include <free_gait_anymal_common/free_gait_anymal_common_test/PoseOptimizationHelper.hpp>

#include <kindr/common/gtest_eigen.hpp>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <gtest/gtest.h>
#include <kindr_ros/RosGeometryMsgPose.hpp>

#include <boost/bind.hpp>
#include <string>

using namespace free_gait;

class FootMarker : public visualization_msgs::InteractiveMarker
{
 public:
  FootMarker(const std::string& limbName, const Position& position)
      : scale_(0.1),
        footRadius_(0.06),
        frameId_("odom")
  {
    color_.r = 1.0;
    color_.a = 1.0;
    setup(limbName, position);
  }

  ~FootMarker() {}

 private:

  void setup(const std::string& limbName, const Position& position)
  {
    controls.clear();
    menu_entries.clear();

    header.frame_id = frameId_;
    name = limbName;
    scale = scale_;
    Pose initialPose;
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

class PoseOptimizationInteractiveHelper : PoseOptimizationHelper
{
 public:
  PoseOptimizationInteractiveHelper()
      : PoseOptimizationHelper(),
        markerServer_("marker_server")
  {
    optimizationStepCallback_ = std::bind(&PoseOptimizationInteractiveHelper::optimizationStepCallback, this,
                                          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                          std::placeholders::_4);
    optimizationSQP_->registerOptimizationStepCallback(optimizationStepCallback_);
    hidePreviousResult_ = false;
    sleepDurationForOptimizationStep_.fromSec(0.0);

    menuHandler_.insert("Add as Support Leg",
                        boost::bind(&PoseOptimizationInteractiveHelper::addSupportLegFeedback, this, _1));
    menuHandler_.insert("Remove as Support Leg",
                        boost::bind(&PoseOptimizationInteractiveHelper::removeSupportLegFeedback, this, _1));
    AdapterBase& adapter(*adapterRos_.getAdapterPtr());
    for (const auto& limb : adapter.getLimbs()) {
      Position position = nominalStanceInBaseFrame_[limb];
      position.z() = 0.0;
      footholdsToReach_[limb] = position;
      FootMarker marker(adapter.getLimbStringFromLimbEnum(limb), position);
      markerServer_.insert(marker, boost::bind(&PoseOptimizationInteractiveHelper::positionFeedback, this, _1));
      menuHandler_.apply(markerServer_, marker.name);
    }
    markerServer_.applyChanges();
    footholdsInSupport_ = nominalStanceInBaseFrame_;

    // First run at initial position.
    visualization_msgs::InteractiveMarkerFeedbackConstPtr emptyFeedback(
        new visualization_msgs::InteractiveMarkerFeedback());
    positionFeedback(emptyFeedback);
  }

 protected:

  void optimizationStepCallback(const size_t iterationStep,
                                const State& state,
                                const double functionValue,
                                const bool finalIteration)
  {
    if (!(finalIteration || iterationStep == 0)) {
      // Deactivate this line to see intermediate results.
      return;
    }
    PoseOptimizationHelper::optimizationStepCallback(iterationStep, state, functionValue, finalIteration);
  }

  void positionFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (!feedback->marker_name.empty()) {
      AdapterBase& adapter(*adapterRos_.getAdapterPtr());
      const LimbEnum limb = adapter.getLimbEnumFromLimbString(feedback->marker_name);
      Position position;
      kindr_ros::convertFromRosGeometryMsg(feedback->pose.position, position);
      footholdsToReach_[limb] = position;
      if (footholdsInSupport_.find(limb) != footholdsInSupport_.end()) footholdsInSupport_[limb] = position;
    }

    setFootholdsToReach(footholdsToReach_);
    setFootholdsInSupport(footholdsInSupport_, 0.0);
    Pose pose;
    optimize(pose);
  }

  void addSupportLegFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    AdapterBase& adapter(*adapterRos_.getAdapterPtr());
    const LimbEnum limb = adapter.getLimbEnumFromLimbString(feedback->marker_name);
    footholdsInSupport_[limb] = footholdsToReach_[limb];
    visualization_msgs::InteractiveMarkerFeedbackConstPtr emptyFeedback(
        new visualization_msgs::InteractiveMarkerFeedback());
    positionFeedback(emptyFeedback);
  }

  void removeSupportLegFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    AdapterBase& adapter(*adapterRos_.getAdapterPtr());
    const LimbEnum limb = adapter.getLimbEnumFromLimbString(feedback->marker_name);
    footholdsInSupport_.erase(limb);
    visualization_msgs::InteractiveMarkerFeedbackConstPtr emptyFeedback(
        new visualization_msgs::InteractiveMarkerFeedback());
    positionFeedback(emptyFeedback);
  }

 private:
  interactive_markers::InteractiveMarkerServer markerServer_;
  interactive_markers::MenuHandler menuHandler_;
  Stance footholdsToReach_;
  Stance footholdsInSupport_;
};

class PoseOptimizationSqpInteractiveTest : public ::testing::Test {
 protected:
  static void SetUpTestCase()
  {
    optimizationHelper_ = new PoseOptimizationInteractiveHelper();
  }

  static void TearDownTestCase()
  {
    delete optimizationHelper_;
    optimizationHelper_ = NULL;
  }

  static PoseOptimizationInteractiveHelper* optimizationHelper_;
};

PoseOptimizationInteractiveHelper* PoseOptimizationSqpInteractiveTest::optimizationHelper_ = NULL;


TEST_F(PoseOptimizationSqpInteractiveTest, DISABLED_Main)
{
  ros::spin();
}

