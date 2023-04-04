/*
 * WholeBody.hpp
 *
 *  Created on: Jan 19, 2017
 *      Author: Christian Gehring
 */

#pragma once

#include <ros/ros.h>
#include <loco/common/WholeBody.hpp>

namespace loco_ros {

class WholeBody {
 public:
  WholeBody() :
    nodeHandle_(),
    measuredMainBodyGravityTerms_(),
    comMarkers_(),
    comPub_()
  {

  }

  virtual ~WholeBody() = default;

  bool initialize(ros::NodeHandle& nodeHandle, const double sphereMarkerScale = 0.03) {
    nodeHandle_ = nodeHandle;

    // Mainbody gravity terms vectors.
    measuredMainBodyGravityTerms_.clear();
    measuredMainBodyGravityTerms_.emplace_back(std::make_pair(nodeHandle_.advertise<kindr_msgs::VectorAtPosition>("/loco_ros/measured_mainbody_gravity_linear", 1),kindr_msgs::VectorAtPosition()));
    measuredMainBodyGravityTerms_.emplace_back(std::make_pair(nodeHandle_.advertise<kindr_msgs::VectorAtPosition>("/loco_ros/measured_mainbody_gravity_angular", 1),kindr_msgs::VectorAtPosition()));

    measuredMainBodyGravityTerms_[0].second.type = kindr_msgs::VectorAtPosition::TYPE_FORCE;
    measuredMainBodyGravityTerms_[0].second.header.frame_id = "odom";
    measuredMainBodyGravityTerms_[0].second.position_frame_id = "odom";

    measuredMainBodyGravityTerms_[1].second.type = kindr_msgs::VectorAtPosition::TYPE_TORQUE;
    measuredMainBodyGravityTerms_[1].second.header.frame_id = "base";
    measuredMainBodyGravityTerms_[1].second.position_frame_id = "odom";

    comMarkers_.markers.clear();
    comMarkers_.markers.push_back(getInitializedReferenceSphereMarker("odom", "planned_com_on_ground", 0, sphereMarkerScale));
    comMarkers_.markers.push_back(getInitializedReferenceSphereMarker("odom", "planned_com", 1, sphereMarkerScale));
    comMarkers_.markers.push_back(getInitializedMeasurementSphereMarker("odom", "measured_com_on_ground", 2, sphereMarkerScale));
    comMarkers_.markers.push_back(getInitializedMeasurementSphereMarker("odom", "measured_com", 3, sphereMarkerScale));
    comMarkers_.markers.push_back(getInitializedMeasurementSphereMarker("odom", "center_of_pressure", 4, sphereMarkerScale));

    comPub_ = nodeHandle_.advertise<decltype(comMarkers_)>("/loco_ros/whole_body_com", 1);

    return true;
  }

  bool shutdown() {
    for (auto& pair : measuredMainBodyGravityTerms_) {
      pair.first.shutdown();
    }
    comPub_.shutdown();

    return true;
  }

  bool update(const loco::WholeBody& wholeBody) {
    if (!(getNumSubscribers() > 0u)) {
      return true;
    }

    ros::Time timestamp = ros::Time::now();

    if (wholeBody.isUpdatingDynamics()) {
      const loco::Position& positionWorldToBaseInWorldFrame =
          wholeBody.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();

      const Eigen::VectorXd& gravityTerms = wholeBody.getWholeBodyGravityTerms();
      if (measuredMainBodyGravityTerms_[0].first.getNumSubscribers()) {
        measuredMainBodyGravityTerms_[0].second.position.x = positionWorldToBaseInWorldFrame.x();
        measuredMainBodyGravityTerms_[0].second.position.y = positionWorldToBaseInWorldFrame.y();
        measuredMainBodyGravityTerms_[0].second.position.z = positionWorldToBaseInWorldFrame.z();
        measuredMainBodyGravityTerms_[0].second.vector.x = gravityTerms[0];
        measuredMainBodyGravityTerms_[0].second.vector.y = gravityTerms[1];
        measuredMainBodyGravityTerms_[0].second.vector.z = gravityTerms[2];
      }

      if (measuredMainBodyGravityTerms_[1].first.getNumSubscribers()) {
        measuredMainBodyGravityTerms_[1].second.position.x = positionWorldToBaseInWorldFrame.x();
        measuredMainBodyGravityTerms_[1].second.position.y = positionWorldToBaseInWorldFrame.y();
        measuredMainBodyGravityTerms_[1].second.position.z = positionWorldToBaseInWorldFrame.z();
        measuredMainBodyGravityTerms_[1].second.vector.x = gravityTerms[3];
        measuredMainBodyGravityTerms_[1].second.vector.y = gravityTerms[4];
        measuredMainBodyGravityTerms_[1].second.vector.z = gravityTerms[5];
      }
    }

    if (comPub_.getNumSubscribers() > 0u) {
      double supportFootprintHeight = 0.0;
      unsigned int numSupportLimbs = 0u;
      for (const auto& limb : wholeBody.getLimbs()) {
        if ( (limb->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Support) ||
             (limb->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactInvariant) ) {
          supportFootprintHeight += limb->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame().z();
          ++numSupportLimbs;
        }
      }
      if (numSupportLimbs > 0) {
        supportFootprintHeight /= (double)numSupportLimbs;
      }
      const loco::Position& positionWorldToDesiredWholeBodyCenterOfMassInWorldFrame =
          wholeBody.getWholeBodyStateDesired().getPositionWorldToWholeBodyCenterOfMassInWorldFrame();

      comMarkers_.markers[0].pose.position.x = positionWorldToDesiredWholeBodyCenterOfMassInWorldFrame.x();
      comMarkers_.markers[0].pose.position.y = positionWorldToDesiredWholeBodyCenterOfMassInWorldFrame.y();
      comMarkers_.markers[0].pose.position.z = supportFootprintHeight;
      comMarkers_.markers[0].header.stamp = timestamp;

      comMarkers_.markers[1].pose.position.x = positionWorldToDesiredWholeBodyCenterOfMassInWorldFrame.x();
      comMarkers_.markers[1].pose.position.y = positionWorldToDesiredWholeBodyCenterOfMassInWorldFrame.y();
      comMarkers_.markers[1].pose.position.z = positionWorldToDesiredWholeBodyCenterOfMassInWorldFrame.z();
      comMarkers_.markers[1].header.stamp = timestamp;

      const loco::Position& positionWorldToMeasuredWholeBodyCenterOfMassInWorldFrame =
          wholeBody.getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame();

      comMarkers_.markers[2].pose.position.x = positionWorldToMeasuredWholeBodyCenterOfMassInWorldFrame.x();
      comMarkers_.markers[2].pose.position.y = positionWorldToMeasuredWholeBodyCenterOfMassInWorldFrame.y();
      comMarkers_.markers[2].pose.position.z = supportFootprintHeight;
      comMarkers_.markers[2].header.stamp = timestamp;

      comMarkers_.markers[3].pose.position.x = positionWorldToMeasuredWholeBodyCenterOfMassInWorldFrame.x();
      comMarkers_.markers[3].pose.position.y = positionWorldToMeasuredWholeBodyCenterOfMassInWorldFrame.y();
      comMarkers_.markers[3].pose.position.z = positionWorldToMeasuredWholeBodyCenterOfMassInWorldFrame.z();
      comMarkers_.markers[3].header.stamp = timestamp;

      const loco::Position& positionWorldToCenterOfPressureInWorldFrame =
          wholeBody.getWholeBodyStateMeasured().getPositionWorldToCenterOfPressureInWorldFrame();

      comMarkers_.markers[4].pose.position.x = positionWorldToCenterOfPressureInWorldFrame.x();
      comMarkers_.markers[4].pose.position.y = positionWorldToCenterOfPressureInWorldFrame.y();
      comMarkers_.markers[4].pose.position.z = positionWorldToCenterOfPressureInWorldFrame.z();
      comMarkers_.markers[4].header.stamp = timestamp;
    }

    return true;
  }

  bool publish() {
    for (auto& pair : measuredMainBodyGravityTerms_) {
      publishMsg(pair);
    }
    publishMsg(comPub_, comMarkers_);
    return true;
  }

  unsigned int getNumSubscribers() const {
    auto numSubs = 0u;
    for (const auto& pair : measuredMainBodyGravityTerms_) {
      numSubs += pair.first.getNumSubscribers();
    }

    numSubs += comPub_.getNumSubscribers();

    return numSubs;
  }

 protected:
  ros::NodeHandle nodeHandle_;
  std::vector<std::pair<ros::Publisher, kindr_msgs::VectorAtPosition> > measuredMainBodyGravityTerms_;
  visualization_msgs::MarkerArray comMarkers_;
  ros::Publisher comPub_;
};


} /* namespace loco_ros */

