/*
 * FrictionCone.cpp
 *
 *  Created on: Nov 17, 2015
 *      Author: Dario Bellicoso
 */

#include <loco_ros/visualization/FrictionCone.hpp>

namespace loco_ros {

FrictionCone::FrictionCone(double frictionCoefficient_, double coneHeight_)
    : frictionCoefficient_(frictionCoefficient_),
      coneHeight_(coneHeight_)
{
}

FrictionCone::~FrictionCone()
{

}

bool FrictionCone::shutdown()
{
  publisher_.shutdown();
  return true;
}

bool FrictionCone::initialize(ros::NodeHandle& nodeHandle, const std::string& topic, int numLegs)
{
  publisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(topic, 100);

  cones_.markers.resize(numLegs, visualization_msgs::Marker());

  for (int k = 0; k < numLegs; k++) {
    cones_.markers[k].pose.orientation.w = 1.0;
    cones_.markers[k].type = visualization_msgs::Marker::ARROW;
    cones_.markers[k].scale.x = 0.0;
    cones_.markers[k].scale.z = coneHeight_;
    cones_.markers[k].color.a = 0.6;
    cones_.markers[k].color.r = 32.0 / 255.0;
    cones_.markers[k].color.g = 178.0 / 255.0;
    cones_.markers[k].color.b = 170.0 / 255.0;
    cones_.markers[k].id = k;
    cones_.markers[k].header.frame_id = "odom";
    cones_.markers[k].ns = "friction_cone";

    cones_.markers[k].points.clear();
    cones_.markers[k].points.resize(2, geometry_msgs::Point());
  }

  return true;
}

unsigned int FrictionCone::getNumSubscribers() const {
  return publisher_.getNumSubscribers();
}

bool FrictionCone::update(const loco::Vector& headingInWorldFrame, const loco::Legs& legs,
                          const loco::TerrainModelBase& terrainModel)
{
  if (publisher_.getNumSubscribers() > 0u) {
    for (const auto& leg : legs) {
      int legId = leg->getId();
      cones_.markers[legId].header.stamp = ros::Time::now();

      if ( (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Support) ||
           (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactInvariant) ) {

        const loco::Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        loco::Vector normalInWorldFrame;
        terrainModel.getNormal(positionWorldToFootInWorldFrame, normalInWorldFrame);

        const loco::Position& vertexA = positionWorldToFootInWorldFrame;
        const loco::Position& vertexB = positionWorldToFootInWorldFrame
                                      + loco::Position(coneHeight_ * normalInWorldFrame);

        cones_.markers[legId].points[0].x = vertexB.x();
        cones_.markers[legId].points[0].y = vertexB.y();
        cones_.markers[legId].points[0].z = vertexB.z();

        cones_.markers[legId].points[1].x = vertexA.x();
        cones_.markers[legId].points[1].y = vertexA.y();
        cones_.markers[legId].points[1].z = vertexA.z();

        terrainModel.getFrictionCoefficientForFoot(vertexA, frictionCoefficient_);
        const double modulatedFriction = frictionCoefficient_*leg->getFrictionModulation();
        cones_.markers[legId].scale.y = 2.0 * coneHeight_ * modulatedFriction;

        cones_.markers[legId].action = visualization_msgs::Marker::ADD;
      } else {
        cones_.markers[legId].action = visualization_msgs::Marker::DELETE;
      }
    }
  }

  return true;
}

bool FrictionCone::publish()
{
  if (publisher_.getNumSubscribers() > 0u) {
    visualization_msgs::MarkerArrayConstPtr msg(new visualization_msgs::MarkerArray(cones_));
    publisher_.publish(msg);
  }

  return true;
}

void FrictionCone::setFrictionCoefficient(double frictionCoefficient) {
  frictionCoefficient_ = frictionCoefficient;
}

double FrictionCone::getFrictionCoefficient() const {
  return frictionCoefficient_;
}

} /* namespace loco_ros */
