/*
 * FrictionPyramid.cpp
 *
 *  Created on: Nov 16, 2015
 *      Author: Dario Bellicoso
 */

// loco ros
#include <loco_ros/visualization/FrictionPyramid.hpp>
#include <loco_ros/loco_ros.hpp>

namespace loco_ros {

FrictionPyramid::FrictionPyramid(double frictionCoefficient_, double pyramidHeight_)
    : frictionCoefficient_(frictionCoefficient_),
      pyramidHeight_(pyramidHeight_)
{
}

bool FrictionPyramid::shutdown()
{
  publisher_.shutdown();
  return true;
}

bool FrictionPyramid::initialize(ros::NodeHandle& nodeHandle, const std::string& topic, int numLegs)
{
  publisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>(topic, 100);

  pyramids_.markers.resize(numLegs, visualization_msgs::Marker());

  for (int k = 0; k < numLegs; k++) {
    pyramids_.markers[k].pose.orientation.w = 1.0;
    pyramids_.markers[k].type = visualization_msgs::Marker::LINE_STRIP;
    pyramids_.markers[k].scale.x = 0.005;
    pyramids_.markers[k].scale.y = 0.005;
    pyramids_.markers[k].color.a = 0.8;
    pyramids_.markers[k].color.r = 32.0;
    pyramids_.markers[k].color.g = 178.0;
    pyramids_.markers[k].color.b = 170.0;
    pyramids_.markers[k].id = k;
    pyramids_.markers[k].header.frame_id = "odom";
    pyramids_.markers[k].ns = "friction_pyramid";

    pyramids_.markers[k].points.clear();
    pyramids_.markers[k].points.resize(10, geometry_msgs::Point());
  }

  return true;
}

bool FrictionPyramid::update(const loco::Vector& headingInWorldFrame, const loco::Legs& legs,
                             const loco::TerrainModelBase& terrainModel)
{
  if (publisher_.getNumSubscribers() > 0u) {

    for (const auto& leg : legs) {
      int legId = leg->getId();
      pyramids_.markers[legId].header.stamp = ros::Time::now();

      if ( (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::Support) ||
           (leg->getLimbStrategy().getLimbStrategyEnum() == loco::LimbStrategyEnum::ContactInvariant) ) {
        const loco::Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        const loco::Position& vertexA = positionWorldToFootInWorldFrame;

        loco::Vector normalInWorldFrame;
        terrainModel.getNormal(positionWorldToFootInWorldFrame, normalInWorldFrame);

        const loco::Vector tangentialInWorldFrame = headingInWorldFrame.cross(normalInWorldFrame).normalized();
        const loco::Vector projectedHeadingInWorldFrame = tangentialInWorldFrame.cross(normalInWorldFrame).normalized();

        terrainModel.getFrictionCoefficientForFoot(vertexA, frictionCoefficient_);

        const double modulatedFriction = frictionCoefficient_*leg->getFrictionModulation();

        // compute pyramid top vertices
        const loco::Position vertexB = vertexA + loco::Position(normalInWorldFrame * pyramidHeight_)
            + loco::Position(projectedHeadingInWorldFrame * modulatedFriction * pyramidHeight_)
            + loco::Position(tangentialInWorldFrame * modulatedFriction * pyramidHeight_);
        const loco::Position vertexC = vertexA + loco::Position(normalInWorldFrame * pyramidHeight_)
            + loco::Position(projectedHeadingInWorldFrame * modulatedFriction * pyramidHeight_)
            - loco::Position(tangentialInWorldFrame * modulatedFriction) * pyramidHeight_;
        const loco::Position vertexD = vertexA + loco::Position(normalInWorldFrame * pyramidHeight_)
            - loco::Position(projectedHeadingInWorldFrame * modulatedFriction * pyramidHeight_)
            - loco::Position(tangentialInWorldFrame * modulatedFriction * pyramidHeight_);
        const loco::Position vertexE = vertexA + loco::Position(normalInWorldFrame * pyramidHeight_)
            - loco::Position(projectedHeadingInWorldFrame * modulatedFriction * pyramidHeight_)
            + loco::Position(tangentialInWorldFrame * modulatedFriction * pyramidHeight_);

        pyramids_.markers[legId].points[0].x = vertexB.x();
        pyramids_.markers[legId].points[0].y = vertexB.y();
        pyramids_.markers[legId].points[0].z = vertexB.z();

        pyramids_.markers[legId].points[1].x = vertexC.x();
        pyramids_.markers[legId].points[1].y = vertexC.y();
        pyramids_.markers[legId].points[1].z = vertexC.z();

        pyramids_.markers[legId].points[2].x = vertexD.x();
        pyramids_.markers[legId].points[2].y = vertexD.y();
        pyramids_.markers[legId].points[2].z = vertexD.z();

        pyramids_.markers[legId].points[3].x = vertexE.x();
        pyramids_.markers[legId].points[3].y = vertexE.y();
        pyramids_.markers[legId].points[3].z = vertexE.z();

        pyramids_.markers[legId].points[4].x = vertexB.x();
        pyramids_.markers[legId].points[4].y = vertexB.y();
        pyramids_.markers[legId].points[4].z = vertexB.z();

        pyramids_.markers[legId].points[5].x = vertexA.x();
        pyramids_.markers[legId].points[5].y = vertexA.y();
        pyramids_.markers[legId].points[5].z = vertexA.z();

        pyramids_.markers[legId].points[6].x = vertexC.x();
        pyramids_.markers[legId].points[6].y = vertexC.y();
        pyramids_.markers[legId].points[6].z = vertexC.z();

        pyramids_.markers[legId].points[7].x = vertexD.x();
        pyramids_.markers[legId].points[7].y = vertexD.y();
        pyramids_.markers[legId].points[7].z = vertexD.z();

        pyramids_.markers[legId].points[8].x = vertexA.x();
        pyramids_.markers[legId].points[8].y = vertexA.y();
        pyramids_.markers[legId].points[8].z = vertexA.z();

        pyramids_.markers[legId].points[9].x = vertexE.x();
        pyramids_.markers[legId].points[9].y = vertexE.y();
        pyramids_.markers[legId].points[9].z = vertexE.z();

        pyramids_.markers[legId].action = visualization_msgs::Marker::ADD;
      } else {
        pyramids_.markers[legId].action = visualization_msgs::Marker::DELETE;
      }
    }
  }

  return true;
}

unsigned int FrictionPyramid::getNumSubscribers() const {
  return publisher_.getNumSubscribers();
}

bool FrictionPyramid::publish() {
  loco_ros::publishMsg(publisher_, pyramids_);
  return true;
}

void FrictionPyramid::setFrictionCoefficient(double frictionCoefficient) {
  frictionCoefficient_ = frictionCoefficient;
}

double FrictionPyramid::getFrictionCoefficient() const {
  return frictionCoefficient_;
}

} /* namespace loco_ros */
