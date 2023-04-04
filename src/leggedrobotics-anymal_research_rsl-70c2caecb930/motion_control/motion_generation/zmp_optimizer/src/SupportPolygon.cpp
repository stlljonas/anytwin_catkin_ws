/*
 * zmp_optimizer.hpp
 *
 *  Created on: Jan 26, 2017
 *      Author: Dario Bellicoso
 */

// anymal_description
#include <anymal_description/LegEnum.hpp>

// zmp optimizer
#include <zmp_optimizer/SupportPolygon.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace zmp {

std::map<zmp::PolygonType, std::string> PolygonTypeNamesMap = {{PolygonType::Empty, "Empty"},         {PolygonType::Point, "Point"},
                                                               {PolygonType::Line, "Line"},           {PolygonType::Triangle, "Triangle"},
                                                               {PolygonType::Rectangle, "Rectangle"}, {PolygonType::Pentagon, "Pentagon"},
                                                               {PolygonType::Hexagon, "Hexagon"},     {PolygonType::Heptagon, "Heptagon"},
                                                               {PolygonType::Octagon, "Octagon"},     {PolygonType::Enneagon, "Enneagon"}};

SupportPolygon::SupportPolygon()
    : polygon_(), duration_(0.0), polygonType_(PolygonType::Empty), isStancedLegId_(false), positionPlaneToFeetInPlaneFrame_() {}

SupportPolygon::SupportPolygon(const motion_generation::anymalLegsPosition& positionWorldToFeetInWorldFrame, double duration,
                               const motion_generation::anymalLegsBool& isStancedLegId, const motion_generation::Pose& posePlaneToWorld)
    : polygon_(), duration_(duration), isStancedLegId_(isStancedLegId), positionPlaneToFeetInPlaneFrame_() {
  if (!createRotatedPolygon(posePlaneToWorld, positionWorldToFeetInWorldFrame)) {
    MELO_WARN_STREAM("[SupportPolygon::SupportPolygon] Failed to create support polygon!")
  }
  polygonType_ = static_cast<zmp::PolygonType>(polygon_.getNumVertices());
}

const robot_utils::geometry::Polygon& SupportPolygon::getPolygon() const {
  return polygon_;
}

robot_utils::geometry::Polygon* SupportPolygon::getPolygonPtr() {
  return &polygon_;
}

double SupportPolygon::getDuration() const {
  return duration_;
}

void SupportPolygon::setDuration(double duration) {
  duration_ = duration;
}

void SupportPolygon::addDuration(double additionalTime) {
  duration_ += additionalTime;
}

PolygonType SupportPolygon::getPolygonType() const {
  return polygonType_;
}

bool SupportPolygon::isStanceLeg(const loco::contact_schedule::LegEnumAnymal& legEnum) const {
  return isStancedLegId_[legEnum];
}

const Eigen::Vector3d& SupportPolygon::getPositionPlaneToFootInPlaneFrame(const loco::contact_schedule::LegEnumAnymal& legEnum) const {
  return positionPlaneToFeetInPlaneFrame_[legEnum];
}

unsigned int SupportPolygon::getNumOfStanceLegs() const {
  return std::count(isStancedLegId_.begin(), isStancedLegId_.end(), true);
}

bool SupportPolygon::createRotatedPolygon(const motion_generation::Pose& posePlaneToWorld,
                                          const motion_generation::anymalLegsPosition& positionWorldToFeetInWorldFrame) {
  assert(polygon_.getNumVertices() == 0u);

  const unsigned int numOfVertices = getNumOfStanceLegs();
  polygon_.resetVertices(numOfVertices);

  if (numOfVertices == 0u) {
    return true;
  }
  unsigned int vertexId = 0u;

  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    if (isStancedLegId_[legEnum]) {
      positionPlaneToFeetInPlaneFrame_[legEnum] =
          posePlaneToWorld.inverseTransform(positionWorldToFeetInWorldFrame[legEnum]).toImplementation();
      polygon_.setVertex(positionPlaneToFeetInPlaneFrame_[legEnum].head<2>(), vertexId);
      ++vertexId;
    }
  }

  return true;
}

bool SupportPolygon::convertToRectangle(double lineToRectangleOffset, double pointToRectangleOffset,
                                        unsigned int numOfVertecesPerCircleConstraint) {
  if (polygon_.getNumVertices() == 2u && polygonType_ == zmp::PolygonType::Line) {
    return turnLineToRectangle(lineToRectangleOffset);
  } else if (polygon_.getNumVertices() == 1u && polygonType_ == zmp::PolygonType::Point) {
    return turnPointToRectangle(pointToRectangleOffset, numOfVertecesPerCircleConstraint);
  }

  return true;
}

bool SupportPolygon::turnLineToRectangle(double lineToRectangleOffset) {
  // Get a copy of the vertices.
  const robot_utils::geometry::Polygon::VertexList vertices = polygon_.getVertices();

  const Eigen::Vector2d lineNormal = Eigen::Vector2d(-vertices[1].y() + vertices[0].y(), vertices[1].x() - vertices[0].x()).normalized();

  const Eigen::Vector2d vertexOffset = lineToRectangleOffset * lineNormal;

  polygon_.resetVertices(4u);
  polygon_.setVertex(vertices[0] + vertexOffset, 0u);
  polygon_.setVertex(vertices[0] - vertexOffset, 1u);
  polygon_.setVertex(vertices[1] + vertexOffset, 2u);
  polygon_.setVertex(vertices[1] - vertexOffset, 3u);

  return true;
}

bool SupportPolygon::turnPointToRectangle(double pointToRectangleOffset, unsigned int numOfVertecesPerCircleConstraint) {
  polygon_ = robot_utils::geometry::Polygon(polygon_.getVertices().front(), numOfVertecesPerCircleConstraint, pointToRectangleOffset);
  return true;
}

}  // namespace zmp
