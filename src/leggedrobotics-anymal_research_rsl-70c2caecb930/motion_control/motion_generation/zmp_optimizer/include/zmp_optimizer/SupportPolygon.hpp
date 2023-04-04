/*
 * zmp_optimizer.hpp
 *
 *  Created on: Jan 26, 2017
 *      Author: Dario Bellicoso, Fabian Jenelten
 */

#pragma once

// motion generation.
#include <motion_generation_utils/typedefs.hpp>

// robot utils.
#include "std_utils/std_utils.hpp"

// robot utils.
#include "robot_utils/geometry/geometry.hpp"
#include "robot_utils/math/math.hpp"

// loco.
#include "loco/gait_pattern/contact_schedule_anymal.hpp"

// stl
#include <map>
#include <string>

// eigen
#include <Eigen/StdVector>

namespace zmp {

// Define the type of the contact
CONSECUTIVE_ENUM(PolygonType, Empty, Point, Line, Triangle, Rectangle, Pentagon, Hexagon, Heptagon, Octagon, Enneagon)
extern std::map<zmp::PolygonType, std::string> PolygonTypeNamesMap;

enum VertexOrder { ClockWise, CounterClockWise };

// Line coefficients for one line of a support polygon
enum LineCoeffId { lineCoeffA = 0, lineCoeffB = 1, lineCoeffC = 2 };

class SupportPolygon {
 public:
  SupportPolygon();

  explicit SupportPolygon(const motion_generation::anymalLegsPosition& positionWorldToFeetInWorldFrame, double duration,
                          const motion_generation::anymalLegsBool& isStancedLegId, const motion_generation::Pose& posePlaneToWorld);

  virtual ~SupportPolygon() = default;

  //! Returns the support polygon.
  const robot_utils::geometry::Polygon& getPolygon() const;

  //! Returns a pointer to the support polygon.
  robot_utils::geometry::Polygon* getPolygonPtr();

  //! Returns the polgon duration.
  double getDuration() const;

  //! Set polygon duration.
  void setDuration(double duration);

  //! Increase the polygonduration by that amount.
  void addDuration(double additionalTime);

  //! Returns the polygon type.
  PolygonType getPolygonType() const;

  //! True if iLeg is grounded, false otherwise.
  bool isStanceLeg(const loco::contact_schedule::LegEnumAnymal& legEnum) const;

  //! Foot-position of support used to form support polygon.
  const Eigen::Vector3d& getPositionPlaneToFootInPlaneFrame(const loco::contact_schedule::LegEnumAnymal& legEnum) const;

  //! Returns number of vertices that correspond to a stance leg.
  unsigned int getNumOfStanceLegs() const;

  //! Convert point and line constraints to rectangular constraints.
  bool convertToRectangle(double lineToRectangleOffset, double pointToRectangleOffset, unsigned int numOfVertecesPerCircleConstraint = 8);

 private:
  //! Create a polygon based on the feet configuration and rotate it to plane frame.
  // Note: Call this function only once!!
  bool createRotatedPolygon(const motion_generation::Pose& posePlaneToWorld,
                            const motion_generation::anymalLegsPosition& positionWorldToFeetInWorldFrame);

  //! Convert a line constraints to rectangular constraint.
  bool turnLineToRectangle(double lineToRectangleOffset);

  //! Convert a point constraint to rectangular constraint.
  bool turnPointToRectangle(double pointToRectangleOffset, unsigned int numOfVertecesPerCircleConstraint);

  //! The support polygon which represents the support geometry w.r.t. the virtual plane.
  robot_utils::geometry::Polygon polygon_;

  //! How long the polygon is a supporting one.
  double duration_;

  //! Characterizes the polygon type and thereby the number of vertices of the contact (e.g., empty, point, line, triangle or square).
  PolygonType polygonType_;

  //! Element is true if leg is grounded.
  motion_generation::anymalLegsBool isStancedLegId_;

  //! Feet position that is used to form current support polygon.
  motion_generation::anymalLegsVector positionPlaneToFeetInPlaneFrame_;
};

}  // namespace zmp
