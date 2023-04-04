/*
 * Triangle.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: dbellicoso
 */

#include <algorithm>
#include <robot_utils/geometry/Triangle.hpp>

namespace robot_utils {
namespace geometry {

Triangle::Triangle(const VertexList& vertices, robot_utils::geometry::VertexOrder vertexOrder) : Polygon(vertices, vertexOrder) {
  assert(vertices_.size() == 3);
  lineCoefficients_ = Polygon::LineCoefficientList(3, {0.0, 0.0, 0.0});
}

}  // namespace geometry
}  // namespace robot_utils
