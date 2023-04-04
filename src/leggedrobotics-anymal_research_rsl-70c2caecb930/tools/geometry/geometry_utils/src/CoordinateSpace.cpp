/**
 * @authors     Remo Diethelm, Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Coordinate space calculations.
 */

#include "geometry_utils/CoordinateSpace.hpp"
#include "geometry_utils/TransformListener.hpp"

namespace geometry_utils {

template <>
Vector3D CoordinateSpace::getWeightedDifferenceBetweenPoints<TranslationDimensionType::Txy>(const Vector3D& point1, const Vector3D& point2,
                                                                                            const Vector3D& weights) {
  Vector3D difference = Vector3D::Zero();
  difference.head<2>() = point2.head<2>() - point1.head<2>();
  return weights.asDiagonal() * difference;
}

template <>
Vector3D CoordinateSpace::getWeightedDifferenceBetweenPoints<TranslationDimensionType::Txyz>(const Vector3D& point1, const Vector3D& point2,
                                                                                             const Vector3D& weights) {
  return weights.asDiagonal() * (point2 - point1);
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoints<TranslationDimensionType::Txy>(const Vector3D& point1, const Vector3D& point2,
                                                                                        const Vector3D& weights) {
  return getWeightedDifferenceBetweenPoints<TranslationDimensionType::Txy>(point1, point2, weights).norm();
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoints<TranslationDimensionType::Txyz>(const Vector3D& point1, const Vector3D& point2,
                                                                                         const Vector3D& weights) {
  return getWeightedDifferenceBetweenPoints<TranslationDimensionType::Txyz>(point1, point2, weights).norm();
}

template <>
Vector4D CoordinateSpace::getWeightedDifferenceBetweenPoses<DimensionType::Txy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                                const Vector4D& weights) {
  Vector4D diff = Vector4D::Zero();
  diff.topRows<3>() = getWeightedDifferenceBetweenPoints<TranslationDimensionType::Txy>(
      pose1.position_.toImplementation(), pose2.position_.toImplementation(), weights.topRows<3>());
  return diff;
}

template <>
Vector4D CoordinateSpace::getWeightedDifferenceBetweenPoses<DimensionType::Txyz>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                                 const Vector4D& weights) {
  Vector4D diff = Vector4D::Zero();
  diff.topRows<3>() = getWeightedDifferenceBetweenPoints<TranslationDimensionType::Txyz>(
      pose1.position_.toImplementation(), pose2.position_.toImplementation(), weights.topRows<3>());
  return diff;
}

template <>
Vector4D CoordinateSpace::getWeightedDifferenceBetweenPoses<DimensionType::Ry>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                               const Vector4D& weights) {
  Vector4D diff = Vector4D::Zero();
  diff(Vector4Index::I_ROT) = weights(Vector4Index::I_ROT) * getDifferenceBetweenAngles(pose1.getYaw(), pose2.getYaw());
  return diff;
}

template <>
Vector4D CoordinateSpace::getWeightedDifferenceBetweenPoses<DimensionType::Rrpy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                                 const Vector4D& weights) {
  Vector4D diff = Vector4D::Zero();
  diff(Vector4Index::I_ROT) = weights(Vector4Index::I_ROT) * pose1.orientation_.getDisparityAngle(pose2.orientation_);
  return diff;
}

template <>
Vector4D CoordinateSpace::getWeightedDifferenceBetweenPoses<DimensionType::TxyRy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                                  const Vector4D& weights) {
  return getWeightedDifferenceBetweenPoses<DimensionType::Txy>(pose1, pose2, weights) +
         getWeightedDifferenceBetweenPoses<DimensionType::Ry>(pose1, pose2, weights);
}

template <>
Vector4D CoordinateSpace::getWeightedDifferenceBetweenPoses<DimensionType::TxyzRy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                                   const Vector4D& weights) {
  return getWeightedDifferenceBetweenPoses<DimensionType::Txyz>(pose1, pose2, weights) +
         getWeightedDifferenceBetweenPoses<DimensionType::Ry>(pose1, pose2, weights);
}

template <>
Vector4D CoordinateSpace::getWeightedDifferenceBetweenPoses<DimensionType::TxyzRrpy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                                     const Vector4D& weights) {
  return getWeightedDifferenceBetweenPoses<DimensionType::Txyz>(pose1, pose2, weights) +
         getWeightedDifferenceBetweenPoses<DimensionType::Rrpy>(pose1, pose2, weights);
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoses<DimensionType::Txy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                            const Vector4D& weights) {
  return getWeightedDistanceBetweenPoints<TranslationDimensionType::Txy>(pose1.position_.toImplementation(),
                                                                         pose2.position_.toImplementation(), weights.topRows<3>());
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoses<DimensionType::Txyz>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                             const Vector4D& weights) {
  return getWeightedDistanceBetweenPoints<TranslationDimensionType::Txyz>(pose1.position_.toImplementation(),
                                                                          pose2.position_.toImplementation(), weights.topRows<3>());
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoses<DimensionType::Ry>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                           const Vector4D& weights) {
  return weights(Vector4Index::I_ROT) * getDistanceBetweenAngles(pose1.getYaw(), pose2.getYaw());
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoses<DimensionType::Rrpy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                             const Vector4D& weights) {
  return weights(Vector4Index::I_ROT) * std::abs(pose1.orientation_.getDisparityAngle(pose2.orientation_));
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoses<DimensionType::TxyRy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                              const Vector4D& weights) {
  return (getWeightedDifferenceBetweenPoses<DimensionType::Txy>(pose1, pose2, weights) +
          getWeightedDifferenceBetweenPoses<DimensionType::Ry>(pose1, pose2, weights))
      .norm();
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoses<DimensionType::TxyzRy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                               const Vector4D& weights) {
  return (getWeightedDifferenceBetweenPoses<DimensionType::Txyz>(pose1, pose2, weights) +
          getWeightedDifferenceBetweenPoses<DimensionType::Ry>(pose1, pose2, weights))
      .norm();
}

template <>
double CoordinateSpace::getWeightedDistanceBetweenPoses<DimensionType::TxyzRrpy>(const PoseStamped& pose1, const PoseStamped& pose2,
                                                                                 const Vector4D& weights) {
  return (getWeightedDifferenceBetweenPoses<DimensionType::Txyz>(pose1, pose2, weights) +
          getWeightedDifferenceBetweenPoses<DimensionType::Rrpy>(pose1, pose2, weights))
      .norm();
}

double CoordinateSpace::wrapAngle(double angle) {
  return kindr::wrapPosNegPI(angle);
}

double CoordinateSpace::getDifferenceBetweenAngles(double angle1, double angle2) {
  return wrapAngle(angle2 - angle1);
}

double CoordinateSpace::getDistanceBetweenAngles(double angle1, double angle2) {
  return std::abs(getDifferenceBetweenAngles(angle1, angle2));
}

}  // namespace geometry_utils
