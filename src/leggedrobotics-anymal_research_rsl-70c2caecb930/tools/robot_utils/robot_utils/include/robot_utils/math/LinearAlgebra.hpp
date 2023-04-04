#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>

namespace robot_utils {

Eigen::MatrixXd pseudoInverseAdaptiveDls(const Eigen::MatrixXd& A, double maxDampingFactor = 0.02, double singularRegionDimension = 0.06);

/**
 * Decomposes matrix A and thresholds the singular values to minimumSingularValueThreshold (ie changes singular or near-singular matrices to
 * be nonsingular)
 * @param                             A: the matrix to be decomposed and adapted
 * @param minimumSingularValueThreshold: the lowest allowed value for the singular values (ie singular values lower than this are set to
 * this)
 * @return the adapted matrix
 */
Eigen::MatrixXd adaptSingularValues(const Eigen::Ref<const Eigen::MatrixXd>& A, double minimumSingularValueThreshold);

/** \brief Computes the L transformation from handbook of robotics page 123
 *
 * \param vec3   a 3d vector to apply the L transformation to.
 * @return the L(vec3) matrix.
 */
Eigen::MatrixXd L(const Eigen::VectorXd& vec3);

/** \brief Computes the skew transformation.
 *
 * \param vec3   a 3d vector to apply the skew transformation to.
 * @return the skew(vec3) matrix.
 */
Eigen::MatrixXd skew(const Eigen::VectorXd& vec3);

}  // namespace robot_utils
