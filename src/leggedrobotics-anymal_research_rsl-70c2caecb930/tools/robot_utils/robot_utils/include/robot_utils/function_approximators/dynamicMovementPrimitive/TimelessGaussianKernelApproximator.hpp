/*********************************************************************

 \author   Peter Fankhauser

 **********************************************************************/

#pragma once

// system includes
#include <Eigen/Eigen>
#include <boost/foreach.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

// Local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/GaussianKernel.hpp"
#include "robot_utils/function_approximators/dynamicMovementPrimitive/constants.hpp"

namespace dmp {

class TimelessGaussianKernelApproximator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! constructor
   */
  TimelessGaussianKernelApproximator() = default;

  /*! destructor
   */
  ~TimelessGaussianKernelApproximator() = default;

  bool initialize(int numSystems, const Eigen::VectorXi& numRfs, const double activation, const double duration,
                  const double samplingFrequency);

  bool isInitialized() const;

  bool isSetup() const;

  bool learnFromThetas(const std::vector<Eigen::VectorXd>& thetas);

  std::string getInfoString();

  /*!
   * Gets the parameters of each system
   * @param thetas
   * @return
   */
  bool getThetas(std::vector<Eigen::VectorXd>& thetas);

  /*!
   * Sets the parameters of each transformation system
   * @param thetas
   * @return
   */
  bool setThetas(const std::vector<Eigen::VectorXd>& thetas);

  /*!
   * Gets the widths and centers of each system
   * @param widths, centers
   * @return
   */
  bool getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers);

  /*!
   * @param widths
   * @param centers
   * @return
   */
  bool getWidthsAndCenters(const int systemIndex, Eigen::VectorXd& widths, Eigen::VectorXd& centers);

  /*!
   * Gets the number of receptive field used by the system with id systemIndex
   * @param transId
   * @param numRfs
   * @return
   */
  bool getNumRFS(const int systemIndex, int& numRfs);

  /*!
   * Gets the number of receptive fields for each transformation system
   * @param numRfs
   * @return
   */
  bool getNumRFS(std::vector<int>& numRfs);

  /*!
   *
   * @param numTimeSteps
   * @param basisFunctions
   * @return
   */
  bool getBasisFunctions(const int numTimeSteps, std::vector<Eigen::MatrixXd>& basisFunctions);

  /*!
   *
   * @param xInput
   * @param currentBasisFunctionsValues
   * @return
   */
  bool getBasisFunctionsValues(const double xInput, std::vector<Eigen::VectorXd>& currentBasisFunctionsValues);

  /*!
   * Update the system by setting the progress
   * @param progress
   * @return true if successful
   */
  bool update(const double progress);

  /*!
   * Returns the current progress based on the time and the duration
   * Start value is 0 and final value is 1
   * @return progress
   */
  double getProgress() const;

  bool getCurrentPosition(Eigen::VectorXd& currentPosition);

  int getNumSystems() const;

  bool reset();

  /*!
   * @return
   */
  std::string getClassName();

 private:
  bool integrate(const double dtTotal, const int numIteration);

  void initialize();

  bool initialized_ = false;
  bool isLearned_ = false;
  double progress_ = 0.0;
  int numSystems_ = 0;

  std::vector<Eigen::VectorXd> coordinates_;
  boost::ptr_vector<GaussianKernel> gaussianKernelModels_;
};

// inline functions follow
inline bool TimelessGaussianKernelApproximator::isInitialized() const {
  return initialized_;
}

inline int TimelessGaussianKernelApproximator::getNumSystems() const {
  return numSystems_;
}

inline std::string TimelessGaussianKernelApproximator::getClassName() {
  return "TimelessGaussianKernelApproximator";
}

}  // namespace dmp
