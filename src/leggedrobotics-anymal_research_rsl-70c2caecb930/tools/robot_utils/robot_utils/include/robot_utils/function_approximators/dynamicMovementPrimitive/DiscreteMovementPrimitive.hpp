/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \author  Peter Pastor, Peter Fankhauser

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
#include "DmpParameters.hpp"
#include "TransformationSystem.hpp"
#include "constants.hpp"

namespace dmp {

// forward declaration
class TransformationSystem;

// TODO(dbellicoso): make this class a base class and derive different versions !!!

/*!
 *  \class DynamicMovementPrimitive represents the interface to learn a movement, save it to disc, load it from disc,
 *  reproduce the movement, generalize to new target, ...
 */
class DiscreteMovementPrimitive {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! constructor
   */
  DiscreteMovementPrimitive();

  /*! destructor
   */
  ~DiscreteMovementPrimitive();

  /*! Initializes the DMP. This is necessary for all future use of the DMP.
   *
   * @param numTransformationSystems (input) Number of transformation systems (dimensions) of the DMP.
   * @param dmp_id (input) ID of the DMP
   * @param dmp_parameter_namespace (input) namespace in which the DMP parameters live on the param server
   * @param lwr_parameter_namespace (input) namespace in which the LWR parameters live on the param server
   * @return true if initialization was successful, false if it failed
   */
  bool initialize(int numTransformationSystems, const Eigen::VectorXi& numRfs, const double activation, const bool exponentiallySpaced,
                  const double canSysCutoff, const double samplingFrequency, const double teachingDuration, const double executionDuration,
                  const double alphaZ, const double betaZ);

  /*! Indicates whether the DMP is initialized
   *
   * @return true if DMP is initialized, false if not
   */
  bool isInitialized() const;

  /*!
   *
   * @param parameter_namespace
   * @return
   */
  bool reInitializeParams();

  // bool learnFromMinJerk(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double duration, const double delta_t);
  /*!
   * @param theta_matrix
   * @return
   */
  bool learnFromThetas(const std::vector<Eigen::VectorXd>& thetas, const Eigen::VectorXd& initialStart, const Eigen::VectorXd& initialGoal,
                       const double samplingFrequency, const double initialDuration);

  /*!
   */
  void print();

  /*!
   */
  // TODO(dbellicoso): change this to not use verbosity stuff...
  std::string getInfoString();

  /*!
   * @return
   */
  bool getInitialDuration(double& initialDuration);

  /*!
   * @param initialStart
   * @return
   */
  bool getInitialStart(Eigen::VectorXd& initialStart);
  /*!
   * @param initialGoal
   * @return
   */
  bool getInitialGoal(Eigen::VectorXd& initialGoal);
  /*!
   * @param goal
   * @return
   */
  bool getGoal(Eigen::VectorXd& initialGoal);

  /*!
   * Gets the parameters of each transformation system
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
   * Gets the linearly interpolated theta value for xInput
   * This function is used for logging and visualization
   * @param xInput
   * @param interpolatedThetas
   * @return True on success, false on failure
   */
  bool getInterpolatedTheta(const double xInput, std::vector<double>& interpolatedThetas);

  /*!
   * Gets the widths and centers of each transformation system
   * @param thetas
   * @return
   */
  bool getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers);

  /*!
   * @param widths
   * @param centers
   * @return
   */
  bool getWidthsAndCenters(const int transSystemIndex, Eigen::VectorXd& widths, Eigen::VectorXd& centers);

  /*!
   * Gets the number of receptive field used by transformation system with id transId
   * @param transId
   * @param numRfs
   * @return
   */
  bool getNumRFS(const int transId, int& numRfs);

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

  /*! set start and goal to initial start and initial goal
   * @param samplingFrequency
   * @return
   */
  bool setup(const double samplingFrequency);
  /*!
   * @param goal
   * @param samplingFrequency
   * @return
   */
  bool setup(const Eigen::VectorXd& goal, const double movementDuration = -1.0, const double samplingFrequency = -1.0);

  /*!
   * @param start
   * @param goal
   * @param movementDuration
   * @param samplingFrequency
   * @return
   */
  bool setup(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double movementDuration = -1.0,
             const double samplingFrequency = -1.0);

  /*! REAL-TIME REQUIREMENTS
   * @param goal
   * @param start_index
   * @param end_index
   * @return
   */
  bool changeGoal(const Eigen::VectorXd& goal, const int start_index, const int end_index);
  bool changeGoal(const double new_goal, const int index);

  /*! REAL-TIME REQUIREMENTS
   *
   * @param start
   * @return
   */

  bool changeStart(const Eigen::VectorXd& start);
  /*!
   * @return
   */
  bool isSetup() const;
  bool isStartSet() const;
  void unsetStart();

  /*!
   * @param movementDuration
   * @param samplingFrequency
   * @return
   */
  bool setDuration(const double movementDuration, const int samplingFrequency);

  /*! REAL-TIME REQUIREMENTS
   *  Propagates the DMP and generates an entire rollout of size numSamples. The duration of the DMP need to be
   *  set previously using one of the setup functions. The sampling duration and the number of samples specified
   *  determine the length of the trajectory and its sampling frequecy. Note, the sampling frequency of the
   *  trajectory may change.
   *
   * @param trajectory
   * @param samplingDuration
   * @param numSamples
   * @return
   */
  // bool propagateFull(Trajectory& trajectory, const double samplingDuration, const int numSamples);
  /*! REAL-TIME REQUIREMENTS
   *
   * @param desiredCoordinates
   * @param finished
   * @param currentPosition
   * @param movementDuration
   * @return
   */
  bool propagateStep(std::vector<Eigen::VectorXd>& desiredCoordinates, bool& movementFinished);

  /*! REAL-TIME REQUIREMENTS
   *
   * @param desiredCoordinates
   * @param movementFinished
   * @param samplingDuration
   * @param numSamples
   * @return
   */
  bool propagateStep(std::vector<Eigen::VectorXd>& desiredCoordinates, bool& movementFinished, const double samplingDuration,
                     const int numSamples);

  /*! writes the data trace within dmp_debug to file. This has to happen outside the run() function since otherwise would violate the
   * real-time constraints.
   *
   */
  // bool writeDebugTrajectory();
  // bool writeDebugTrajectory(const std::string& filename);
  /*!
   * @param current_desired_position
   * @param start_index
   * @param end_index
   * @return
   */
  // bool getCurrentPosition(Eigen::VectorXd& current_desired_position, const int start_index, const int end_index);
  bool getCurrentPosition(Eigen::VectorXd& currentPosition);
  bool getCurrentVelocity(Eigen::VectorXd& currentVelocity);
  bool getCurrentAcceleration(Eigen::VectorXd& currentAcceleration);

  /*!
   * @return
   */
  double getProgress(const double& xInput) const;

  /*!
   * @return
   */
  double getProgress() const;

  /*!
   * @return
   */
  int getNumTransformationSystems() const;

  /*!
   * @param transformation_systemIndex
   * @param is_increasing
   * @return
   */
  bool isIncreasing(int transformation_systemIndex, bool& is_increasing);

  /*!
   *
   * @param canonicalSystem_value
   * @param canonicalSystemTime
   * @return
   */
  bool getCanonicalSystemState(double& canonicalSystem_value, double& canonicalSystemTime);

  /*!
   * Gets the linear progress of the canonical system from 0 to 1
   *
   * @param canonicalSystem_value
   * @param canonicalSystem_progress
   * @return
   */
  // bool getCanonicalSystemProgress(const double canonicalSystem_value, double& canonicalSystem_progress);
  /*!
   * Evaluates the canonical system using its duration (tau) and alpha_x at numTimeSteps time steps going from 0 to 1.
   * @param numTimeSteps
   * @param canSystemVector
   * @return
   */
  bool getCanonicalSystem(const int numTimeSteps, Eigen::VectorXd& canSystemVector);

  /*! Reads a DMP from the directory provided by directory_name and the dmp_id
   * @param directory_name
   * @param item_id
   * @return
   */
  // bool readFromDisc(const std::string& abs_bagfile_name);
  // bool readFromDisc(const std::string& library_directory_name, const int dmp_id, const int trial_id = 0);
  /*! Writes the DMP to the directory
   * @return
   */
  // bool writeToDisc(const int trial_id = 0);
  // bool writeToDisc(const std::string& abs_bagfile_name);
  /*!
   * @return
   */
  // Parameters::Version getVersion() const;
  /*!
   *
   * @param version
   * @return
   */
  // bool getVersion(std::string& version) const;
  // ###############################################################################################################
  // TODO(dbellicoso): (MAJOR HACK!!) these functions/variables copied over from policy_library/library::LibraryItem
  // ###############################################################################################################
  /*!
   *
   * @param directory_name
   * @param item_id
   * @param item_name
   * @param is_new_dmp
   * @return
   */
  // bool initializeBase(const std::string directory_name, const int item_id, const std::string item_name, const bool is_new_dmp = true);
  bool initializeBase(const int item_id, const std::string item_name, const bool is_new_dmp = true);

  /*!
   */
  int getID() const;
  bool setID(const int id);

  /*!
   */
  std::string getName() const;
  std::string getFileName(const int trial_id);

  /*!
   * @param description
   */
  void setDescription(const std::string& description);
  std::string getDescription() const;

  /*!
   * @return
   */
  std::string getClassName();

 private:
  /*!
   * the structure that contains the states of the canonical system
   */
  struct DMPCanonical {
    double x_ = 0.0;
    double time_ = 0.0;
  };

  /*!
   */
  bool integrateAndFit();

  /*!
   *
   * @return
   */
  bool learnTransformationTarget();

  /*!
   *
   * @return returns false if the Gaussian kernel model could not come up with a prediction for various reasons, otherwise true.
   */
  bool integrate(const double dtTotal, const int numIteration);

  /*!
   * Evaluates the canonical system at the given time step (canonicalSystemTime) and writes it to canonicalSystemX
   * @param canonicalSystemX (output)
   * @param canonicalSystemTime (input)
   * @return
   */
  void integrateCanonicalSystem(double& canonicalSystemX, const double canonicalSystemTime) const;

  /*!
   */
  void resetCanonicalState();

  /*!
   */
  void initialize();

  bool initialized_ = false;

  /*!
   */
  // ros::NodeHandle node_handle_;
  /*!
   * parameters that are read from file and (usually) kept the same for all DMPs
   */
  DmpParameters params_;

  /*!
   * struct that contains the variables of the canonical system (the phase variable x and time)
   */
  DMPCanonical canonicalSystem_;

  /*!
   * vector of numTransformationSystems_ instances of the class DMPTransformationSystem that contain relevant
   * information for each dimension of the movement system.
   */

  boost::ptr_vector<TransformationSystem> transformationSystems_;

  // std::vector<double> trajectoryTargetFunctionInput_;
};

// inline functions follow
inline bool DiscreteMovementPrimitive::isInitialized() const {
  return initialized_;
}
inline bool DiscreteMovementPrimitive::isSetup() const {
  return params_.isSetUp_;
}
inline bool DiscreteMovementPrimitive::isStartSet() const {
  return params_.isStartSet_;
}
inline void DiscreteMovementPrimitive::unsetStart() {
  params_.isStartSet_ = false;
}

inline void DiscreteMovementPrimitive::resetCanonicalState() {
  canonicalSystem_.x_ = 1.0;
  canonicalSystem_.time_ = 0.0;
  params_.sampleIndex_ = 0;
}
inline bool DiscreteMovementPrimitive::getInitialDuration(double& initialDuration) {
  if (!initialized_) {
    return false;
  }
  initialDuration = params_.initialTau_;
  return true;
}
inline int DiscreteMovementPrimitive::getNumTransformationSystems() const {
  return params_.numTransformationSystems_;
}

inline std::string DiscreteMovementPrimitive::getClassName() {
  return "DynamicMovementPrimitive";
}

// #############################################################################################################
// TODO(dbellicoso): (MAJOR HACK) these functions/variables copied over from policy_library/library::LibraryItem
// #############################################################################################################
// inline bool DiscreteMovementPrimitive::initializeBase(const std::string library_directory_name, const int item_id, const std::string
// item_name, const bool is_new_dmp)
inline bool DiscreteMovementPrimitive::initializeBase(const int /*item_id*/, const std::string /*item_name*/, const bool /*is_new_dmp*/) {
  initialized_ = true;
  return true;
}

}  // namespace dmp
