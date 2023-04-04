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

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/DiscreteMovementPrimitive.hpp"
#include "kindr/common/assert_macros.hpp"

namespace dmp {

static const char* DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME = "DiscreteMovementPrimitive";

/*DiscreteMovementPrimitive::DiscreteMovementPrimitive(ros::NodeHandle& node_handle) :
 initialized_(false), item_id_(-1), node_handle_(node_handle), params_(node_handle_), dmp_debug_(node_handle_)
 {
 }*/

DiscreteMovementPrimitive::DiscreteMovementPrimitive() = default;

DiscreteMovementPrimitive::~DiscreteMovementPrimitive() {
  transformationSystems_.clear();
}

bool DiscreteMovementPrimitive::initialize(int numTransformationSystems, const Eigen::VectorXi& numRfs, const double activation,
                                           const bool exponentiallySpaced, const double canSysCutoff, const double samplingFrequency,
                                           const double teachingDuration, const double executionDuration, const double alphaZ,
                                           const double betaZ) {
  DmpParameters dmp_params;
  if (!dmp_params.initialize(samplingFrequency, teachingDuration, executionDuration, canSysCutoff, alphaZ, betaZ)) {
    printf("Could not initialize dmp parameters\n");
    initialized_ = false;
    return initialized_;
  }

  // initialize directory name and id in the base class to generate item_name_
  if (!initializeBase(0, DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME))  // TODO(dbellicoso)
  {
    printf("Could not initialize base.\n");
    initialized_ = false;
    return initialized_;
  }

  // assign dmp parameters
  params_ = dmp_params;

  // overwrite number of transformation system in dmp_params
  if (numTransformationSystems <= 0) {
    printf("Number of transformation system %i is not valid\n", numTransformationSystems);
    initialized_ = false;
    return initialized_;
  }
  params_.numTransformationSystems_ = numTransformationSystems;

  // initialized transformation systems using the gaussian kernel model parameters
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    printf("Initializing transformation system %i.\n", i);
    transformationSystems_.push_back(new TransformationSystem(this));
    if (!transformationSystems_[i].initialize(numRfs(i), activation, exponentiallySpaced, canSysCutoff)) {
      printf("Could not initialize transformation system %i.\n", i);
      initialized_ = false;
      return initialized_;
    }
  }

  // set the version of the dmp formulation
  // params_.version_ = version;

  // set canonical system to pre-defined state
  resetCanonicalState();

  // allocate some memory
  initialize();

  initialized_ = true;
  return initialized_;
}

void DiscreteMovementPrimitive::initialize() {
  // trajectoryTargetFunctionInput_.clear();
  // debug_trajectory_point_ = Eigen::VectorXd::Zero(NUM_DEBUG_CANONICAL_SYSTEM_VALUES + (params_.numTransformationSystems_ *
  // NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES));
}

bool DiscreteMovementPrimitive::reInitializeParams() {
  // return params_.initialize();
  return true;
}

bool DiscreteMovementPrimitive::learnFromThetas(const std::vector<Eigen::VectorXd>& thetas, const Eigen::VectorXd& initialStart,
                                                const Eigen::VectorXd& initialGoal, const double samplingFrequency,
                                                const double initialDuration) {
  if (!initialized_) {
    printf("DMP is not initialized.\n");
    params_.isLearned_ = false;
    return params_.isLearned_;
  }

  if (params_.numTransformationSystems_ != initialStart.size()) {
    printf("Number of transformation system (%i) does not correspond to initial start dimension (%i).\n", params_.numTransformationSystems_,
           static_cast<int>(initialStart.size()));
    params_.isLearned_ = false;
    return params_.isLearned_;
  }

  if (params_.numTransformationSystems_ != initialGoal.size()) {
    printf("Number of transformation system (%i) does not correspond to initial goal dimension (%i).\n", params_.numTransformationSystems_,
           static_cast<int>(initialGoal.size()));
    params_.isLearned_ = false;
    return params_.isLearned_;
  }

  // set y0 to start state of trajectory and set goal to end of the trajectory
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    // set initial start and initial goal
    transformationSystems_[i].setInitialStart(initialStart(i));
    transformationSystems_[i].setInitialGoal(initialGoal(i));
  }

  params_.alphaX_ = -log(params_.canSysCutoff_);
  params_.teachingDuration_ = initialDuration;

  params_.deltaT_ = static_cast<double>(1.0) / samplingFrequency;
  params_.initialDeltaT_ = params_.deltaT_;

  params_.tau_ = params_.teachingDuration_;
  params_.initialTau_ = params_.tau_;

  if (!setThetas(thetas)) {
    printf("Could not set theta parameters.\n");
    params_.isLearned_ = false;
    return params_.isLearned_;
  }

  printf("DMP learned from Thetas.\n");
  params_.isLearned_ = true;
  return params_.isLearned_;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::getInitialStart(Eigen::VectorXd& initialStart) {
  if (initialStart.size() != params_.numTransformationSystems_) {
    return false;
  }
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    initialStart(i) = transformationSystems_[i].initialY0_;
  }
  return true;
}
// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::getInitialGoal(Eigen::VectorXd& initialGoal) {
  if (initialGoal.size() != params_.numTransformationSystems_) {
    return false;
  }
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    initialGoal(i) = transformationSystems_[i].initialGoal_;
  }
  return true;
}
// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::getGoal(Eigen::VectorXd& goal) {
  if (goal.size() != params_.numTransformationSystems_) {
    return false;
  }
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    goal(i) = transformationSystems_[i].goal_;
  }
  return true;
}

bool DiscreteMovementPrimitive::setup(const double samplingFrequency) {
  Eigen::VectorXd start = Eigen::VectorXd::Zero(params_.numTransformationSystems_);
  Eigen::VectorXd goal = Eigen::VectorXd::Zero(params_.numTransformationSystems_);
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    start(i) = transformationSystems_[i].initialY0_;
    goal(i) = transformationSystems_[i].initialGoal_;
  }

  return setup(start, goal, params_.initialTau_, samplingFrequency);
}

bool DiscreteMovementPrimitive::setup(const Eigen::VectorXd& goal, const double movementDuration, const double samplingFrequency) {
  Eigen::VectorXd start = Eigen::VectorXd(params_.numTransformationSystems_);
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    start(i) = transformationSystems_[i].initialY0_;
  }
  if (!setup(start, goal, movementDuration, samplingFrequency)) {
    params_.isSetUp_ = false;
    return params_.isSetUp_;
  }

  // start has not been specified, need to be set before the DMP can be propagated
  params_.isStartSet_ = false;

  params_.isSetUp_ = true;
  return params_.isSetUp_;
}

bool DiscreteMovementPrimitive::setup(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double movementDuration,
                                      const double samplingFrequency) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if (!params_.isLearned_) {
    printf("DMP unit is not learned.\n");
    params_.isSetUp_ = false;
    return params_.isSetUp_;
  }

  if (start.size() != params_.numTransformationSystems_) {
    printf("Cannot set start of the DMP, the size is %i, but should be %i.", static_cast<int>(start.size()),
           params_.numTransformationSystems_);
    params_.isSetUp_ = false;
    return params_.isSetUp_;
  }

  if (goal.size() != params_.numTransformationSystems_) {
    printf("Cannot set goal of the DMP, the size is %i, but should be %i.", static_cast<int>(goal.size()),
           params_.numTransformationSystems_);
    params_.isSetUp_ = false;
    return params_.isSetUp_;
  }

  // reset canonical system
  resetCanonicalState();

  if (movementDuration > 0) {
    params_.tau_ = movementDuration;

    if (samplingFrequency <= 0) {
      printf("Sampling frequency %f [Hz] of the trajectory is not valid.", samplingFrequency);
      params_.isSetUp_ = false;
      return params_.isSetUp_;
    }
    params_.deltaT_ = static_cast<double>(1.0) / static_cast<double>(samplingFrequency);
  } else {
    params_.tau_ = params_.initialTau_;
    params_.deltaT_ = params_.initialDeltaT_;
  }

  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    // set internal variables to zero
    transformationSystems_[i].reset();

    // set start and goal
    transformationSystems_[i].setStart(start(i));
    transformationSystems_[i].setGoal(goal(i));

    // set current state to start state (position and velocity)
    transformationSystems_[i].setState(start(i), 0.0);
  }

  // reset the sample counter
  params_.sampleIndex_ = 0;

  // start is set
  params_.isStartSet_ = true;

  params_.isSetUp_ = true;
  return params_.isSetUp_;
}

bool DiscreteMovementPrimitive::getCurrentPosition(Eigen::VectorXd& currentPosition) {
  if (!params_.isSetUp_) {
    return false;
  }
  if (currentPosition.size() != params_.numTransformationSystems_) {
    printf("Provided vector has wrong size (%i), required size is (%i). Cannot get current position.\n",
           static_cast<int>(currentPosition.size()), params_.numTransformationSystems_);
    return false;
  }
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    currentPosition(i) = transformationSystems_[i].y_;
  }
  return true;
}

bool DiscreteMovementPrimitive::getCurrentVelocity(Eigen::VectorXd& currentVelocity) {
  if (!params_.isSetUp_) {
    return false;
  }
  if (currentVelocity.size() != params_.numTransformationSystems_) {
    printf("Provided vector has wrong size (%i), required size is (%i). Cannot get current velocity.\n",
           static_cast<int>(currentVelocity.size()), params_.numTransformationSystems_);
    return false;
  }
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    currentVelocity(i) = transformationSystems_[i].yd_;
  }
  return true;
}

bool DiscreteMovementPrimitive::getCurrentAcceleration(Eigen::VectorXd& currentAcceleration) {
  if (!params_.isSetUp_) {
    return false;
  }
  if (currentAcceleration.size() != params_.numTransformationSystems_) {
    printf("Provided vector has wrong size (%i), required size is (%i). Cannot get current acceleration.\n",
           static_cast<int>(currentAcceleration.size()), params_.numTransformationSystems_);
    return false;
  }
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    currentAcceleration(i) = transformationSystems_[i].ydd_;
  }
  return true;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::changeGoal(const Eigen::VectorXd& goal, const int start_index, const int end_index) {
  if ((!params_.isSetUp_) || (start_index < 0) || (end_index > params_.numTransformationSystems_) || (end_index <= start_index)) {
    return false;
  }
  if (goal.size() != end_index - start_index) {
    printf("Provided vector has wrong size (%i), required size is (%i). Cannot change goal position.\n", static_cast<int>(goal.size()),
           end_index - start_index);
    return false;
  }
  for (int i = start_index; i < end_index; i++) {
    transformationSystems_[i].setGoal(goal(i - start_index));
  }
  return true;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::changeGoal(const double new_goal, const int index) {
  if ((!params_.isSetUp_) || (index < 0) || (index > params_.numTransformationSystems_)) {
    return false;
  }
  transformationSystems_[index].setGoal(new_goal);
  return true;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::changeStart(const Eigen::VectorXd& start) {
  if (!params_.isSetUp_) {
    printf("DMP is not setup\n");
    return false;
  }
  if (start.size() != params_.numTransformationSystems_) {
    printf("Start vector has wrong size (%i), it should be %i.\n", static_cast<int>(start.size()), params_.numTransformationSystems_);
    return false;
  }
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    transformationSystems_[i].setStart(start(i));
    // set current state to start state (position and velocity)
    transformationSystems_[i].setState(start(i), 0.0);
  }
  params_.isStartSet_ = true;
  return true;
}

bool DiscreteMovementPrimitive::getThetas(std::vector<Eigen::VectorXd>& thetas) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  thetas.clear();
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    int numRfs;
    if (!transformationSystems_[i].gaussianKernelModel_->getNumRFS(numRfs)) {
      printf("Could not get number of receptive fields.\n");
      return false;
    }

    Eigen::VectorXd thetaVector = Eigen::VectorXd::Zero(numRfs);
    if (!transformationSystems_[i].gaussianKernelModel_->getThetas(thetaVector)) {
      printf("Could not retrieve thetas from transformation system %i.\n", i);
      return false;
    }
    thetas.push_back(thetaVector);
  }
  return true;
}

bool DiscreteMovementPrimitive::setThetas(const std::vector<Eigen::VectorXd>& thetas) {
  if (static_cast<int>(thetas.size()) != params_.numTransformationSystems_) {
    printf("Number of thetas (%i) is not equal to number of transformation systems (%i).\n", static_cast<int>(thetas.size()),
           params_.numTransformationSystems_);
    return false;
  }

  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    if (!transformationSystems_[i].gaussianKernelModel_->setThetas(thetas[i])) {
      printf("Could not set thetas of transformation system %i.\n", i);
      return false;
    }
  }
  return true;
}

bool DiscreteMovementPrimitive::getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  widths.clear();
  centers.clear();

  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    int numRfs;
    if (!transformationSystems_[i].gaussianKernelModel_->getNumRFS(numRfs)) {
      printf("Could not get number of receptive fields.\n");
      return false;
    }

    Eigen::VectorXd centerVector = Eigen::VectorXd::Zero(numRfs);
    Eigen::VectorXd widthsVector = Eigen::VectorXd::Zero(numRfs);
    if (!transformationSystems_[i].gaussianKernelModel_->getWidthsAndCenters(widthsVector, centerVector)) {
      printf("Could not retrieve thetas from transformation system %i.\n", i);
      return false;
    }
    widths.push_back(widthsVector);
    centers.push_back(centerVector);
  }
  return true;
}

bool DiscreteMovementPrimitive::getWidthsAndCenters(const int transSystemIndex, Eigen::VectorXd& widths, Eigen::VectorXd& centers) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  int numRfs;
  KINDR_ASSERT_TRUE(std::runtime_error, !getNumRFS(transSystemIndex, numRfs), "Could not get number of RFS.\n");

  KINDR_ASSERT_TRUE(std::runtime_error, widths.size() == numRfs, "");
  KINDR_ASSERT_TRUE(std::runtime_error, centers.size() == numRfs, "");

  if (!transformationSystems_[transSystemIndex].gaussianKernelModel_->getWidthsAndCenters(widths, centers)) {
    printf("Could not get widths and centers of transformation system %i.\n", transSystemIndex);
    return false;
  }
  return true;
}

bool DiscreteMovementPrimitive::getBasisFunctions(const int numTimeSteps, std::vector<Eigen::MatrixXd>& basisFunctions) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  basisFunctions.clear();
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    int numRfs;
    if (!getNumRFS(i, numRfs)) {
      return false;
    }
    Eigen::MatrixXd basisFunctionMatrix = Eigen::MatrixXd::Zero(numTimeSteps, numRfs);
    Eigen::VectorXd xInputVector = Eigen::VectorXd::Zero(numTimeSteps);
    double dx = static_cast<double>(1.0) / static_cast<double>(numTimeSteps - 1);
    xInputVector(0) = 0.0;
    for (int j = 1; j < numTimeSteps; j++) {
      xInputVector(j) = xInputVector(j - 1) + dx;
    }
    if (!transformationSystems_[i].gaussianKernelModel_->generateBasisFunctionMatrix(xInputVector, basisFunctionMatrix)) {
      printf("Gaussian kernel basis function generation failed!\n");
      return false;
    }
    basisFunctions.push_back(basisFunctionMatrix);
  }
  return true;
}

bool DiscreteMovementPrimitive::getBasisFunctionsValues(const double xInput, std::vector<Eigen::VectorXd>& basisFunctionsValues) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if (basisFunctionsValues.size() != static_cast<size_t>(params_.numTransformationSystems_)) {
    printf("basisFunctionsValues vector size (%i) does not match the number of transformation systems %i.\n",
           static_cast<int>(basisFunctionsValues.size()), params_.numTransformationSystems_);
    return false;
  }

  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    int numRfs;
    if (!getNumRFS(i, numRfs)) {
      return false;
    }

    basisFunctionsValues.at(i).setZero(numRfs);

    if (!transformationSystems_[i].gaussianKernelModel_->generateBasisFunctionVector(xInput, basisFunctionsValues.at(i))) {
      printf("Gaussian kernel basis function generation failed!\n");
      return false;
    }
  }
  return true;
}

bool DiscreteMovementPrimitive::getCanonicalSystem(const int numTimeSteps, Eigen::VectorXd& canSystemVector) {
  KINDR_ASSERT_TRUE(std::runtime_error, canSystemVector.size() == numTimeSteps, "");
  KINDR_ASSERT_TRUE(std::runtime_error, numTimeSteps > 0, "");

  double dt = params_.tau_ / static_cast<double>(numTimeSteps - 1);
  double time = 0;

  canSystemVector(0) = 1;
  for (int j = 1; j < numTimeSteps; j++) {
    integrateCanonicalSystem(canSystemVector(j), time);
    time += dt;
  }
  return true;
}

bool DiscreteMovementPrimitive::getNumRFS(const int transId, int& numRfs) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if ((transId < 0) || (transId >= params_.numTransformationSystems_)) {
    printf("Could not get number of receptive fields, the transformation system id (%i) is invalid.\n", transId);
    return false;
  }

  return transformationSystems_[transId].gaussianKernelModel_->getNumRFS(numRfs);
}

bool DiscreteMovementPrimitive::getNumRFS(std::vector<int>& numRfs) {
  numRfs.clear();
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    int tmp_numRfs;
    if (!getNumRFS(i, tmp_numRfs)) {
      return false;
    }
    numRfs.push_back(tmp_numRfs);
  }
  return true;
}

bool DiscreteMovementPrimitive::setDuration(const double movementDuration, const int samplingFrequency) {
  if (!params_.isSetUp_) {
    printf("DMP need to be setup first.");
    return false;
  }

  if (samplingFrequency <= 0) {
    printf("Sampling frequency %i [Hz] is not valid.", samplingFrequency);
    return false;
  }

  if (movementDuration <= 0.09) {
    printf("Movement duration (%f) is too small.", movementDuration);
    return false;
  }

  params_.tau_ = movementDuration;
  params_.deltaT_ = static_cast<double>(1.0) / static_cast<double>(samplingFrequency);
  return true;
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::propagateStep(std::vector<Eigen::VectorXd>& desiredCoordinates, bool& movementFinished) {
  return propagateStep(desiredCoordinates, movementFinished, params_.tau_, static_cast<int>(params_.tau_ / params_.deltaT_) + 1);
}

// REAL-TIME REQUIREMENTS
bool DiscreteMovementPrimitive::propagateStep(std::vector<Eigen::VectorXd>& desiredCoordinates, bool& movementFinished,
                                              const double samplingDuration, const int numSamples) {
  if ((!params_.isLearned_) || (!params_.isSetUp_) || (!params_.isStartSet_)) {
    if (!params_.isLearned_) {
      printf("DMP is not learned.\n");
    }
    // if(!isSetUp_) printf("DMP with id %i is not setup. Need to set start, goal, and duration first.", item_id_);
    if (!params_.isSetUp_) {
      printf("DMP with is not setup. Need to set start, goal, and duration first.\n");
    }
    movementFinished = true;
    return false;
  }

  if (desiredCoordinates.size() != static_cast<size_t>(params_.numTransformationSystems_)) {
    printf("Number of desired coordinates (%i) is not correct, it should be %i.\n", static_cast<int>(desiredCoordinates.size()),
           params_.numTransformationSystems_);
    movementFinished = true;
    return false;
  }

  if (numSamples <= 0) {
    printf("Number of samples (%i) is not valid.\n", numSamples);
  }

  double dtTotal = samplingDuration / static_cast<double>(numSamples - 1);
  double dtThreshold = 1.0 / params_.defaultSamplingFrequency_;
  int numIteration = ceil(dtTotal / dtThreshold);

  // printf("sampling duration: %f, dtTotal: %f, dtThreshold: %f, numIteration: %i\n", samplingDuration, dtTotal, dtThreshold,
  // numIteration);

  // integrate the system, make sure that all internal variables are set properly
  if (!integrate(dtTotal, numIteration)) {
    printf("Problem while integrating the dynamical system.\n");
    movementFinished = true;
    return false;
  }
  params_.sampleIndex_++;

  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    desiredCoordinates.at(i)(_POS_) = transformationSystems_[i].y_;
    desiredCoordinates.at(i)(_VEL_) = transformationSystems_[i].yd_;
    desiredCoordinates.at(i)(_ACC_) = transformationSystems_[i].ydd_;
  }

  // check whether movement has finished...
  if (params_.sampleIndex_ >= numSamples) {
    params_.isSetUp_ = false;
    params_.isStartSet_ = false;
    movementFinished = true;
    return true;
  } else {
    movementFinished = false;
  }
  return true;
}

std::string DiscreteMovementPrimitive::getInfoString() {
  std::string info;
  std::stringstream ss;

  info.append(std::string("id: ") + ss.str());
  ss.str("");
  ss.clear();

  info.append(std::string("\n\t"));
  info.append(std::string("initialized: "));
  if (initialized_) {
    info.append(std::string("true "));
  } else {
    info.append(std::string("false"));
  }

  info.append(std::string("  learned: "));
  if (params_.isLearned_) {
    info.append(std::string("true "));
  } else {
    info.append(std::string("false"));
  }

  info.append(std::string("  setup: "));
  if (params_.isSetUp_) {
    info.append(std::string("true "));
  } else {
    info.append(std::string("false"));
  }

  info.append(std::string("  is start set: "));
  if (params_.isStartSet_) {
    info.append(std::string("true "));
  } else {
    info.append(std::string("false"));
  }

  info.append(std::string("\n") + params_.getInfoString());
  for (int i = 0; i < params_.numTransformationSystems_; i++) {
    info.append(std::string("\n\t"));
    info.append(transformationSystems_[i].getInfoString());
  }

  return info;
}

bool DiscreteMovementPrimitive::integrateAndFit() {
  return false;
}

inline bool DiscreteMovementPrimitive::integrate(const double dtTotal, const int numIteration) {
  double dt = dtTotal / static_cast<double>(numIteration);

  // canonical system
  canonicalSystem_.time_ += dt;
  integrateCanonicalSystem(canonicalSystem_.x_, canonicalSystem_.time_);

  // double log_x = -log(canonicalSystem_.x_) /* * params_.tau_*/ / params_.alphaX_;

  for (int n = 0; n < numIteration; n++) {
    for (int i = 0; i < params_.numTransformationSystems_; i++) {
      // compute nonlinearity using Gaussian kernel model
      double prediction = 0;
      if (!transformationSystems_[i].gaussianKernelModel_->predict(canonicalSystem_.x_, prediction, true)) {
        printf("Could not predict output.\n");
        return false;
      }

      transformationSystems_[i].f_ = prediction * canonicalSystem_.x_;  // TODO(dbellicoso): Is this correct? Compare with predict function

      /*			transformationSystems_[i].zd_ = (params_.k_gain_ * (transformationSystems_[i].goal_ -
       * transformationSystems_[i].y_) - params_.d_gain_ transformationSystems_[i].z_ - params_.k_gain_ *
       * (transformationSystems_[i].goal_ - transformationSystems_[i].y0_) canonicalSystem_.x_ + params_.k_gain_ *
       * transformationSystems_[i].f_) * (static_cast<double> (1.0) / params_.tau_);*/

      transformationSystems_[i].zd_ =
          (params_.alphaZ_ *
               (params_.betaZ_ * (transformationSystems_[i].goal_ - transformationSystems_[i].y_) - transformationSystems_[i].z_) +
           transformationSystems_[i].f_) *
          (static_cast<double>(1.0) / params_.tau_);

      transformationSystems_[i].yd_ = transformationSystems_[i].z_ * (static_cast<double>(1.0) / params_.tau_);
      transformationSystems_[i].ydd_ = transformationSystems_[i].zd_ * (static_cast<double>(1.0) / params_.tau_);

      transformationSystems_[i].z_ += transformationSystems_[i].zd_ * dt;  //* params_.deltaT_;
      transformationSystems_[i].y_ += transformationSystems_[i].yd_ * dt;  //* params_.deltaT_;
    }
  }

  return true;
}

inline void DiscreteMovementPrimitive::integrateCanonicalSystem(double& canonicalSystemX, const double canonicalSystemTime) const {
  canonicalSystemX = exp(-(params_.alphaX_ / params_.tau_) * canonicalSystemTime);
}

bool DiscreteMovementPrimitive::isIncreasing(int transformation_systemIndex, bool& /*is_increasing*/) {
  if ((transformation_systemIndex >= 0) && (transformation_systemIndex < params_.numTransformationSystems_) && params_.isLearned_) {
    return (transformationSystems_[transformation_systemIndex].initialY0_ >=
            transformationSystems_[transformation_systemIndex].initialGoal_);
  }
  return false;
}

bool DiscreteMovementPrimitive::getCanonicalSystemState(double& canonicalSystem_value, double& canonicalSystemTime) {
  if (!initialized_) {
    printf("DMP unit is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  canonicalSystem_value = canonicalSystem_.x_;
  canonicalSystemTime = canonicalSystem_.time_;
  return true;
}

double DiscreteMovementPrimitive::getProgress() const {
  return getProgress(canonicalSystem_.x_);
}

double DiscreteMovementPrimitive::getProgress(const double& xInput) const {
  double progress;
  if (xInput < 1.0e-8) {
    progress = 1.0;
  } else if (params_.alphaX_ > 1.0e-8) {
    progress = -log(xInput) / params_.alphaX_;
  } else {
    progress = 0.0;
  }
  return progress;
}

}  // namespace dmp
