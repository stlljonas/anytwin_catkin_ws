/*********************************************************************

 \author  Peter Fankhauser

 **********************************************************************/

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/GaussianKernelApproximator.hpp"
#include "kindr/common/assert_macros.hpp"

namespace dmp {

GaussianKernelApproximator::GaussianKernelApproximator() = default;

GaussianKernelApproximator::~GaussianKernelApproximator() = default;

bool GaussianKernelApproximator::initialize(int numSystems, const Eigen::VectorXi& numRfs, const double activation, const double duration,
                                            const double samplingFrequency) {
  // overwrite number of system
  if (numSystems <= 0) {
    printf("Number of systems %i is not valid\n", numSystems);
    initialized_ = false;
    return initialized_;
  }
  numSystems_ = numSystems;

  // initialized systems using the gaussian kernel model parameters
  for (int i = 0; i < numSystems_; i++) {
    printf("Initializing Gaussian kernel system %i.\n", i);
    gaussianKernelModels_.push_back(new GaussianKernel());
    if (!gaussianKernelModels_[i].initialize(numRfs(i), activation)) {
      printf("Could not initialize Gaussian kernel system %i.\n", i);
      initialized_ = false;
      return initialized_;
    }
  }

  // coordinates
  coordinates_.resize(numSystems_, Eigen::VectorXd(dmp::POS_VEL_ACC));

  // set duration
  if (duration <= 0) {
    printf("Duration (%f s) of the trajectory is not valid.", duration);
    initialized_ = false;
    return initialized_;
  }
  duration_ = duration;

  // sampling time step
  if (samplingFrequency <= 0) {
    printf("Sampling frequency %f [Hz] of the trajectory is not valid.", samplingFrequency);
    initialized_ = false;
    return initialized_;
  }
  deltaT_ = static_cast<double>(1.0) / samplingFrequency;

  // reset time
  reset();

  initialized_ = true;
  return initialized_;
}

bool GaussianKernelApproximator::learnFromThetas(const std::vector<Eigen::VectorXd>& thetas) {
  if (!initialized_) {
    printf("DMP is not initialized.\n");
    isLearned_ = false;
    return isLearned_;
  }

  if (!setThetas(thetas)) {
    printf("Could not set theta parameters.\n");
    isLearned_ = false;
    return isLearned_;
  }

  printf("Gaussian kernel model learned from Thetas.\n");
  isLearned_ = true;
  return isLearned_;
}

bool GaussianKernelApproximator::getCurrentPosition(Eigen::VectorXd& currentPosition) {
  if (!initialized_) {
    printf("Gaussian kernel model is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if (currentPosition.size() != numSystems_) {
    printf("Provided vector has wrong size (%i), required size is (%i). Cannot get current position.\n",
           static_cast<int>(currentPosition.size()), numSystems_);
    return false;
  }

  for (int i = 0; i < numSystems_; i++) {
    currentPosition(i) = coordinates_.at(i)(_POS_);
  }
  return true;
}

bool GaussianKernelApproximator::getThetas(std::vector<Eigen::VectorXd>& thetas) {
  if (!initialized_) {
    printf("Gaussian kernel model is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  thetas.clear();
  for (int i = 0; i < numSystems_; i++) {
    int numRfs;
    if (!gaussianKernelModels_[i].getNumRFS(numRfs)) {
      printf("Could not get number of receptive fields.\n");
      return false;
    }

    Eigen::VectorXd thetaVector = Eigen::VectorXd::Zero(numRfs);
    if (!gaussianKernelModels_[i].getThetas(thetaVector)) {
      printf("Could not retrieve thetas from Gaussian kernel model %i.\n", i);
      return false;
    }
    thetas.push_back(thetaVector);
  }
  return true;
}

bool GaussianKernelApproximator::setThetas(const std::vector<Eigen::VectorXd>& thetas) {
  if (static_cast<int>(thetas.size()) != numSystems_) {
    printf("Number of thetas (%i) is not equal to number of systems (%i).\n", static_cast<int>(thetas.size()), numSystems_);
    return false;
  }

  for (int i = 0; i < numSystems_; i++) {
    if (!gaussianKernelModels_[i].setThetas(thetas[i])) {
      printf("Could not set thetas of system %i.\n", i);
      return false;
    }
  }
  return true;
}

bool GaussianKernelApproximator::getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers) {
  if (!initialized_) {
    printf("Gaussian kernel model is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  widths.clear();
  centers.clear();

  for (int i = 0; i < numSystems_; i++) {
    int numRfs;
    if (!gaussianKernelModels_[i].getNumRFS(numRfs)) {
      printf("Could not get number of receptive fields.\n");
      return false;
    }

    Eigen::VectorXd centerVector = Eigen::VectorXd::Zero(numRfs);
    Eigen::VectorXd widthsVector = Eigen::VectorXd::Zero(numRfs);
    if (!gaussianKernelModels_[i].getWidthsAndCenters(widthsVector, centerVector)) {
      printf("Could not retrieve thetas from gaussian kernel model %i.\n", i);
      return false;
    }

    widths.push_back(widthsVector);
    centers.push_back(centerVector);
  }
  return true;
}

bool GaussianKernelApproximator::getWidthsAndCenters(const int systemIndex, Eigen::VectorXd& widths, Eigen::VectorXd& centers) {
  if (!initialized_) {
    printf("Gaussian kernel model is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  int numRfs;
  KINDR_ASSERT_TRUE(std::runtime_error, !getNumRFS(systemIndex, numRfs), "Could not get number of RFS.\n");
  KINDR_ASSERT_TRUE(std::runtime_error, widths.size() == numRfs, "");
  KINDR_ASSERT_TRUE(std::runtime_error, centers.size() == numRfs, "");

  if (!gaussianKernelModels_[systemIndex].getWidthsAndCenters(widths, centers)) {
    printf("Could not get widths and centers of system %i.\n", systemIndex);
    return false;
  }
  return true;
}

bool GaussianKernelApproximator::getBasisFunctions(const int numTimeSteps, std::vector<Eigen::MatrixXd>& basisFunctions) {
  if (!initialized_) {
    printf("Gaussian kernel model is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  basisFunctions.clear();
  for (int i = 0; i < numSystems_; i++) {
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
    if (!gaussianKernelModels_[i].generateBasisFunctionMatrix(xInputVector, basisFunctionMatrix)) {
      printf("Gaussian kernel basis function generation failed!\n");
      return false;
    }
    basisFunctions.push_back(basisFunctionMatrix);
  }
  return true;
}

bool GaussianKernelApproximator::getBasisFunctionsValues(const double xInput, std::vector<Eigen::VectorXd>& basisFunctionsValues) {
  if (!initialized_) {
    printf("Gaussian kernel model is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if (basisFunctionsValues.size() != static_cast<size_t>(numSystems_)) {
    printf("basisFunctionsValues vector size (%i) does not match the number of transformation systems %i.\n",
           static_cast<int>(basisFunctionsValues.size()), numSystems_);
    return false;
  }

  for (int i = 0; i < numSystems_; i++) {
    int numRfs;
    if (!getNumRFS(i, numRfs)) {
      return false;
    }

    basisFunctionsValues.at(i).setZero(numRfs);

    if (!gaussianKernelModels_[i].generateBasisFunctionVector(xInput, basisFunctionsValues.at(i))) {
      printf("Gaussian kernel basis function generation failed!\n");
      return false;
    }
  }
  return true;
}

bool GaussianKernelApproximator::getNumRFS(const int systemIndex, int& numRfs) {
  if (!initialized_) {
    printf("Gaussian kernel model is not initialized.\n");
    initialized_ = false;
    return initialized_;
  }

  if ((systemIndex < 0) || (systemIndex >= numSystems_)) {
    printf("Could not get number of receptive fields, the system id (%i) is invalid.\n", systemIndex);
    return false;
  }

  return gaussianKernelModels_[systemIndex].getNumRFS(numRfs);
}

bool GaussianKernelApproximator::getNumRFS(std::vector<int>& numRfs) {
  numRfs.clear();
  for (int i = 0; i < numSystems_; i++) {
    int tmp_numRfs;
    if (!getNumRFS(i, tmp_numRfs)) {
      return false;
    }
    numRfs.push_back(tmp_numRfs);
  }
  return true;
}

bool GaussianKernelApproximator::propagateStep(bool& movementFinished) {
  return propagateStep(movementFinished, duration_, static_cast<int>(duration_ / deltaT_) + 1);
}

bool GaussianKernelApproximator::propagateStep(bool& movementFinished, const double samplingDuration, const int numSamples) {
  if (!isLearned_) {
    printf("Gaussian kernel model is not learned.\n");
    movementFinished = true;
    return false;
  }

  if (numSamples <= 0) {
    printf("Number of samples (%i) is not valid.\n", numSamples);
  }

  double dtTotal = samplingDuration / static_cast<double>(numSamples - 1);
  double dtThreshold = deltaT_;
  int numIteration = ceil(dtTotal / dtThreshold);

  // integrate the system, make sure that all internal variables are set properly
  if (!integrate(dtTotal, numIteration)) {
    printf("Problem while integrating the system.\n");
    movementFinished = true;
    return false;
  }

  sampleIndex_++;

  // check whether movement has finished...
  if (sampleIndex_ >= numSamples) {
    movementFinished = true;
    return true;
  } else {
    movementFinished = false;
  }
  return true;
}

double GaussianKernelApproximator::getProgress() const {
  double progress;

  if (time_ < 0.0) {
    progress = 0.0;
  } else if (time_ > duration_) {
    progress = 1.0;
  } else {
    progress = time_ / duration_;
  }

  return progress;
}

bool GaussianKernelApproximator::reset() {
  time_ = 0.0;
  sampleIndex_ = 0;

  for (int i = 0; i < numSystems_; i++) {
    coordinates_.at(i).setConstant(0.0);
  }

  return true;
}

std::string GaussianKernelApproximator::getInfoString() {
  /*    std::string info("");
      std::stringstream ss;

      //ss << item_id_;
      info.append(std::string("id: ") + ss.str());
      ss.str("");
      ss.clear();

     // info.append(std::string("\t"));
      //info.append(std::string("name: ") + item_name_);

      //info.append(std::string("\n\t"));
      //info.append(std::string("description: ") + description_);

      info.append(std::string("\n\t"));
      info.append(std::string("initialized: "));
      if (initialized_)
      {
          info.append(std::string("true "));
      }
      else
      {
          info.append(std::string("false"));
      }

      info.append(std::string("  learned: "));
      if (params_.isLearned_)
      {
          info.append(std::string("true "));
      }
      else
      {
          info.append(std::string("false"));
      }

      info.append(std::string("  setup: "));
      if (params_.isSetUp_)
      {
          info.append(std::string("true "));
      }
      else
      {
          info.append(std::string("false"));
      }

      info.append(std::string("  is start set: "));
      if (params_.isStartSet_)
      {
          info.append(std::string("true "));
      }
      else
      {
          info.append(std::string("false"));
      }

      info.append(std::string("\n") + params_.getInfoString());
      for (int i = 0; i < params_.numTransformationSystems_; i++)
      {
          info.append(std::string("\n\t"));
          info.append(transformationSystems_[i].getInfoString());
      }

      return info;*/
  return std::string();
}

inline bool GaussianKernelApproximator::integrate(const double dtTotal, const int numIteration) {
  double dt = dtTotal / static_cast<double>(numIteration);

  time_ += dt;

  for (int n = 0; n < numIteration; n++) {
    for (int i = 0; i < numSystems_; i++) {
      // Compute nonlinearity using Gaussian kernel model
      double prediction = 0;
      if (!gaussianKernelModels_[i].predict(getProgress(), prediction, false)) {
        printf("Could not predict output.\n");
        return false;
      }

      coordinates_.at(i)(_POS_) = prediction;
      coordinates_.at(i)(_VEL_) = 0.0;  // TODO(dbellicoso)
      coordinates_.at(i)(_ACC_) = 0.0;  // TODO(dbellicoso)
    }
  }

  return true;
}

}  // namespace dmp
