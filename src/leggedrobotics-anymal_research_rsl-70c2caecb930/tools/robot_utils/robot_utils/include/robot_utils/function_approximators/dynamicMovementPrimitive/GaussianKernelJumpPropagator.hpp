#pragma once

#include "robot_utils/function_approximators/dynamicMovementPrimitive/GaussianKernel.hpp"
#include "tinyxml.h"

class GaussianKernelJumpPropagator {
 public:
  GaussianKernelJumpPropagator();
  virtual ~GaussianKernelJumpPropagator();

  virtual bool loadTrajectory(const TiXmlHandle& hJump);
  bool inVelocityMode();

  bool initialize(double dt);
  double getMaxDuration();
  double getProgress();
  Eigen::VectorXd predict();

 protected:
  Eigen::VectorXd thetas_;
  std::vector<dmp::GaussianKernel> desiredTrajectories_;

  virtual bool loadGaussianKernel(TiXmlElement* pElem, dmp::GaussianKernel* desiredTrajectory);
  virtual bool loadMovement(const TiXmlHandle& hTrajectory);
  virtual bool loadThetas(const TiXmlHandle& hThetas);

 private:
  std::vector<double> values_;
  double timeStep_ = 0.0;
  double currentTime_ = 0.0;
  double maxDuration_ = 0.0;
  bool velocityMode_ = false;

  void resetTime();
  void incrementTime();
};
