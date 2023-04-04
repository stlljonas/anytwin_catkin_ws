/*
 * GaussianBaseTrajectory.hpp
 *
 *  Created on: Oct 17, 2014
 *      Author: huub
 */

#pragma once

#include <vector>

namespace robot_utils {

class GaussianBaseTrajectory {
 public:
  using Parameters = std::vector<double>;
  using BaseValues = std::vector<double>;

 public:
  GaussianBaseTrajectory();
  ~GaussianBaseTrajectory();

  void initialize(const Parameters& par, double timeHorizon, double dt);
  double advance();
  double next() const;
  double previous() const;
  double getCurrentSecondDerivative() const;
  double getCurrentDerivative() const;
  double getCurrentValue() const;
  double getValueAtTimeFromNow(double time, double dt) const;
  void reset();

  void setParameters(const Parameters& parameters);
  void setSigmaOfBasisFunctions(double sigma);
  void setNumberOfBasisFunctions(int numberOfBasisFunctions);
  void setDurationOfTrajectory(double timeHorizon);
  void setTimeStep(double dt);

  double getLengthOfTrajectory() const;
  double getSigma() const;
  int getNumberOfBasisFunctions() const;
  Parameters getParameters() const;

 protected:
  double sigma_ = 0.1;
  double timeBetweenFirstAndLastParametrizedBaseFunction_ = 1.0;
  int numberOfBasisfunctions_ = 5;
  double basisScaling_ = 1.0;
  double timeStep_ = 0.0;
  Parameters parameters_;
  double currentTime_ = 0.0;

  double previousValue_ = 0.0;
  double currentValue_ = 0.0;
  double currentDerivativeValue_ = 0.0;
  double currentSecondDerivativeValue_ = 0.0;

  std::vector<double> values_;
  std::vector<double> derivativeValues_;
  std::vector<double> secondDerivativeValues_;

  void setScalingOfBasisFunctions();

  void generateValueVectors();
  double getValues(double& value, double& derivativeValue, double& secondDerivativeValue, double t) const;
  void generateBasisFunctionVectors(BaseValues& baseValues, BaseValues& baseDerivatives, BaseValues& baseSecondDerivatives, double time,
                                    double sigma, double timeBetweenFirstAndLastParametrizedBaseFunction, int numberOfBasisFunctions) const;
  double getValueOfGaussian(double x, double mu, double sigma) const;
  double calculateValuesOfTrajectory(const BaseValues& basisVector, const Parameters& parameters) const;

  Parameters elementWiseVectorMultiplication(const BaseValues& vecB, const Parameters& vecP) const;
  double sum(const Parameters& parameters) const;
};

}  // namespace robot_utils
