/*
 * NelderMeadOptimizer.cpp
 *
 *  Created on: Jul 19, 2015
 *      Author: Dario Bellicoso, Remo Diethelm
 */


#include "neldermead_optimizer/NelderMeadOptimizer.hpp"

namespace neldermead_optimizer {


NelderMeadOptimizer::NelderMeadOptimizer() {}
NelderMeadOptimizer::~NelderMeadOptimizer() {}


bool NelderMeadOptimizer::optimize(double& fOpt, Eigen::VectorXd& xOpt, const Eigen::VectorXd& x0,
                                   const Fcn& fcn, bool minimize, long int maxIterations,
                                   double maxFValueDiff, double maxVertexDiff) {

  // Set run time variables.
  numCoordinates_ = x0.size();
  numVertices_ = numCoordinates_ + 1;
  s_ = numVertices_ - 2;
  h_ = numVertices_ - 1;

  simplex_ = Eigen::MatrixXd::Zero(numCoordinates_, numVertices_);
  fValues_.resize(numVertices_, 0.0);

  currentIteration_ = 0;

  // Set optimization parameters.
  x0_ = x0;
  fcn_ = fcn;
  minimize_ = minimize;
  maxIterations_ = (maxIterations == -1) ? 200 * numCoordinates_ : maxIterations;
  maxFValueDiff_ = maxFValueDiff;
  maxVertexDiff_ = maxVertexDiff;

  setupSimplex();

  while (!terminateOptimization()) {
    runMinimizationStep();
  }

  if (!minimize_) {
    fOpt = -fValues_[0];
  } else {
    fOpt = fValues_[0];
  }

  xOpt = simplex_.col(0);

  return true;
}


int NelderMeadOptimizer::getIterations() {
  return currentIteration_;
}


void NelderMeadOptimizer::setupSimplex()
{
  // Set initial point
  simplex_.col(0) = x0_;
  fValues_[0] = evalFcn(x0_);

  // Setup other simplex points
  const double usual_d = 0.05;
  const double zero_term_d = 0.00025;

  for (int k = 0; k < numCoordinates_; k++) {
    Eigen::VectorXd y = x0_;
    if (y[k] != 0) {
      y[k] = (1 + usual_d) * y[k];
    } else {
      y[k] = zero_term_d;
    }

    simplex_.col(k + 1) = y;
    fValues_[k + 1] = evalFcn(y);
  }

  sortSimplex();
}


double NelderMeadOptimizer::evalFcn(const Eigen::VectorXd& x) {
  if (minimize_) return fcn_(x);
  else return -fcn_(x);
}


void NelderMeadOptimizer::runMinimizationStep() {

  // Compute the centroid of the best n points.
  Eigen::VectorXd c = Eigen::VectorXd::Zero(numCoordinates_);
  for (int k=0; k<h_; k++) {
    c += simplex_.col(k);
  }
  c /= h_;

  // Compute the reflection point.
  const Eigen::VectorXd xr = (1.0+rho_)*c - rho_*simplex_.col(h_);
  const double fr = evalFcn(xr);

  if (fr < fValues_[0]) {
    // Compute the expansion point.
    const Eigen::VectorXd xe = (1.0+rho_*chi_)*c - rho_*chi_*simplex_.col(h_);
    const double fe = evalFcn(xe);

    if (fe < fr) {
      // Expansion.
      simplex_.col(h_) = xe;
      fValues_[h_] = fe;
    } else {
      // Reflection.
      simplex_.col(h_) = xr;
      fValues_[h_] = fr;
    }

  } else {
    if (fr < fValues_[s_]) {
      // Reflection.
      simplex_.col(h_) = xr;
      fValues_[h_] = fr;
    } else {
      // Contraction.
      if (fr < fValues_[h_]) {
        const Eigen::VectorXd xc = (1.0+psi_*rho_)*c - psi_*rho_*simplex_.col(h_);
        const double fc = evalFcn(xc);

        if (fc <= fr) {
          // Outside contraction.
          simplex_.col(h_) = xc;
          fValues_[h_] = fc;
        } else {
          // Shrink.
          shrinkSimplex();
        }

      } else {
        const Eigen::VectorXd xcc = (1.0-psi_)*c + psi_*simplex_.col(h_);
        const double fcc = evalFcn(xcc);

        if (fcc < fValues_[h_]) {
          // Inside contraction.
          simplex_.col(h_) = xcc;
          fValues_[h_] = fcc;
        } else {
          // Shrink.
          shrinkSimplex();
        }
      }
    }
  }

  sortSimplex();

  currentIteration_++;

  //printSimplex();
}


bool NelderMeadOptimizer::terminateOptimization() {
  // Check for number of iterations.
  if (currentIteration_ >= maxIterations_) {
    return true;
  }

  // Check for relative difference between f values.
  double maxFValueDiff = 0.0;
  for (size_t k=1; k<fValues_.size(); k++) {
    maxFValueDiff = std::max(maxFValueDiff, std::fabs(fValues_[0]-fValues_[k]));
  }

  // Check for relative difference between vertices.
  Eigen::MatrixXd simplexFirstColCopy(numCoordinates_, numVertices_-1);
  simplexFirstColCopy.setZero();

  for (int k=0; k<simplexFirstColCopy.cols(); k++) {
    simplexFirstColCopy.col(k) = simplex_.col(0);
  }

  const double maxVertexDiff = ((simplex_.block(0, 1, numCoordinates_, numVertices_-1) - simplexFirstColCopy).cwiseAbs()).maxCoeff();

  // Check if smaller than tolerance.
  if (maxFValueDiff <= maxFValueDiff_ && maxVertexDiff <= maxVertexDiff_) {
    return true;
  }

  return false;
}


void NelderMeadOptimizer::sortSimplex() {
  // Get sorted indexes.
  std::vector<int> indexes = internal::getSortedIndexes(fValues_);

  // Create local copies.
  Eigen::MatrixXd simplexCopy(simplex_);
  std::vector<double> fValuesCopy(fValues_);

  // Sort.
  for (size_t k=0; k<indexes.size(); k++) {
    simplexCopy.col(k) = simplex_.col(indexes[k]);
    fValuesCopy[k] = fValues_[indexes[k]];
  }

  // Update.
  simplex_ = simplexCopy;
  fValues_ = fValuesCopy;
}


void NelderMeadOptimizer::shrinkSimplex() {
  for (int k=1; k<numVertices_; k++) {
    simplex_.col(k) = simplex_.col(0) + sigma_*(simplex_.col(k)-simplex_.col(0));
    fValues_[k] = evalFcn(simplex_.col(k));
  }
}


void NelderMeadOptimizer::printSimplex() const {
  std::cout << "--------------------------------" << std::endl;
  std::cout << "Iteration: " << currentIteration_ << std::endl;
  std::cout << "Fcn values:";
  for (auto fValue : fValues_) {
    std::cout << " " << fValue;
  }
  std::cout << std::endl;
  std::cout << "Simplex:" << std::endl;
  std::cout << simplex_ << std::endl;
  std::cout << "--------------------------------" << std::endl << std::endl;
}

} /* namespace neldermead_optimizer */
