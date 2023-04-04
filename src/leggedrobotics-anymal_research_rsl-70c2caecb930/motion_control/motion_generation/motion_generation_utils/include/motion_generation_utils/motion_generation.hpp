/*
 * motion_generation.hpp
 *
 *  Created on: Jan 26, 2017
 *      Author: Dario Bellicoso, Fabian Jenelten
 */

#pragma once

//motion_generation
#include <motion_generation_utils/typedefs.hpp>

// eigen
#include "Eigen/Core"
#include <Eigen/StdVector>

// std utils
#include <std_utils/ConsecutiveEnum.hpp>

// boost
#include <boost/math/special_functions.hpp>

namespace zmp {

enum LegId : unsigned int {
  legIdLF = 0u,
  legIdRF = 1u,
  legIdLH = 2u,
  legIdRH = 3u
};

/*
 * Termination state of optimization
 * > returnOptimizedSolution          optimization was successful
 * > returnInitialGuess               initial guess is returned, optimization was not performed
 * > returnApproximation              some approximized solution is returned, initial guess was not computed and optimization was not performed.
 * > failedToSetUpOptimizationProblem something went wrong in setting up the optimization
 * > failedToSolveOptimization        optimization failed
 * > failedToCreateSolution           optimization succeeded, but the solution spline could not be created
 * > invalidMotionPlan                optimization succceeded, but motion plan appears to be invalid.
 */
CONSECUTIVE_ENUM(TerminationState,
    returnOptimizedSolution,
    returnInitialGuess,
    failedToSetUpOptimizationProblem,
    failedToSolveOptimization,
    failedToCreateSolution,
    failedToReadMotionPlan,
    invalidSolution,
    Undefined)
static std::map<TerminationState, std::string> terminationStateMap =
{{TerminationState::returnOptimizedSolution,            "return optimized solution"},
 {TerminationState::returnInitialGuess,                 "return initial guess"},
 {TerminationState::failedToSetUpOptimizationProblem,   "failed to set up optimization problem"},
 {TerminationState::failedToSolveOptimization,          "failed to solve optimization"},
 {TerminationState::failedToCreateSolution,             "failed to createSolution"},
 {TerminationState::failedToReadMotionPlan,             "failed to read motion plan"},
 {TerminationState::invalidSolution,                    "invalid solution"},
 {TerminationState::Undefined,                          "Undefined"},
 {TerminationState::SIZE,                               "SIZE"}};


/*
 * Enumeration of the derivative space:
 *  [] zero   -> position or angle
 *  [] first  -> linear or angular velocity
 *  [] second -> linear or angular acceleration
 */
CONSECUTIVE_ENUM(Derivative, Zero, First, Second)
static std::map<Derivative, std::string> derivativeMap =
{{Derivative::Zero,     "Zero"},
 {Derivative::First,    "First"},
 {Derivative::Second,   "Second"},
 {Derivative::SIZE,      "SIZE"}};

// Enumeration of COG state space.
CONSECUTIVE_ENUM(CogDim, x, y, z, yaw, pitch, roll)
static std::map<CogDim, std::string> cogDimMap =
{{CogDim::x,     "x"},
 {CogDim::y,     "y"},
 {CogDim::z,     "z"},
 {CogDim::yaw,   "yaw"},
 {CogDim::pitch, "pitch"},
 {CogDim::roll,  "roll"},
 {CogDim::SIZE,  "SIZE"}};

constexpr std::array<zmp::CogDim, 2> optimizationXYDofs = {CogDim::x, CogDim::y};
constexpr std::array<zmp::CogDim, 3> optimizationTranslationalDofs = {CogDim::x, CogDim::y, CogDim::z};
constexpr std::array<zmp::CogDim, 3> optimizationRotationalDofs = {CogDim::yaw, CogDim::pitch, CogDim::roll};
constexpr std::array<zmp::CogDim, 6> optimizationTranslationalRotationalDofs = {CogDim::x, CogDim::y, CogDim::z, CogDim::yaw, CogDim::pitch, CogDim::roll};

//! True if CogDim-enum corresponds to a translational state.
inline bool isTranslation(CogDim dim) {
  return (dim==zmp::CogDim::x || dim==zmp::CogDim::y || dim==zmp::CogDim::z);
}

//! True if CogDim-enum corresponds to a rotational state.
inline bool isRotation(CogDim dim) {
  return (dim==zmp::CogDim::yaw || dim==zmp::CogDim::pitch || dim==zmp::CogDim::roll);
}

inline bool containsTranslation(const std::vector<zmp::CogDim>& optimizationDofs) {
  for (const auto& dim : optimizationDofs) {
    if (isTranslation(dim)) { return true; }
  }
  return false;
}

inline bool containsRotation(const std::vector<zmp::CogDim>& optimizationDofs) {
  for (const auto& dim : optimizationDofs) {
    if (isRotation(dim)) { return true; }
  }
  return false;
}

inline const std::vector<zmp::CogDim> computeUnionsSet(const std::vector<zmp::CogDim>& set1, const std::vector<zmp::CogDim>& set2) {
  std::vector<zmp::CogDim> unionSet;
  unionSet.reserve(std::max(set1.size(), set2.size()));
  for (const auto& dim : optimizationTranslationalRotationalDofs) {
    if (std_utils::containsEnum(set1, dim) || std_utils::containsEnum(set2, dim)) {
      unionSet.push_back(dim);
    }
  }
  return unionSet;
}

//! Transforms CoGDim enum to index of a 3d vector.
inline int toIndex(CogDim dim) noexcept {
  if (optimizationTranslationalDofs[0] == dim || optimizationRotationalDofs[0] == dim) { return 0; }
  if (optimizationTranslationalDofs[1] == dim || optimizationRotationalDofs[1] == dim) { return 1; }
  if (optimizationTranslationalDofs[2] == dim || optimizationRotationalDofs[2] == dim) { return 2; }
  std::cout << "[toIndex] Unknown dimension " << cogDimMap[dim] << "!\n";
  return -1;
}

//! States the position of dim within the optimizationTranslationalRotationalDofs vector.
inline int toStackedIndex(CogDim dim) noexcept {
  for (auto id = 0u; id<optimizationTranslationalRotationalDofs.size(); ++id) {
    if (optimizationTranslationalRotationalDofs[id] == dim) { return id; }
  }
  std::cout << "[toStackedIndex] Unknown dimension " << cogDimMap[dim] << "!\n";
  return -1;
}

struct LineSearchOptions {
  LineSearchOptions() :
    tol_(1e-4),
    maxIter_(12u),
    maxIterBackTraces_(10u),
    alpha_(0.4),
    beta_(0.6),
    verbose_(false),
    rate_(2.0) {
  }

  LineSearchOptions(
      double tol,
      unsigned int maxIter,
      unsigned int maxIterBackTraces,
      double alpha,
      double beta,
      bool verbose,
      double rate) :
    tol_(tol),
    maxIter_(maxIter),
    maxIterBackTraces_(maxIterBackTraces),
    alpha_(alpha),
    beta_(beta),
    verbose_(verbose),
    rate_(rate) {
  }

  //! Tolerance for line search.
  double tol_;

  //! max iteration for Newton iterations.
  unsigned int maxIter_;

  //! max iterations for backtracing line search.
  unsigned int maxIterBackTraces_;

  //! backtracing parameters.
  double alpha_;  // alpha in (0,0.5)
  double beta_;   // beta in (0,1)

  //! If true, displays result of line search.
  bool verbose_;

  //! Plotting.
  double rate_;
};

// Enumeration of coefficients for (quintic) splines.
CONSECUTIVE_ENUM(Coeff, a5, a4, a3, a2, a1, a0)

constexpr unsigned int splineCoeffs       = static_cast<unsigned int>(Coeff::SIZE);  // number of coefficients to approximate a spline segment
constexpr unsigned int dofTransl          = 3u;                                      // dimension space for translation (x,y,z)
constexpr unsigned int dofRot             = 3u;                                      // dimension space for rotation (yaw, pitch, roll)
constexpr unsigned int dofFullState       = static_cast<unsigned int>(CogDim::SIZE); // number of states (translation + rotation)
constexpr unsigned int coeffsTransl       = dofTransl*splineCoeffs;                  // dimension space for translation (x,y,z) spline based
constexpr unsigned int coeffsRot          = dofRot*splineCoeffs;                     // dimension space for rotation (yaw, pitch, roll) spline based
constexpr unsigned int coeffsFullState    = dofFullState*splineCoeffs;               // number of state per spline

using ZmpOptState = std_utils::EnumArray<CogDim, double>;
using ZmpOptStateVector = std_utils::EnumArray<Derivative, ZmpOptState>;

} /* namespace zmp */
