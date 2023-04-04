/*
 * TaskHandlerBase.hpp
 *
 *  Created on: March 03, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include <motion_generation_utils/motion_generation.hpp>

// eigen
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/StdVector>

// curves
#include <curves/PolynomialSplineContainer.hpp>

// robot utils
#include <robot_utils/geometry/geometry.hpp>

// stl
#include <memory>

// messagge logger
#include "message_logger/message_logger.hpp"

// boost
#include <boost/math/special_functions/pow.hpp>

// robot utils
#include <robot_utils/math/math.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

 // boost
#include <cassert>

#define EIGEN_INITIALIZE_MATRICES_BY_NAN

class TiXmlHandle;


namespace motion_gen {

class TaskHandlerBase {
 public:
  using SplineType = curves::PolynomialSplineQuintic;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using TimeVector              = Eigen::Matrix<double,1,zmp::splineCoeffs>;
  using StackedTimeVector       = std_utils::EnumArray<zmp::Derivative, TimeVector>;
  using TimeMatrix              = Eigen::Matrix<double,zmp::splineCoeffs,zmp::splineCoeffs>;
  using StackedTimeMatrix       = std_utils::EnumArray<zmp::Derivative, TimeMatrix>;

  using TimeVectorTranspose     = Eigen::Matrix<double, zmp::splineCoeffs,1>;

  using VectorXY                = Eigen::Matrix<double,2,1>;
  using TimeMatrixXY            = Eigen::Matrix<double,2,zmp::splineCoeffs>;

  using TimeMatrixAccel         = Eigen::Matrix<double,zmp::splineCoeffs-2,zmp::splineCoeffs-2>;

  using TimeMatrixTransl        = Eigen::Matrix<double,zmp::dofTransl,zmp::coeffsTransl>;
  using StackedTimeMatrixTransl = std_utils::EnumArray<zmp::Derivative, TimeMatrixTransl>;

  using TranslCoeffsVector      = Eigen::Matrix<double,1,zmp::coeffsTransl>;

  TaskHandlerBase():
      optimizationDofs_(),
      activeSplineId_(0),
      timeVectors_ (TimeVector::Zero()),
      timeVectors0_(TimeVector::Zero()),
      timeVectorsF_(TimeVector::Zero()),
      translTimeMatrices_(TimeMatrixTransl::Zero()),
      HessianMinAcceleration1D_(TimeMatrixAccel::Zero()),
      HessianTracking1D_ (TimeMatrix::Zero()),
      HessianTracking1D0_(TimeMatrix::Zero()),
      HessianTracking1DF_(TimeMatrix::Zero()),
      numOfTranslationalStates_(0.0),
      numOfRotationalStates_(0.0)

  {
  }

  virtual ~TaskHandlerBase() = default;

  bool setOptimizationDofs(const std::vector<zmp::CogDim>& optimizationDofs) {
    optimizationDofs_ = optimizationDofs;
    numOfTranslationalStates_ = 0u;
    numOfRotationalStates_ = 0u;
    for (const auto& dim : optimizationDofs) {
      if (zmp::isTranslation(dim)) { numOfTranslationalStates_++; }
      else if (zmp::isRotation(dim)) { numOfRotationalStates_++; }
      else {
        MELO_WARN_STREAM("[TaskHandlerBase::setOptimizationDofs] Unknown  dim.");
        return false;
      }
    }
    return true;
  }

  bool initialize() {
    optimizationDofs_.clear();

    // Evaluate time vectors at zero time.
    for (const auto derivative : std_utils::enum_iterator<zmp::Derivative>()) {
      getTimeVectorAtTime(timeVectors0_[derivative], 0.0, derivative);
      HessianTracking1D0_[derivative] = timeVectors0_[derivative].transpose()*timeVectors0_[derivative];
    }

    return true;
  }

  bool loadParameters(const TiXmlHandle& /* handle */) {
    return true;
  }

  //! Returns number of degrees of freedom
  const std::vector<zmp::CogDim>& getOptimizationDofs() const {
    return optimizationDofs_;
  }

  unsigned int getNumOfRotationalStates() const noexcept {
    return numOfRotationalStates_;
  }

  unsigned int getNumOfTranslationalStates() const noexcept {
    return numOfTranslationalStates_;
  }


  /*
   * !The function maps zmp::CogDim enums to an int s.t. the ints represents the index of the enum
   * within optimizationDofs_.
   * Example:
   *    zmp::CogDim       = {x=0, y=1, z=2, phi=3, theta=4, psi=5}
   *    optimizationDofs_ = {x=0, y=1, phi=2}
   *    remapToOptVariables(zmp::CogDim::phi) will return 2
   * Assumptions: zmp::CogDim must be of type CONSECUTIVE_ENUM!!
   */
  unsigned int remapToOptVariables(zmp::CogDim dim) const {
    unsigned int id = 0u;
    for (const auto& optDim : optimizationDofs_) {
      if (dim == optDim) { return id; }
      id++;
    }

    MELO_WARN_STREAM("[TaskHandlerBase::cogToInt] Index not found");
    return id;
  }

  //! Returns the coefficient index. Coefficients are enumerated as follows:
  // a = [a0 a1 ... aK] with K+1 splines
  //     > for spline i:           ai = [aix aiy aiz]
  //     > for spline i, dim j:   aij = [aij5 aij4 ... aij0] with j = {x, y, z}
  inline int getCoeffIndex(unsigned int splineId, zmp::CogDim dimId, zmp::Coeff coeffId) const {
    return ( zmp::splineCoeffs*(optimizationDofs_.size()*splineId + remapToOptVariables(dimId)) + static_cast<unsigned int>(coeffId) );
  }

  //! Returns the first index of the spline (aij5).
  inline int getCoeffStartIndex(unsigned int splineId, zmp::CogDim dimId = zmp::CogDim::x) const {
    return ( zmp::splineCoeffs*(optimizationDofs_.size()*splineId + remapToOptVariables(dimId)) );
  }

  //! Returns the first index assocaited to position (aij0).
  inline int getPositionIndex(unsigned int splineId, zmp::CogDim dimId = zmp::CogDim::x) const {
    return ( zmp::splineCoeffs*(optimizationDofs_.size()*splineId + remapToOptVariables(dimId)) + zmp::splineCoeffs - 1u);
  }

  //! Returns first coefficient index of translational states.
  inline int getCoeffStartIndexTranslation(unsigned int splineId) const {
    for (const auto& dim : zmp::optimizationTranslationalDofs) {
      if (std_utils::containsEnum(optimizationDofs_,dim)) {
        return getCoeffStartIndex(splineId, dim);
      }
    }
    std::cout << "[TaskHandlerBase::getCoeffStartIndexTranslation] dim not found!\n";
    return -1;
  }

  //! Returns first coefficient index of rotational states.
  inline int getCoeffStartIndexRotation(unsigned int splineId) const {
    for (const auto& dim : zmp::optimizationRotationalDofs) {
      if (std_utils::containsEnum(optimizationDofs_,dim)) {
        return getCoeffStartIndex(splineId, dim);
      }
    }
    std::cout << "[TaskHandlerBase::getCoeffStartIndexRotation] dim not found!\n";
    return -1;
  }

  //! Compute time vector (corresponding to 0th, 1th or 2th derivative).
  void getTimeVectorAtTime(
      Eigen::Ref<TimeVector> timeVec,
      double t_k,
      zmp::Derivative derivative) const {
    assert(t_k>=0.0);
    if (t_k == 0.0) {
      if      (derivative == zmp::Derivative::Zero)   { SplineType::getTimeVectorAtZero(timeVec); }
      else if (derivative == zmp::Derivative::First)  { SplineType::getDTimeVectorAtZero(timeVec); }
      else if (derivative == zmp::Derivative::Second) { SplineType::getDDTimeVectorAtZero(timeVec); }
    }

    else {
      if      (derivative == zmp::Derivative::Zero)   { SplineType::getTimeVector(timeVec, t_k); }
      else if (derivative == zmp::Derivative::First)  { SplineType::getDTimeVector(timeVec, t_k); }
      else if (derivative == zmp::Derivative::Second) { SplineType::getDDTimeVector(timeVec, t_k); }
    }
  }

  //! Returns time vector (corresponding to 0th, 1th or 2th derivative).
  const TimeVector& getTimeVector(zmp::Derivative derivative) const {
    return timeVectors_[derivative];
  }

  //! Returns time vector (corresponding to 0th, 1th or 2th derivative) at zero time.
  const TimeVector& getTimeVector0(zmp::Derivative derivative) const {
    return timeVectors0_[derivative];
  }

  //! Returns time vector (corresponding to 0th, 1th or 2th derivative) at final time.
  const TimeVector& getTimeVectorF(zmp::Derivative derivative) const {
    return timeVectorsF_[derivative];
  }

  //! Performs sample time based computations. t_k is the accumulated time for a spline.
  void setTime(double t_k) {
    for (const auto derivative : std_utils::enum_iterator<zmp::Derivative>()) {
      getTimeVectorAtTime(timeVectors_[derivative], t_k, derivative);
      HessianTracking1D_[derivative] = timeVectors_[derivative].transpose()*timeVectors_[derivative];

      translTimeMatrices_[derivative].setZero();
      for (unsigned int dim=0u; dim<zmp::dofTransl; ++dim) {
        translTimeMatrices_[derivative].block(dim,dim*zmp::splineCoeffs,1,zmp::splineCoeffs) = timeVectors_[derivative];
      }
    }
  }

  //! Performs spline time based computations.
  bool setActiveSplineId(unsigned int activeSplineId, double splineDuration) {
    activeSplineId_ = activeSplineId;
    if(!getAccelerationHessian(HessianMinAcceleration1D_, splineDuration)) { return false; }

    for (const auto derivative : std_utils::enum_iterator<zmp::Derivative>()) {
      getTimeVectorAtTime(timeVectorsF_[derivative], splineDuration, derivative);
      HessianTracking1DF_[derivative] = timeVectorsF_[derivative].transpose()*timeVectorsF_[derivative];
    }
    return true;
  }

  //! Objective for approaching the state to a certain point in state space.
  bool getStateObjective(
      Eigen::Ref<TimeMatrix> hessian,
      Eigen::Ref<TimeVectorTranspose> linearTerm,
      double desiredState,
      zmp::Derivative derivative,
      double weight) const {
    /*
     * Objective:
     *    w(r-r_des)^2 = (r-r_des)'w(r-r_des) = r'wr - 2*r_des'r + r_des'r_des
     * Substituting spline coefficients
     *    a'T'Ta - 2*r_des'Ta
     * Notice that the QP will solve 0.5*x'Qx+b'x, i.e., (objetive multiplied with 2)
     *    Q = T'T
     *    b = -r_des'*Ta
     *
     */
    assert(weight>=0.0);

    hessian.triangularView<Eigen::Lower>() += HessianTracking1D_[derivative]*weight;
    linearTerm                             += -desiredState*weight*timeVectors_[derivative].transpose();
    return true;
  }

  //! Objective for approaching the initial state to a certain point in state space.
  bool getInitialStateObjective(
      Eigen::Ref<TimeMatrix> hessian,
      Eigen::Ref<TimeVectorTranspose> linearTerm,
      double desiredState,
      zmp::Derivative derivative,
      double weight) const {
    assert(weight>=0.0);

    hessian.triangularView<Eigen::Lower>() += HessianTracking1D0_[derivative]*weight;
    linearTerm                             += -desiredState*weight*timeVectors0_[derivative].transpose();
    return true;
  }

  //! Objective for approaching the final state to a certain point in state space.
  bool getFinalStateObjective(
      Eigen::Ref<TimeMatrix> hessian,
      Eigen::Ref<TimeVectorTranspose> linearTerm,
      double desiredState,
      zmp::Derivative derivative,
      double weight) const {
    assert(weight>=0.0);

    hessian.triangularView<Eigen::Lower>() += HessianTracking1DF_[derivative]*weight;
    linearTerm                             += -desiredState*weight*timeVectorsF_[derivative].transpose();
    return true;
  }


  //! Objective for minimizing a one dimensional acceleration over a quintic spline segment.
  bool getAccelerationObjective(
      Eigen::Ref<TimeMatrixAccel> hessian,
      double weight) const {
    assert(weight>=0.0);
    hessian.triangularView<Eigen::Lower>() += HessianMinAcceleration1D_*weight;
    return true;
  }

  //!This constraint enforces the CoG state to lay within a box (-boundMin, +boundMax) centered at center.
  // Both boundaries must be larger than zero. By default, boundMax = boundMin.
  bool getMaxStateInequalityConstraints(
      Eigen::Ref<TimeMatrixXY> inequalityMat,
      Eigen::Ref< VectorXY> inequalityVecMin,
      zmp::Derivative derivative,
      double center,
      double boundMin,
      double boundMax = -1.0) const {

    inequalityMat.block<1, zmp::splineCoeffs>(0, 0) = timeVectors_[derivative];
    // fixme: is eval() needed?
    inequalityMat.block<1, zmp::splineCoeffs>(1, 0) = -timeVectors_[derivative];

    if (boundMin<0.0) { return false; }
    if (boundMax<0.0) { boundMax = boundMin; }

    inequalityVecMin(0) = center-boundMin;     // min -> T*a >= center-bound
    // fixme: is eval() needed?
    inequalityVecMin(1) = -(center+boundMax);  // max -> T*a <= center+bound

    return true;
  }

  //!This constraint enforces the CoG state to lay within a box (boundMin, boundMax)
  bool getMaxStateInequalityConstraintsEffective(
      Eigen::Ref<TimeMatrixXY> inequalityMat,
      Eigen::Ref< VectorXY> inequalityVecMin,
      zmp::Derivative derivative,
      double boundMin,
      double boundMax) const {

    inequalityMat.block<1, zmp::splineCoeffs>(0, 0) = timeVectors_[derivative];
    inequalityMat.block<1, zmp::splineCoeffs>(1, 0) = -timeVectors_[derivative];

    inequalityVecMin(0) = boundMin;  // min -> T*a >= boundMin
    // fixme: is eval() needed?
    inequalityVecMin(1) = -boundMax; // max -> T*a <= boundMax

    return true;
  }

  //!This constraint enforces the final CoG state to lay within a box (-boundMin, +boundMax) centered at center.
  // Both boundaries must be larger than zero. By default, boundMax = boundMin.
  // The the time is asusmed to be equal to the final spline time.
  template<typename Derived1, typename Derived2>
  bool getFinalMaxStateInequalityConstraints(
      Eigen::MatrixBase<Derived1> const & inequalityMat,
      Eigen::MatrixBase<Derived2> const & inequalityVecMin,
      zmp::Derivative derivative,
      double center,
      double boundMin,
      double boundMax = -1.0) const {

    (const_cast<Eigen::MatrixBase<Derived1>&>(inequalityMat)).template block<1,zmp::splineCoeffs>(0,0) = timeVectorsF_[derivative];
    (const_cast<Eigen::MatrixBase<Derived1>&>(inequalityMat)).template block<1,zmp::splineCoeffs>(1,0) = -timeVectorsF_[derivative];

    if (boundMin<0.0) { return false; }
    if (boundMax<0.0) { boundMax = boundMin; }

    // min -> T*a >= center-bound, max -> T*a <= center+bound
    (const_cast<Eigen::MatrixBase<Derived2>&>(inequalityVecMin))(0) =  center-boundMin;
    (const_cast<Eigen::MatrixBase<Derived2>&>(inequalityVecMin))(1) = -center-boundMax;

    return true;
  }

  //! Force the CoG state to a certain point in state space.
  void getStateEqualityConstraints(Eigen::Ref<TimeVector> equalityMat, zmp::Derivative derivative) const {
    equalityMat = timeVectors_[derivative];
  }

  //! Set CoG initial conditions.
  template<typename Derived>
  void getInitialStateEqualityConstraints(Eigen::MatrixBase<Derived> const & equalityMat, zmp::Derivative derivative) const {
    const_cast<Eigen::MatrixBase<Derived>&>(equalityMat) = timeVectors0_[derivative];
  }

  //! Set CoG final conditions.
  template<typename Derived>
  void getFinalStateEqualityConstraints(Eigen::MatrixBase<Derived> const & equalityMat, zmp::Derivative derivative) const {
    const_cast<Eigen::MatrixBase<Derived>&>(equalityMat) = timeVectorsF_[derivative];
  }

  //! Constraints that smoothly connect two adjacent splines.
  template<typename Derived>
  bool getJunctionConstraints(
      Eigen::MatrixBase<Derived> const & equalityMatCurrentSpline,
      Eigen::MatrixBase<Derived> const & equalityMatNextSpline,
      double& equalityVec,
      zmp::Derivative derivative) const {
    const_cast<Eigen::MatrixBase<Derived>&>(equalityMatCurrentSpline) = -timeVectorsF_[derivative];
    const_cast<Eigen::MatrixBase<Derived>&>(equalityMatNextSpline)    = timeVectors0_[derivative];
    equalityVec = 0.0;
    return true;
  }

 protected:
  //! Compute objective for minimizing a one dimensional acceleration.
  bool getAccelerationHessian(
      Eigen::Ref<TimeMatrixAccel> hessian,
      double tf) const {
    const double tf2 = tf*tf;
    const double tf3 = tf2*tf;
    const double tf4 = tf3*tf;
    const double tf5 = tf4*tf;
    const double tf6 = tf5*tf;
    const double tf7 = tf6*tf;

    static_assert(zmp::splineCoeffs==6u, "Spline coefficient size should be 6");

    hessian << 57.142857143*tf7, 40.0*tf6,       24.0*tf5,       10.0*tf4,
               40.0*tf6,         28.8*tf5,       18.0*tf4,       8.0*tf3,
               24.0*tf5,         18.0*tf4,       12.0*tf3,       6.0*tf2,
               10.0*tf4,         8.0*tf3,        6.0*tf2,        4.0*tf;

    return true;
  }

  //! Number of DoFs (x,y,z,...).
  std::vector<zmp::CogDim> optimizationDofs_;

  // Currently active spline index.
  unsigned int activeSplineId_;

  //! Vector consisting of time vectors (corresponding to 0th, 1th or 2th derivative).
  StackedTimeVector timeVectors_;

  //! Vector consisting of time vectors (corresponding to 0th, 1th or 2th derivative) at zero time.
  StackedTimeVector timeVectors0_;

  //! Vector consisting of time vectors (corresponding to 0th, 1th or 2th derivative) at final time.
  StackedTimeVector timeVectorsF_;

  //! Matrix consisting of time vectors (corresponding to 0th, 1th or 2th derivative) for extracting translational state.
  StackedTimeMatrixTransl translTimeMatrices_;

  //! Hessian for minimizing one-dimensional acceleration.
  TimeMatrixAccel HessianMinAcceleration1D_;

  //! Hessian for approaching solution to a point in state space (corresponding to 0th, 1th or 2th derivative).
  StackedTimeMatrix HessianTracking1D_;

  //! Hessian for approaching solution to a point in state space (corresponding to 0th, 1th or 2th derivative) at zero time.
  StackedTimeMatrix HessianTracking1D0_;

  //! Hessian for approaching solution to a point in state space (corresponding to 0th, 1th or 2th derivative) at final time.
  StackedTimeMatrix HessianTracking1DF_;

  //! Number of translational states (e.g. x,y,z).
  unsigned int numOfTranslationalStates_;

  //! Number of rotational states (e.g. roll, pitch, yaw)
  unsigned int numOfRotationalStates_;
};

} /* namespace motion_gen */
