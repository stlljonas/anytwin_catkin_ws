/*
 * ContactForceObjectiveHandler.hpp
 *
 *  Created on: March 03, 2017
 *      Author: Fabian Jenelten, Dario Bellicoso
 */

#pragma once

#include <zmp_optimizer/zmp_optimizer.hpp>

// motion generation utils
#include <motion_generation_utils/TaskHandlerBase.hpp>
#include <motion_generation_utils/conversions.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

// robot_utils
#include <robot_utils/physical_definitions.hpp>

// Initialize eigen matrices with NaN to catch uninitialized objects.
#define EIGEN_INITIALIZE_MATRICES_BY_NAN

namespace zmp {

class ZmpOptimizerObjectiveHandler : public motion_gen::TaskHandlerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using TimeVectorDouble = Eigen::Matrix<double, 1, 2 * zmp::splineCoeffs>;
  using TimeMatrixDouble = Eigen::Matrix<double, 2 * zmp::splineCoeffs, 2 * zmp::splineCoeffs>;
  using TimeMatrixTriple = Eigen::Matrix<double, 3 * zmp::splineCoeffs, 3 * zmp::splineCoeffs>;

  explicit ZmpOptimizerObjectiveHandler()
      : TaskHandlerBase(),
        wholeBodyMass_(0.0),
        torsoInertiaTensorInBaseFrame_(),
        gravityVectorInPlaneFrame_(0.0, 0.0, -robot_utils::physical_definitions::getAbsoluteGravityAcceleration()),
        gravityProjection_(0.0),
        planeNormalInPlaneFrame_(Eigen::Vector3d::UnitZ()),
        mu_(0.0),
        Fnmax_(0.0) {
    // gravity projected along plane normal
    gravityProjection_ = planeNormalInPlaneFrame_.dot(gravityVectorInPlaneFrame_);
  }

  ~ZmpOptimizerObjectiveHandler() override = default;

  bool initialize(double wholeBodyMass, const Eigen::Matrix3d& torsoInertiaTensorInBaseFrame) {
    assert(wholeBodyMass > 0.0);
    if (!TaskHandlerBase::initialize()) {
      return false;
    }
    wholeBodyMass_ = wholeBodyMass;
    torsoInertiaTensorInBaseFrame_ = torsoInertiaTensorInBaseFrame;
    return true;
  }

  bool loadParameters(const TiXmlHandle& handle) {
    if (!TaskHandlerBase::loadParameters(handle)) {
      return false;
    }

    TiXmlHandle ineqConstraintHandle = handle;
    if (!tinyxml_tools::getChildHandle(ineqConstraintHandle, handle, "InequalityConstraints")) {
      return false;
    }

    // Force model.
    TiXmlHandle forceModelHandle = handle;
    if (!tinyxml_tools::getChildHandle(forceModelHandle, ineqConstraintHandle, "ForceModel")) {
      return false;
    }
    if (!tinyxml_tools::loadParameter(mu_, forceModelHandle, "friction_coefficient")) {
      return false;
    }
    if (!tinyxml_tools::loadParameter(Fnmax_, forceModelHandle, "max_normal_force")) {
      return false;
    }

    return true;
  }

  //! Returns gravity acceleration in plane frame.
  const Eigen::Vector3d& getGravityVectorInPlaneFrame() const { return gravityVectorInPlaneFrame_; }

  //! Returns gravity acceleration in plane frame.
  double getGravityInPlaneFrame(zmp::CogDim dim) const {
    if (zmp::isTranslation(dim)) {
      return gravityVectorInPlaneFrame_(zmp::toIndex(dim));
    }
    return 0.0;
  }

  //! Set gravity vector and plane normal in plane frame
  void setGravityAndPlaneNormalInPlaneFrame(const Eigen::Vector3d& planeNormalInPlaneFrame,
                                            const Eigen::Vector3d& gravityVectorInPlaneFrame) {
    planeNormalInPlaneFrame_ = planeNormalInPlaneFrame;
    gravityVectorInPlaneFrame_ = gravityVectorInPlaneFrame;
    gravityProjection_ = planeNormalInPlaneFrame_.dot(gravityVectorInPlaneFrame_);
  }

  //! Get plane normal in plane frame.
  const Eigen::Vector3d& getPlaneNormalInPlaneFrame() const { return planeNormalInPlaneFrame_; }

  //! Zmp inequality constraint of the form c(a*)>=0.
  // Compute gradient and Hessian for the zmp inequality constraint c(a*) given the solution of the previous SQP iteration a*.
  // The function also evaluates the inequality constraints c(a*) at that previous solution.
  template <typename Derived>
  bool getTranslationalZmpInequalityConstraintObjective(Eigen::MatrixBase<Derived> const& gradient, double& evaluatedInequality,
                                                        const Eigen::Ref<const Eigen::VectorXd> splineCoefficientsAtIterationK,
                                                        const std::vector<double>& lineCoeffs,
                                                        const motion_generation::Position& pathPosition,
                                                        const motion_generation::LinearAcceleration& pathAccleration) const {
    // Gradient for constraint: c(a) <= 0

    // Get translational spline coefficients (spline coeffs of fixed dofs are set to zero).
    const auto startCoeffId = getCoeffStartIndexTranslation(activeSplineId_);
    Eigen::VectorXd splineCoeffsTranslationAtIterationK(coeffsTransl);
    splineCoeffsTranslationAtIterationK.setZero();
    auto offset = 0u;
    for (const auto& dim : optimizationDofs_) {
      const unsigned int startTranslCoeffId = toIndex(dim) * splineCoeffs;
      if (isTranslation(dim)) {
        splineCoeffsTranslationAtIterationK.segment<splineCoeffs>(startTranslCoeffId) =
            splineCoefficientsAtIterationK.segment<splineCoeffs>(startCoeffId + offset);
        offset += splineCoeffs;
      }
    }

    // The COM position and acceleration computed at the previous (k-th) iteration of the SQP.
    // Needed in this method to evaluate the gradient at iteration k.
    Eigen::Vector3d cogPositionAtIterationK = translTimeMatrices_[zmp::Derivative::Zero] * splineCoeffsTranslationAtIterationK;
    Eigen::Vector3d cogAcclerationAtIterationK = translTimeMatrices_[zmp::Derivative::Second] * splineCoeffsTranslationAtIterationK;

    // Add fixed dofs (we assume these dofs to be known).
    for (const auto& dim : optimizationTranslationalDofs) {
      if (!std_utils::containsEnum(optimizationDofs_, dim)) {
        const auto id = toIndex(dim);
        cogPositionAtIterationK(id) = pathPosition.toImplementation()(id);
        cogAcclerationAtIterationK(id) = pathAccleration.toImplementation()(id);
      }
    }

    // Compute helper variables.
    const Eigen::Vector3d dVec = Eigen::Vector3d(lineCoeffs[zmp::lineCoeffA], lineCoeffs[zmp::lineCoeffB], 0.0);
    const Eigen::Vector3d gravityMinusCogAccelerationInPlaneFrame = gravityVectorInPlaneFrame_ - cogAcclerationAtIterationK;
    const Eigen::Matrix3d gammaMatrixInPlaneFrame = kindr::getSkewMatrixFromVector((planeNormalInPlaneFrame_.cross(dVec)).eval());
    const Eigen::Vector3d giForceInPlaneFrame = wholeBodyMass_ * (gravityMinusCogAccelerationInPlaneFrame);
    const Eigen::Vector3d nablaConstraintPosition = gammaMatrixInPlaneFrame * giForceInPlaneFrame;
    const Eigen::Vector3d nablaConstraintAcceleration =
        wholeBodyMass_ * (gammaMatrixInPlaneFrame * cogPositionAtIterationK - lineCoeffs[zmp::lineCoeffC] * planeNormalInPlaneFrame_);

    // Compute the column format gradient. The minus sign is due to the optimizer solving for
    // lower bounded constraints, i.e., c(x)>=0. Here, the calculations are done w.r.t. c(x)<=0.
    const_cast<Eigen::MatrixBase<Derived>&>(gradient) =
        -translTimeMatrices_[zmp::Derivative::Zero].transpose() * nablaConstraintPosition -
        translTimeMatrices_[zmp::Derivative::Second].transpose() * nablaConstraintAcceleration;

    // Evaluate the non-linear inequality constraint at the solution of the previous SQP iteration.
    // The evaluated inequality c(x) is used as c(x) >= 0, hence the negative sign.
    evaluatedInequality = -(dVec.dot(planeNormalInPlaneFrame_.cross(cogPositionAtIterationK.cross(giForceInPlaneFrame))) +
                            lineCoeffs[zmp::lineCoeffC] * planeNormalInPlaneFrame_.dot(giForceInPlaneFrame));

    return true;
  }

  //! Zmp inequality constraint of the form c(a*)>=0.
  // Compute gradient and Hessian for the zmp inequality constraint c(a*) given the solution of the previous SQP iteration a*.
  // The function also evaluates the inequality constraints c(a*) at that previous solution.
  template <typename Derived>
  bool getTranslationalZmpInequalityConstraintObjective(Eigen::MatrixBase<Derived> const& gradient, Eigen::Ref<TimeMatrixTriple> hessian,
                                                        double& evaluatedInequality,
                                                        const Eigen::Ref<const Eigen::VectorXd> splineCoefficients0,
                                                        const std::vector<double>& lineCoeffs,
                                                        const motion_generation::Position& pathPosition,
                                                        const motion_generation::LinearAcceleration& pathAccleration) const {
    // Gradient for constraint: c(a) <= 0
    if (!getTranslationalZmpInequalityConstraintObjective(gradient, evaluatedInequality, splineCoefficients0, lineCoeffs, pathPosition,
                                                          pathAccleration)) {
      return false;
    }

    // Hessian of the constraints.
    const TimeMatrixTriple hessianIntermediate =
        translTimeMatrices_[zmp::Derivative::Second].transpose() *
        kindr::getSkewMatrixFromVector(static_cast<Eigen::Vector3d>(computeProjectedLineCoeffs(lineCoeffs))) *
        translTimeMatrices_[zmp::Derivative::Zero];

    TimeMatrixTriple lowerHessian = TimeMatrixTriple::Zero();
    lowerHessian.triangularView<Eigen::Lower>() = -(hessianIntermediate.transpose() + hessianIntermediate);
    hessian = lowerHessian.selfadjointView<Eigen::Lower>();

    return true;
  }

  //! Linear zmp inequality constraints (valid only for planar zmp model).
  template <typename Derived>
  bool getPlanerZmpInequalityConstraints(Eigen::MatrixBase<Derived> const& inequalityMat, double& inequalityVecMin, double cog_height,
                                         const std::vector<double>& lineCoeffs) const {
    /*
     * Note: Constraints are valid only for plane normal = [0 0 1]. Otherwise, the constraints would be nonlinear.
     * Use the 3d zmp model instead. Constraints are valid for any plane rotation.
     *
     * Constraints
     *  -a*x_zmp - b*y_zmp <= c
     * with
     *  x_zmp = x +ddot x*z/gz - gx*z/gz and y_zmp = y +ddot y*z/gz - gy*z/gz
     * yiels
     *  -a(x*gz + ddot x*z) - b(y*gz + ddot y*x) <= c*gz - z*(a*gx+b*gy)
     */
    assert(optimizationDofs_.size() == 2u);
    assert(robot_utils::areNear(planeNormalInPlaneFrame_.z(), 1.0));

    (const_cast<Eigen::MatrixBase<Derived>&>(inequalityMat)).template block<1, zmp::splineCoeffs>(0, 0) =
        -lineCoeffs[zmp::lineCoeffA] *
        (gravityVectorInPlaneFrame_.z() * timeVectors_[zmp::Derivative::Zero] + cog_height * timeVectors_[zmp::Derivative::Second]);

    (const_cast<Eigen::MatrixBase<Derived>&>(inequalityMat)).template block<1, zmp::splineCoeffs>(0, zmp::splineCoeffs) =
        -lineCoeffs[zmp::lineCoeffB] *
        (gravityVectorInPlaneFrame_.z() * timeVectors_[zmp::Derivative::Zero] + cog_height * timeVectors_[zmp::Derivative::Second]);

    inequalityVecMin = lineCoeffs[zmp::lineCoeffC] * gravityVectorInPlaneFrame_.z() -
                       cog_height * (lineCoeffs[zmp::lineCoeffA] * gravityVectorInPlaneFrame_.x() +
                                     lineCoeffs[zmp::lineCoeffB] * gravityVectorInPlaneFrame_.y());

    return true;
  }

  //! Zmp inequality constraint of the form c(a*)>=0.
  // Compute gradient and Hessian for the zmp inequality constraint c(a*) given the solution of the previous SQP iteration a*.
  // The function also evaluates the inequality constraints c(a*) at that previous solution.
  template <typename Derived>
  bool getRotationalTranslationalZmpInequalityConstraintObjective(
      Eigen::MatrixBase<Derived> const& gradient, double& evaluatedInequality, const Eigen::Ref<const Eigen::VectorXd> splineCoefficients0,
      const std::vector<double>& lineCoeffs, const motion_generation::Position& pathPosition,
      const motion_generation::LinearAcceleration& pathAccleration, const motion_generation::EulerAnglesZyx& eulerAnglesPathZyx,
      const motion_generation::EulerAnglesZyxDiff& eulerAnglesPathZyx_dot,
      const motion_generation::EulerAnglesZyxDiff& eulerAnglesPathZyx_ddot) const {
    /************************
     * Translational States *
     ************************/
    double evaluatedInequalityTransl;
    if (!getTranslationalZmpInequalityConstraintObjective(const_cast<Eigen::MatrixBase<Derived>&>(gradient).template head<coeffsTransl>(),
                                                          evaluatedInequalityTransl, splineCoefficients0, lineCoeffs, pathPosition,
                                                          pathAccleration)) {
      return false;
    }
    /************************/

    /*********************
     * Rotational States *
     *********************/
    // Get rotational spline coefficients (spline coeffs of fixed dofs are set to zero)
    const auto startCoeffId = getCoeffStartIndexRotation(activeSplineId_);
    Eigen::VectorXd splineCoeffsRot0(coeffsRot);
    splineCoeffsRot0.setZero();
    auto offset = 0u;
    for (const auto& dim : optimizationDofs_) {
      if (isRotation(dim)) {
        const unsigned int startRotCoeffId = toIndex(dim) * splineCoeffs;
        splineCoeffsRot0.segment<splineCoeffs>(startRotCoeffId) = splineCoefficients0.segment<splineCoeffs>(startCoeffId + offset);
        offset += splineCoeffs;
      }
    }

    // Extract rotational state.
    motion_generation::EulerAnglesZyx eulerAnglesZyx0(translTimeMatrices_[zmp::Derivative::Zero] * splineCoeffsRot0);
    motion_generation::EulerAnglesZyxDiff eulerAnglesZyx0_dot(translTimeMatrices_[zmp::Derivative::First] * splineCoeffsRot0);
    motion_generation::EulerAnglesZyxDiff eulerAnglesZyx0_ddot(translTimeMatrices_[zmp::Derivative::Second] * splineCoeffsRot0);

    // Add fixed dofs (we assume these dofs to be known).
    for (const auto& dim : optimizationRotationalDofs) {
      if (!std_utils::containsEnum(optimizationDofs_, dim)) {
        const auto id = toIndex(dim);
        eulerAnglesZyx0.toImplementation()(id) = eulerAnglesPathZyx.toImplementation()(id);
        eulerAnglesZyx0_dot.toImplementation()(id) = eulerAnglesPathZyx_dot.toImplementation()(id);
        eulerAnglesZyx0_ddot.toImplementation()(id) = eulerAnglesPathZyx_ddot.toImplementation()(id);
      }
    }

    // Rotation matrix base (body frame) to plane (inertial frame).
    const Eigen::Matrix3d rotationMatrixBaseToWorld = motion_generation::RotationMatrix(eulerAnglesZyx0).matrix();

    // Compute transformation matrices.
    const Eigen::Matrix3d mappingEulerZyxToOmegaInWorldFrame = zmp::getMapEulerAnglesZyxToAngularVelocityInInertialFrame(eulerAnglesZyx0);
    const Eigen::Matrix3d timeDerivativeMappingEulerZyxToOmegaInWorldFrame =
        zmp::getTimeDerivativeMapEulerAnglesZyxToAngularVelocityInInertialFrame(eulerAnglesZyx0, eulerAnglesZyx0_dot);

    // Transform angular velocity and acceleration to Euler rates and acceleration.
    const Eigen::Vector3d angularVelocityBaseInInertialFrame = mappingEulerZyxToOmegaInWorldFrame * eulerAnglesZyx0_dot.toImplementation();
    const Eigen::Vector3d angularAccelerationBaseInInertialFrame =
        mappingEulerZyxToOmegaInWorldFrame * eulerAnglesZyx0_ddot.toImplementation() +
        timeDerivativeMappingEulerZyxToOmegaInWorldFrame * eulerAnglesZyx0_dot.toImplementation();

    // Angular Momentum in inertial frame.
    const Eigen::Matrix3d inertiaTensorBaseInInertialFrame =
        rotationMatrixBaseToWorld * torsoInertiaTensorInBaseFrame_ * rotationMatrixBaseToWorld.transpose();
    const Eigen::Vector3d angularMomentumInInertialFrame = inertiaTensorBaseInInertialFrame * angularVelocityBaseInInertialFrame;

    // Time derivative of Angular Momentum in inertial frame.
    const Eigen::Vector3d angularMomemtumDotInInertialFrame = inertiaTensorBaseInInertialFrame * angularAccelerationBaseInInertialFrame +
                                                              angularVelocityBaseInInertialFrame.cross(angularMomentumInInertialFrame);

    // Derivative of rate of change of angular momentum w.r.t Euler angles.
    const Eigen::Matrix3d partial_ldot_eul =
        zmp::getDerivativeLdotWrtEulerAngles(eulerAnglesZyx0, eulerAnglesZyx0_dot, eulerAnglesZyx0_ddot, torsoInertiaTensorInBaseFrame_);

    // Derivative of rate of change of angular momentum w.r.t Euler rates.
    const Eigen::Matrix3d partial_ldot_w =
        kindr::getSkewMatrixFromVector(angularVelocityBaseInInertialFrame) * inertiaTensorBaseInInertialFrame -
        kindr::getSkewMatrixFromVector(angularMomentumInInertialFrame);
    const Eigen::Matrix3d partial_w_eulDot = mappingEulerZyxToOmegaInWorldFrame;
    const Eigen::Matrix3d partial_ldot_wdot = inertiaTensorBaseInInertialFrame;
    const Eigen::Matrix3d partial_wdot_eulDot = zmp::getDerivativeAngularAccelerationWrtEulerRates(eulerAnglesZyx0, eulerAnglesZyx0_dot);
    const Eigen::Matrix3d partial_ldot_eulDot = partial_ldot_w * partial_w_eulDot + partial_ldot_wdot * partial_wdot_eulDot;

    // Derivative of rate of change of angular momentum w.r.t Euler acceleration.
    const Eigen::Matrix3d partial_ldot_eulDDot = inertiaTensorBaseInInertialFrame * mappingEulerZyxToOmegaInWorldFrame;

    // Compute constant helper vectors: projectedLineCoeffs = d^T * S(n) = (d x n)^T.
    const auto projectedLineCoeffs = computeProjectedLineCoeffs(lineCoeffs);

    // Compute and set gradient.
    const Eigen::VectorXd rotational_gradient =
        (projectedLineCoeffs * (partial_ldot_eul * translTimeMatrices_[zmp::Derivative::Zero] +
                                partial_ldot_eulDot * translTimeMatrices_[zmp::Derivative::First] +
                                partial_ldot_eulDDot * translTimeMatrices_[zmp::Derivative::Second]))
            .transpose();
    const_cast<Eigen::MatrixBase<Derived>&>(gradient).template tail<coeffsRot>() = rotational_gradient;

    // Evaluate inequality.
    evaluatedInequality = evaluatedInequalityTransl + projectedLineCoeffs * angularMomemtumDotInInertialFrame;
    /*********************/

    return true;
  }

  //! Ensure that contact pushes on the ground.
  template <typename Derived>
  bool getPushContactConstraints(Eigen::MatrixBase<Derived> const& inequalityMat, double& inequalityVecMin) const {
    assert(optimizationDofs_.size() >= 3u);
    const_cast<Eigen::MatrixBase<Derived>&>(inequalityMat) =
        planeNormalInPlaneFrame_.transpose() * translTimeMatrices_[zmp::Derivative::Second];
    inequalityVecMin = gravityProjection_ + 1e-7;
    return true;
  }

  //! Objective for approaching the final touch-down state to a certain point in state space.
  bool getFinalFlightPhaseStateObjective(Eigen::Ref<TimeMatrix> hessian, Eigen::Ref<TimeVectorTranspose> linearTerm, double weight,
                                         double flightDuration, zmp::Derivative derivative, zmp::CogDim dim,
                                         double desiredTouchDownPosition, double desiredTouchDownVelocity) const {
    assert(weight >= 0.0);

    TimeVector equalityMat;
    double equalityVec;
    if (!getFinalFlightPhaseStateEqualityConstraints(equalityMat, equalityVec, flightDuration, derivative, dim, desiredTouchDownPosition,
                                                     desiredTouchDownVelocity)) {
      return false;
    }

    hessian.triangularView<Eigen::Lower>() += (equalityMat.transpose() * equalityMat) * weight;
    linearTerm += -equalityVec * equalityMat.transpose() * weight;

    return true;
  }

  //! Objective for approaching the initial touch-down state to a certain point in state space.
  bool getInitialFlightPhaseStateObjective(Eigen::Ref<TimeMatrix> hessian, Eigen::Ref<TimeVectorTranspose> linearTerm, double weight,
                                           double flightDuration, zmp::Derivative derivative, zmp::CogDim dim,
                                           double measuredFlightPosition, double measuredFlightVelocity) const {
    assert(weight >= 0.0);

    double predictedTouchDownState;
    if (!predictTouchDownState(predictedTouchDownState, flightDuration, derivative, dim, measuredFlightPosition, measuredFlightVelocity)) {
      return false;
    }

    if (!getInitialStateObjective(hessian, linearTerm, predictedTouchDownState, derivative, weight)) {
      return false;
    }

    return true;
  }

  //! This constraint enforces the final CoG state to lay within a box (-boundMin, +boundMax) centered at center,
  // if the current phase corresponds to a full flight phase. Both boundaries must be larger than zero.
  // By default, boundMax = boundMin.
  template <typename Derived1, typename Derived2>
  bool getFinalFlightPhaseMaxStateInequalityConstraints(Eigen::MatrixBase<Derived1> const& inequalityMat,
                                                        Eigen::MatrixBase<Derived2> const& inequalityVecMin, double center,
                                                        double flightDuration, zmp::Derivative derivative, zmp::CogDim dim,
                                                        double /*desiredTouchDownPosition*/, double /*desiredTouchDownVelocity*/,
                                                        double boundMin, double boundMax = -1.0) const {
    /*
     * Ballistic System:
     *     ddot r(t) = g
     *      dot r(t) = g*t + dot r(0)
     *          r(t) = 0.5*g*t^2 + dot r(0)*tf + r(0)
     * where r(0) defines the state at lift-off and r(t) at touch-down. The variable t is the
     * flight duration. Substituting spline coefficients results in:
     *     ddot r(t) = g
     *      dot r(t) = g*t + dot T(t)*a
     *          r(t) = 0.5*g*t^2 + (dot T(t)*tf + T(t) )*a
     * r(t) is the center.
     */

    assert(flightDuration >= 0.0);

    TimeVector timeVec;
    if (derivative == zmp::Derivative::Zero) {
      timeVec = timeVectorsF_[zmp::Derivative::First] * flightDuration + timeVectorsF_[zmp::Derivative::Zero];
    } else if (derivative == zmp::Derivative::First) {
      timeVec = timeVectorsF_[zmp::Derivative::First];
    } else if (derivative == zmp::Derivative::Second) {
      timeVec.setZero();
    } else {
      return false;
    }

    (const_cast<Eigen::MatrixBase<Derived1>&>(inequalityMat)).template block<1, zmp::splineCoeffs>(0, 0) = timeVec;
    (const_cast<Eigen::MatrixBase<Derived1>&>(inequalityMat)).template block<1, zmp::splineCoeffs>(1, 0) = -timeVec;

    if (boundMin < 0.0) {
      return false;
    }
    if (boundMax < 0.0) {
      boundMax = boundMin;
    }

    if (zmp::isTranslation(dim)) {
      if (derivative == zmp::Derivative::Zero) {
        center -= 0.5 * getGravityInPlaneFrame(dim) * boost::math::pow<2>(flightDuration);
      } else if (derivative == zmp::Derivative::First) {
        center -= getGravityInPlaneFrame(dim) * flightDuration;
      } else if (derivative == zmp::Derivative::Second) {
        center -= getGravityInPlaneFrame(dim);
      }
    }

    // min -> T*a >= center-bound, max -> T*a <= center+bound
    (const_cast<Eigen::MatrixBase<Derived2>&>(inequalityVecMin))(0) = center - boundMin;
    (const_cast<Eigen::MatrixBase<Derived2>&>(inequalityVecMin))(1) = -center - boundMax;

    return true;
  }

  //! Set CoG initial conditions for flight phase.
  template <typename Derived>
  bool getInitialFlightPhaseStateEqualityConstraints(Eigen::MatrixBase<Derived> const& equalityMat, double& equalityVec,
                                                     double predictedDurationLeftInFlight, zmp::CogDim dim, zmp::Derivative derivative,
                                                     double measuredFlightPosition, double measuredFlightVelocity) const {
    assert(predictedDurationLeftInFlight >= 0.0);

    const_cast<Eigen::MatrixBase<Derived>&>(equalityMat) = timeVectors0_[derivative];
    return predictTouchDownState(equalityVec, predictedDurationLeftInFlight, derivative, dim, measuredFlightPosition,
                                 measuredFlightVelocity);
  }

  //! Set CoG final conditions for flight phase.
  template <typename Derived>
  bool getFinalFlightPhaseStateEqualityConstraints(Eigen::MatrixBase<Derived> const& equalityMat, double& equalityVec,
                                                   double flightDuration, zmp::Derivative derivative, zmp::CogDim dim,
                                                   double desiredTouchDownPosition, double desiredTouchDownVelocity) const {
    assert(flightDuration >= 0.0);
    /*
     * Ballistic System:
     *     ddot r(t) = g
     *      dot r(t) = g*t + dot r(0)
     *          r(t) = 0.5*g*t^2 + dot r(0)*t + r(0)
     * where r(0) defines the state at lift-off and r(t) at touch-down. The variable t is the
     * flight duration. Substituting spline coefficients results in:
     *     ddot r(t) = g
     *      dot r(t) = g*t + dot T(t)*a
     *          r(t) = 0.5*g*t^2 + (dot T(t)*t + T(t) )*a
     */
    if (derivative == zmp::Derivative::Zero) {
      const_cast<Eigen::MatrixBase<Derived>&>(equalityMat) =
          timeVectorsF_[zmp::Derivative::First] * flightDuration + timeVectorsF_[zmp::Derivative::Zero];
      equalityVec = desiredTouchDownPosition - 0.5 * getGravityInPlaneFrame(dim) * boost::math::pow<2>(flightDuration);
    } else if (derivative == zmp::Derivative::First) {
      const_cast<Eigen::MatrixBase<Derived>&>(equalityMat) = timeVectorsF_[zmp::Derivative::First];
      equalityVec = desiredTouchDownVelocity - getGravityInPlaneFrame(dim) * flightDuration;
    } else if (derivative == zmp::Derivative::Second) {
      const_cast<Eigen::MatrixBase<Derived>&>(equalityMat) = TimeVector::Zero();
      equalityVec = getGravityInPlaneFrame(dim);  // lift-off constraint
    } else {
      return false;
    }

    return true;
  }

  //! Constraint that interconnects two splines that are separated by a flight phase.
  template <typename Derived>
  bool getJunctionConstraintsFlightPhase(Eigen::MatrixBase<Derived> const& equalityMatCurrentSpline,
                                         Eigen::MatrixBase<Derived> const& equalityMatNextSpline, double& equalityVec,
                                         double flightDuration, zmp::Derivative derivative, zmp::CogDim dim) const {
    assert(flightDuration >= 0.0);
    /*
     * Ballistic System:
     *     ddot r(t) = g
     *      dot r(t) = g*t + dot r(0)
     *          r(t) = 0.5*g*t^2 + dot r(0)*t + r(0)
     * where r(0) defines the state at lift-off and r(t) at touch-down. The variable t is the
     * flight duration. Substituting spline coefficients results in:
     *     ddot T(0)a{i+1} = g
     *      dot T(0)a{i+1} = g*t + dot T(t)*a{i}
     *          T(0)a{i+1} = 0.5*g*t^2 + (dot T(t)*t + T(t) )*a{i}
     */
    const_cast<Eigen::MatrixBase<Derived>&>(equalityMatNextSpline) = timeVectors0_[derivative];

    if (derivative == zmp::Derivative::Zero) {
      const_cast<Eigen::MatrixBase<Derived>&>(equalityMatCurrentSpline) =
          -timeVectorsF_[zmp::Derivative::First] * flightDuration - timeVectorsF_[zmp::Derivative::Zero];
      equalityVec = 0.5 * getGravityInPlaneFrame(dim) * boost::math::pow<2>(flightDuration);
    } else if (derivative == zmp::Derivative::First) {
      const_cast<Eigen::MatrixBase<Derived>&>(equalityMatCurrentSpline) = -timeVectorsF_[zmp::Derivative::First];
      equalityVec = getGravityInPlaneFrame(dim) * flightDuration;
    } else if (derivative == zmp::Derivative::Second) {
      const_cast<Eigen::MatrixBase<Derived>&>(equalityMatCurrentSpline) = TimeVector::Zero();
      equalityVec = getGravityInPlaneFrame(dim);
    } else {
      return false;
    }

    return true;
  }

  //! Integrate ballistic dynamics until touch down to obtain touch-down state.
  bool predictTouchDownState(double& predictedTouchDownState, double predictedDurationLeftInFlight, zmp::Derivative derivative,
                             zmp::CogDim dim, double measuredFlightPosition, double measuredFlightVelocity) const {
    /*
     * Ballistic System:
     *     ddot r(t) = g
     *      dot r(t) = g*t + dot r(0)
     *          r(t) = 0.5*g*t^2 + dot r(0)*t + r(0)
     */

    assert(predictedDurationLeftInFlight >= 0.0);

    if (derivative == zmp::Derivative::Zero) {
      predictedTouchDownState = measuredFlightVelocity * predictedDurationLeftInFlight + measuredFlightPosition +
                                0.5 * getGravityInPlaneFrame(dim) * boost::math::pow<2>(predictedDurationLeftInFlight);
    } else if (derivative == zmp::Derivative::First) {
      predictedTouchDownState = measuredFlightVelocity + getGravityInPlaneFrame(dim) * predictedDurationLeftInFlight;
    } else if (derivative == zmp::Derivative::Second) {
      predictedTouchDownState = getGravityInPlaneFrame(dim);
    } else {
      return false;
    }

    return true;
  }

  //! Constrain CoG state s.t. contact wrench lays within a friction cone.
  template <typename Derived1, typename Derived2>
  bool getFrictionPyramideConstraints(Eigen::MatrixBase<Derived1> const& inEqualityMat, Eigen::MatrixBase<Derived2> const& inequalityVecMin,
                                      const Eigen::Ref<const Eigen::Vector3d> averagedContactNormal) const {
    assert(optimizationDofs_.size() > 2u);
    assert(averagedContactNormal.z() != 0.0);

    // We need two tangential directions.
    Eigen::Vector3d tangentHeading(1.0, 0.0, -averagedContactNormal.x() / averagedContactNormal.z());
    tangentHeading.normalize();

    Eigen::Vector3d tangentLateral(0.0, 1.0, -averagedContactNormal.y() / averagedContactNormal.z());
    tangentLateral.normalize();

    Eigen::Matrix<double, 4, dofTransl> U;
    U.block<1, dofTransl>(0, 0) = (tangentHeading - mu_ * averagedContactNormal).transpose();
    U.block<1, dofTransl>(1, 0) = (-tangentHeading - mu_ * averagedContactNormal).transpose();
    U.block<1, dofTransl>(2, 0) = (tangentLateral - mu_ * averagedContactNormal).transpose();
    U.block<1, dofTransl>(3, 0) = (-tangentLateral - mu_ * averagedContactNormal).transpose();

    const_cast<Eigen::MatrixBase<Derived1>&>(inEqualityMat) = -U * translTimeMatrices_[zmp::Derivative::Second];
    const_cast<Eigen::MatrixBase<Derived2>&>(inequalityVecMin) = -U * gravityVectorInPlaneFrame_;

    return true;
  }

  //! Constrain CoG state s.t. contact wrench is upper bounded.
  template <typename Derived>
  bool getMaxNormalContactForceConstraints(Eigen::MatrixBase<Derived> const& inEqualityMat, double& inequalityVecMin,
                                           const Eigen::Ref<const Eigen::Vector3d> averagedContactNormal,
                                           unsigned int numOfGroundedLegs) const {
    assert(numOfGroundedLegs >= 0);

    const_cast<Eigen::MatrixBase<Derived>&>(inEqualityMat) =
        -averagedContactNormal.transpose() * translTimeMatrices_[zmp::Derivative::Second];
    inequalityVecMin = -averagedContactNormal.dot(gravityVectorInPlaneFrame_) - numOfGroundedLegs / wholeBodyMass_ * Fnmax_;

    return true;
  }

 protected:
  // Compute constant helper vectors: projectedLineCoeffs = d^T * S(n) = (d x n)^T.
  Eigen::Matrix<double, 1, dofTransl> computeProjectedLineCoeffs(const std::vector<double>& lineCoeffs) const {
    return (Eigen::Vector3d(lineCoeffs[zmp::lineCoeffA], lineCoeffs[zmp::lineCoeffB], 0.0).cross(planeNormalInPlaneFrame_).transpose());
  }

  //! Point mass of the robot.
  double wholeBodyMass_;

  //! Inertia of the torso given in plane frame.
  Eigen::Matrix3d torsoInertiaTensorInBaseFrame_;

  //! Gravity Vector given in vitual plane frame.
  Eigen::Vector3d gravityVectorInPlaneFrame_;

  //! Gravity vector projected along plane normal.
  double gravityProjection_;

  //! Normal of virtual plane given in virtual plane frame.
  Eigen::Vector3d planeNormalInPlaneFrame_;

  //! Friction coefficient.
  double mu_;

  //! Max normal contact force.
  double Fnmax_;
};

} /* namespace zmp */
