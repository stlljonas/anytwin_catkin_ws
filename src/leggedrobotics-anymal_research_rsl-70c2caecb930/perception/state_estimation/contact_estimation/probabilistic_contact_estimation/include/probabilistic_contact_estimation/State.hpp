/*
 * State.hpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// probabilistic contact estimation
#include <probabilistic_contact_estimation/probabilistic_contact_estimation.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>

// basic filters
#include <basic_filters/filters.hpp>

// kindr.
#include <kindr/Core>

// tinyxml.
#include <tinyxml.h>

//std.
#include <string>

class TiXmlHandle;

namespace contact_estimation {

class State {
 public:
  using StackedSupportJacobian = Eigen::Matrix<double, AD::getNumTranslationalDof() * AD::getNumLimbs(), AD::getNumDof()>;

  public:
    explicit State(anymal_model::AnymalModel* model);
    virtual ~State() = default;

    virtual bool loadParameters(const TiXmlHandle& handle);
    virtual bool advance(double dt, const std_utils::EnumArray<AD::ContactEnum, ContactState>& contactState);
    virtual bool initialize(const double dt);

    //! Get state.
    const Eigen::MatrixXd& getMassMatrix() const noexcept;
    const Eigen::Matrix3d& getLegMassMatrixInverted(AD::ContactEnum contactEnum) const noexcept;
    const Eigen::MatrixXd& getSelectionMatrixTransposed() const noexcept;
    const Eigen::VectorXd& getNonlinearityAndTorque() const noexcept;

    //! Get raw measurements.
    const Eigen::Vector3d& getPositionWorldToEndEffectorInWorldFrame(AD::ContactEnum contactEnum) const noexcept;
    const anymal_model::GeneralizedVelocities& getGeneralizedVelocities() const noexcept;
    const anymal_model::GeneralizedAccelerations& getGeneralizedAccelerations() const noexcept;
    const anymal_model::JointTorques& getJointTorques() const noexcept;
    const Eigen::Vector3d& getLinearVelocityEndEffectorInWorldFrame(AD::ContactEnum contactEnum) const noexcept;
    const Eigen::Vector3d& getContactForceInWorldFrame(AD::ContactEnum contactEnum) const noexcept;

    //! Get average height.
    double getExpectedGroundHeightInWorldFrame(AD::ContactEnum contactEnum) const noexcept;

    //! Get plane orientation.
    const kindr::EulerAnglesZyxPD& getOrientationWorldToPlane() const noexcept;

    //! Get Jacobian.
    const Eigen::MatrixXd& getJacobianTranslationWorldToEndEffectorInWorldFrame(AD::ContactEnum contactEnum) const noexcept;

    //! Returns the contact state of a limb.
    const ContactState& getContactState(AD::ContactEnum contactEnum) const;

  protected:
    //! Update terrain orientation (control frame).
    bool updatePlaneEstimation();

    //! Predicts touch-down location using local terrain plane approximation.
    virtual bool updateExpectedTouchDownPosition(const std_utils::EnumArray<AD::ContactEnum, ContactState>& contactState);

    //! Get height above terrain along gravity vector. The terrain is approximated as a plane.
    virtual double getHeight(const Eigen::Vector3d& positionWorldToLocationInWorldFrame) const;

    //! Anymal model.
    anymal_model::AnymalModel* model_;

    //! Number of dofs for limb.
    std_utils::EnumArray<AD::ContactEnum, unsigned int> numDofLimb_;

    //! Number of legs (not limbs).
    const unsigned int numLegs_;

    //! if true, derivative of base velocity is computed.
    bool computeBaseAcceleration_;

    //! If true, contact forces are computed as combined system, otherwise for each leg individually.
    bool useWholeBodyContactForces_;

    //! State.
    Eigen::MatrixXd massMatrix_;
    Eigen::VectorXd nonlinearEffects_;
    std_utils::EnumArray<AD::ContactEnum, Eigen::Matrix3d> legMassMatrixInverted_;
    Eigen::MatrixXd selectionMatrixTransposed_;
    Eigen::VectorXd nonlinearityAndTorque_;
    std_utils::EnumArray<AD::ContactEnum, ContactState> previousContactState_;

    //! Raw measurement.
    std_utils::EnumArray<AD::ContactEnum, Eigen::Vector3d> positionWorldToEndEffectorInWorldFrameVector_;
    std_utils::EnumArray<AD::ContactEnum, Eigen::Vector3d> positionWorldToEndEffectorTouchDownInWorldFrameVector_;
    anymal_model::GeneralizedVelocities generalizedVelocities_;
    anymal_model::GeneralizedAccelerations generalizedAccelerations_;
    anymal_model::JointTorques jointTorques_;
    std_utils::EnumArray<AD::ContactEnum, Eigen::Vector3d> linearVelocityEndEffectorInWorldFrameVector_;
    std_utils::EnumArray<AD::ContactEnum, Eigen::Vector3d> contactForceInWorldFrameVector_;

    //! Average height.
    std_utils::EnumArray<AD::ContactEnum, basic_filters::FirstOrderFilter<double>> expectedGroundHeightInWorldFrame_;
    double groundHeightFilterConstant_;

    //! Terrain orientation.
    kindr::EulerAnglesZyxPD orientationWorldToPlane_;
    basic_filters::FirstOrderFilter<Eigen::Vector3d> filteredTerrainNormal_;
    basic_filters::FirstOrderFilter<Eigen::Vector3d> filteredPositionOnPlane_;
    double terrainFilterConstant_;

    //! Jacobians.
    std_utils::EnumArray<AD::ContactEnum, Eigen::MatrixXd> jacobianTranslationWorldToEndEffectorInWorldFrameVector_;
    std_utils::EnumArray<AD::ContactEnum, Eigen::MatrixXd> jacobianTranslationTimeDerivativeWorldToEndEffectorInWorldFrameVector_;
};

} /* namespace contact_estimation */
