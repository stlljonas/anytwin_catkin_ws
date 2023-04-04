#include "basic_controllers/PidControllerQuaternion.hpp"

namespace basic_controllers {

PidControllerQuaternion::PidControllerQuaternion()
    : PidControllerQuaternion(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) {}

PidControllerQuaternion::PidControllerQuaternion(const Eigen::Vector3d maxEffort, const Eigen::Vector3d kp, const Eigen::Vector3d ki,
                                                 const Eigen::Vector3d kd)
    : integral_(Eigen::Vector3d::Zero()),
      qBiPrev_(),
      kp_("Kp", kp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
      ki_("Ki", ki, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
      kd_("Kd", kd, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
      maxEffort_("PidOrientation/MaxEffort", maxEffort, Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Constant(std::numeric_limits<double>::max())) {}

PidControllerQuaternion::~PidControllerQuaternion() = default;

void PidControllerQuaternion::reset() {
  integral_.setZero();
  qBiPrev_.setIdentity();
}

Eigen::Vector3d PidControllerQuaternion::update(const double dt, const kindr::RotationQuaternionPD q_BI_des,
                                                const kindr::RotationQuaternionPD q_BI_meas) {
  const Eigen::Vector3d B_w_IB_meas = -q_BI_meas.boxMinus(qBiPrev_) / dt;
  return update(dt, q_BI_des, q_BI_meas, kindr::LocalAngularVelocityPD(), B_w_IB_meas);
}

Eigen::Vector3d PidControllerQuaternion::update(const double dt, const kindr::RotationQuaternionPD q_BI_des,
                                                const kindr::RotationQuaternionPD q_BI_meas,
                                                const kindr::LocalAngularVelocityPD B_w_IB_des) {
  const Eigen::Vector3d B_w_IB_meas = -q_BI_meas.boxMinus(qBiPrev_) / dt;
  return update(dt, q_BI_des, q_BI_meas, B_w_IB_des, B_w_IB_meas);
}

Eigen::Vector3d PidControllerQuaternion::update(const double dt, const kindr::RotationQuaternionPD q_BI_des,
                                                const kindr::RotationQuaternionPD q_BI_meas, const kindr::LocalAngularVelocityPD B_w_IB_des,
                                                const kindr::LocalAngularVelocityPD B_w_IB_meas) {
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  const Eigen::Vector3d derivative = B_w_IB_des.toImplementation() - B_w_IB_meas.toImplementation();
  const Eigen::Vector3d errorInBFrame = -q_BI_des.boxMinus(q_BI_meas);

  integral_ += errorInBFrame * dt;
  integral_ = integral_.cwiseMin(maxEffort_.getValue()).cwiseMax(-maxEffort_.getValue());

  out = kp_.getValue().cwiseProduct(errorInBFrame) + ki_.getValue().cwiseProduct(integral_) + kd_.getValue().cwiseProduct(derivative);
  out = out.cwiseMin(maxEffort_.getValue()).cwiseMax(-maxEffort_.getValue());

  return out;
}

bool PidControllerQuaternion::addParametersToHandler(const std::string& pidName) {
  parameter_handler::handler->addParam(pidName + std::string("/PidOrientation/MaxEffort"), maxEffort_);
  parameter_handler::handler->addParam(pidName + std::string("/PidOrientation/Kp"), kp_);
  parameter_handler::handler->addParam(pidName + std::string("/PidOrientation/Ki"), ki_);
  parameter_handler::handler->addParam(pidName + std::string("/PidOrientation/Kd"), kd_);
  return true;
}

}  // namespace basic_controllers
