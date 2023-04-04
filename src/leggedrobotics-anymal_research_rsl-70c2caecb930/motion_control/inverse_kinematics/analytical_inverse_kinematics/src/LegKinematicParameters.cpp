/**
 * @authors     Dario Bellicoso, Francisco Giraldez Gamez
 * @affiliation ETH Zurich, ANYbotics
 * @brief       Class storing the necessary parameters and trigonometric coefficients for analytical IK of a point-foot 3DOF leg.
 */

#include "analytical_inverse_kinematics/LegKinematicParameters.hpp"

namespace analytical_inverse_kinematics {

const Position& LegKinematicParameters::getPositionBaseToHipInBaseFrame() const {
  return positionBaseToHipInBaseFrame_;
}

const Position& LegKinematicParameters::getPositionHipToThighInHipFrame() const {
  return positionHipToThighInHipFrame_;
}

const Position& LegKinematicParameters::getPositionThighToShankInThighFrame() const {
  return positionThighToShankInThighFrame_;
}

const Position& LegKinematicParameters::getPositionShankToFootInShankFrame() const {
  return positionShankToFootInShankFrame_;
}

const Position& LegKinematicParameters::getDefaultPositionHipToFootInBaseFrame() const {
  return defaultPositionHipToFootInBaseFrame_;
}

const EulerAnglesZyx& LegKinematicParameters::getOrientationHipToBase() const {
  return orientationHipToBase_;
}

const EulerAnglesZyx& LegKinematicParameters::getOrientationThighToHip() const {
  return orientationThighToHip_;
}

const EulerAnglesZyx& LegKinematicParameters::getOrientationShankToThigh() const {
  return orientationShankToThigh_;
}

const EulerAnglesZyx& LegKinematicParameters::getOrientationFootToShank() const {
  return orientationFootToShank_;
}

void LegKinematicParameters::setPositionBaseToHipInBaseFrame(const Position& positionBaseToHipInBaseFrame) {
  positionBaseToHipInBaseFrame_ = positionBaseToHipInBaseFrame;
}

void LegKinematicParameters::setPositionHipToThighInHipFrame(const Position& positionHipToThighInHipFrame) {
  positionHipToThighInHipFrame_ = positionHipToThighInHipFrame;
}

void LegKinematicParameters::setPositionThighToShankInThighFrame(const Position& positionThighToShankInThighFrame) {
  positionThighToShankInThighFrame_ = positionThighToShankInThighFrame;
}

void LegKinematicParameters::setPositionShankToFootInShankFrame(const Position& positionShankToFootInShankFrame) {
  positionShankToFootInShankFrame_ = positionShankToFootInShankFrame;
}

void LegKinematicParameters::setOrientationHipToBase(const EulerAnglesZyx& orientationHipToBase) {
  orientationHipToBase_ = orientationHipToBase;
}

void LegKinematicParameters::setOrientationThighToHip(const EulerAnglesZyx& orientationThighToHip) {
  orientationThighToHip_ = orientationThighToHip;
}

void LegKinematicParameters::setOrientationShankToThigh(const EulerAnglesZyx& orientationShankToThigh) {
  orientationShankToThigh_ = orientationShankToThigh;
}

void LegKinematicParameters::setOrientationFootToShank(const EulerAnglesZyx& orientationFootToShank) {
  orientationFootToShank_ = orientationFootToShank;
}

void LegKinematicParameters::initialize() {
  computeTrigonometricEquationCoefficientsKfe();

  defaultPositionHipToFootInBaseFrame_ = positionHipToThighInHipFrame_ + orientationThighToHip_.rotate(positionThighToShankInThighFrame_) +
                                         (orientationThighToHip_ * orientationShankToThigh_).rotate(positionShankToFootInShankFrame_);
}

void LegKinematicParameters::printParameters() {
  std::cout << "B_r_BH: " << positionBaseToHipInBaseFrame_ << std::endl;
  std::cout << "H_r_HT: " << positionHipToThighInHipFrame_ << std::endl;
  std::cout << "T_r_TS: " << positionThighToShankInThighFrame_ << std::endl;
  std::cout << "S_r_SF: " << positionShankToFootInShankFrame_ << std::endl;

  std::cout << "C_BH: " << EulerAnglesZyx(orientationHipToBase_) << std::endl;
  std::cout << "C_HT: " << EulerAnglesZyx(orientationThighToHip_) << std::endl;
  std::cout << "C_TS: " << EulerAnglesZyx(orientationShankToThigh_) << std::endl;
  std::cout << "C_SF: " << EulerAnglesZyx(orientationFootToShank_) << std::endl;

  std::cout << "H_r_HF: " << defaultPositionHipToFootInBaseFrame_ << std::endl;

  std::cout << "aKFE: " << aKFE_ << std::endl;
  std::cout << "bKFE: " << bKFE_ << std::endl;
  std::cout << "cKFE: " << cKFE_ << std::endl;
}

double LegKinematicParameters::getTrigonometricEquationCoefficientHfeA(double qKFE) const {
  const Position& H_r_HT = positionHipToThighInHipFrame_;
  const Position& T_r_TS = positionThighToShankInThighFrame_;
  const Position& S_r_SF = positionShankToFootInShankFrame_;

  const double H_r_HT_x = H_r_HT.x();
  const double H_r_HT_z = H_r_HT.z();
  const double S_r_SF_x = S_r_SF.x();
  const double S_r_SF_z = S_r_SF.z();
  const double T_r_TS_x = T_r_TS.x();
  const double T_r_TS_z = T_r_TS.z();
  const double t2 = std::cos(qKFE);
  const double t3 = std::sin(qKFE);

  return H_r_HT_x * T_r_TS_z * 2.0 - H_r_HT_z * T_r_TS_x * 2.0 - H_r_HT_x * S_r_SF_x * t3 * 2.0 + H_r_HT_x * S_r_SF_z * t2 * 2.0 -
         H_r_HT_z * S_r_SF_x * t2 * 2.0 - H_r_HT_z * S_r_SF_z * t3 * 2.0;
}

double LegKinematicParameters::getTrigonometricEquationCoefficientHfeB(double qKFE) const {
  const double H_r_HT_x = positionHipToThighInHipFrame_.x();
  const double H_r_HT_z = positionHipToThighInHipFrame_.z();
  const double S_r_SF_x = positionShankToFootInShankFrame_.x();
  const double S_r_SF_z = positionShankToFootInShankFrame_.z();
  const double T_r_TS_x = positionThighToShankInThighFrame_.x();
  const double T_r_TS_z = positionThighToShankInThighFrame_.z();
  const double t2 = std::cos(qKFE);
  const double t3 = std::sin(qKFE);

  return (H_r_HT_x * T_r_TS_x * 2.0 + H_r_HT_z * T_r_TS_z * 2.0 + H_r_HT_x * S_r_SF_x * t2 * 2.0 + H_r_HT_x * S_r_SF_z * t3 * 2.0 -
          H_r_HT_z * S_r_SF_x * t3 * 2.0 + H_r_HT_z * S_r_SF_z * t2 * 2.0);
}

double LegKinematicParameters::getTrigonometricEquationCoefficientHfeC(double qKFE) const {
  const Position& H_r_HT = positionHipToThighInHipFrame_;
  const Position& T_r_TS = positionThighToShankInThighFrame_;
  const Position& S_r_SF = positionShankToFootInShankFrame_;

  const double H_r_HT_x = H_r_HT.x();
  const double H_r_HT_y = H_r_HT.y();
  const double H_r_HT_z = H_r_HT.z();
  const double S_r_SF_x = S_r_SF.x();
  const double S_r_SF_y = S_r_SF.y();
  const double S_r_SF_z = S_r_SF.z();
  const double T_r_TS_x = T_r_TS.x();
  const double T_r_TS_y = T_r_TS.y();
  const double T_r_TS_z = T_r_TS.z();
  const double t2 = std::cos(qKFE);
  const double t3 = std::sin(qKFE);

  return S_r_SF_y * T_r_TS_y * 2.0 + H_r_HT_x * H_r_HT_x + H_r_HT_y * H_r_HT_y + H_r_HT_z * H_r_HT_z + S_r_SF_x * S_r_SF_x +
         S_r_SF_y * S_r_SF_y + S_r_SF_z * S_r_SF_z + T_r_TS_x * T_r_TS_x + T_r_TS_y * T_r_TS_y + T_r_TS_z * T_r_TS_z +
         H_r_HT_y * S_r_SF_y * 2.0 + H_r_HT_y * T_r_TS_y * 2.0 + S_r_SF_x * T_r_TS_x * t2 * 2.0 - S_r_SF_x * T_r_TS_z * t3 * 2.0 +
         S_r_SF_z * T_r_TS_x * t3 * 2.0 + S_r_SF_z * T_r_TS_z * t2 * 2.0;
}

double LegKinematicParameters::getTrigonometricEquationCoefficientKfeA() const {
  return aKFE_;
}

double LegKinematicParameters::getTrigonometricEquationCoefficientKfeB() const {
  return bKFE_;
}

double LegKinematicParameters::getTrigonometricEquationCoefficientKfeC() const {
  return cKFE_;
}

void LegKinematicParameters::computeTrigonometricEquationCoefficientsKfe() {
  const double S_r_SF_x = positionShankToFootInShankFrame_.x();
  const double S_r_SF_y = positionShankToFootInShankFrame_.y();
  const double S_r_SF_z = positionShankToFootInShankFrame_.z();
  const double T_r_TS_x = positionThighToShankInThighFrame_.x();
  const double T_r_TS_y = positionThighToShankInThighFrame_.y();
  const double T_r_TS_z = positionThighToShankInThighFrame_.z();

  aKFE_ = S_r_SF_x * T_r_TS_z * -2.0 + S_r_SF_z * T_r_TS_x * 2.0;
  bKFE_ = S_r_SF_x * T_r_TS_x * 2.0 + S_r_SF_z * T_r_TS_z * 2.0;
  cKFE_ = S_r_SF_y * T_r_TS_y * 2.0 + S_r_SF_x * S_r_SF_x + S_r_SF_y * S_r_SF_y + S_r_SF_z * S_r_SF_z + T_r_TS_x * T_r_TS_x +
          T_r_TS_y * T_r_TS_y + T_r_TS_z * T_r_TS_z;
}

}  // namespace analytical_inverse_kinematics