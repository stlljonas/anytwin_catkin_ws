/**
 * @authors     Dario Bellicoso, Francisco Giraldez Gamez
 * @affiliation ETH Zurich, ANYbotics
 * @brief
 */

#include "anymal_model/AnymalParameters.hpp"

namespace anymal_model {

AnymalParameters::AnymalParameters() {
  flipVector_[AD::LimbEnum::LF_LEG] = Vector(1, 1, 1);
  flipVector_[AD::LimbEnum::RF_LEG] = Vector(1, -1, 1);
  flipVector_[AD::LimbEnum::LH_LEG] = Vector(-1, 1, 1);
  flipVector_[AD::LimbEnum::RH_LEG] = Vector(-1, -1, 1);

  for (auto& limbKey : AD::getLimbKeys()) {
    const auto limb = limbKey.getEnum();
    legKinematicParameters_.emplace(limb, LegParameters());
  }
}

bool AnymalParameters::initialize() {
  for (auto& limbKey : AD::getLimbKeys()) {
    const auto limb = limbKey.getEnum();
    legKinematicParameters_.at(limb).initialize();
  }

  return true;
}

const AnymalParameters::LegParameters& AnymalParameters::getLegKinematicParameters(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb);
}

const Position& AnymalParameters::getPositionBaseToHipInBaseFrameForLeg(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb).getPositionBaseToHipInBaseFrame();
}

const Position& AnymalParameters::getPositionHipToThighInHipFrameForLeg(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb).getPositionBaseToHipInBaseFrame();
}

const Position& AnymalParameters::getPositionThighToShankInThighFrameForLeg(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb).getPositionThighToShankInThighFrame();
}

const Position& AnymalParameters::getPositionShankToFootInShankFrameForLeg(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb).getPositionShankToFootInShankFrame();
}

const EulerAnglesZyx& AnymalParameters::getOrientationHipToBaseForLeg(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb).getOrientationHipToBase();
}

const EulerAnglesZyx& AnymalParameters::getOrientationThighToHipForLeg(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb).getOrientationThighToHip();
}

const EulerAnglesZyx& AnymalParameters::getOrientationShankToThighForLeg(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb).getOrientationShankToThigh();
}

const EulerAnglesZyx& AnymalParameters::getOrientationFootToShankForLeg(AD::LimbEnum limb) const {
  return legKinematicParameters_.at(limb).getOrientationFootToShank();
}

void AnymalParameters::setPositionBaseToHipInBaseFrameSymmetrically(const Position& positionBaseToHipInBaseFrame) {
  for (auto& limbKey : AD::getLimbKeys()) {
    const auto limb = limbKey.getEnum();
    setPositionBaseToHipInBaseFrame(
        Position(positionBaseToHipInBaseFrame.toImplementation().cwiseProduct(flipVector_.at(limb).toImplementation())), limb);
  }
}

void AnymalParameters::setPositionHipToThighInHipFrameSymmetrically(const Position& positionHipToThighInHipFrame) {
  for (auto& limbKey : AD::getLimbKeys()) {
    const auto limb = limbKey.getEnum();
    setPositionHipToThighInHipFrame(
        Position(positionHipToThighInHipFrame.toImplementation().cwiseProduct(flipVector_.at(limb).toImplementation())), limb);
  }
}

void AnymalParameters::setPositionThighToShankInThighFrameSymmetrically(const Position& positionThighToShankInThighFrame) {
  for (auto& limbKey : AD::getLimbKeys()) {
    const auto limb = limbKey.getEnum();
    setPositionThighToShankInThighFrame(
        Position(positionThighToShankInThighFrame.toImplementation().cwiseProduct(flipVector_.at(limb).toImplementation())), limb);
  }
}

void AnymalParameters::setPositionShankToFootInShankFrameSymmetrically(const Position& positionShankToFootInShankFrame) {
  for (auto& limbKey : AD::getLimbKeys()) {
    const auto limb = limbKey.getEnum();
    setPositionShankToFootInShankFrame(
        Position(positionShankToFootInShankFrame.toImplementation().cwiseProduct(flipVector_.at(limb).toImplementation())), limb);
  }
}

void AnymalParameters::setOrientationHipToBaseSymmetrically(const EulerAnglesZyx& eulerAnglesZyx) {
  setOrientationHipToBase(EulerAnglesZyx(eulerAnglesZyx.yaw(), eulerAnglesZyx.pitch(), eulerAnglesZyx.roll()).getUnique(),
                          AD::LimbEnum::LF_LEG);
  setOrientationHipToBase(EulerAnglesZyx(eulerAnglesZyx.yaw(), eulerAnglesZyx.pitch(), -eulerAnglesZyx.roll()).getUnique(),
                          AD::LimbEnum::RF_LEG);
  setOrientationHipToBase(EulerAnglesZyx(eulerAnglesZyx.yaw(), -eulerAnglesZyx.pitch(), eulerAnglesZyx.roll()).getUnique(),
                          AD::LimbEnum::LH_LEG);
  setOrientationHipToBase(EulerAnglesZyx(eulerAnglesZyx.yaw(), -eulerAnglesZyx.pitch(), -eulerAnglesZyx.roll()).getUnique(),
                          AD::LimbEnum::RH_LEG);
}

void AnymalParameters::setOrientationThighToHipSymmetrically(const EulerAnglesZyx& eulerAnglesZyx) {
  setOrientationThighToHip(EulerAnglesZyx(eulerAnglesZyx.yaw(), eulerAnglesZyx.pitch(), eulerAnglesZyx.roll()).getUnique(),
                           AD::LimbEnum::LF_LEG);
  setOrientationThighToHip(EulerAnglesZyx(eulerAnglesZyx.yaw(), eulerAnglesZyx.pitch(), -eulerAnglesZyx.roll()).getUnique(),
                           AD::LimbEnum::RF_LEG);
  setOrientationThighToHip(EulerAnglesZyx(eulerAnglesZyx.yaw(), -eulerAnglesZyx.pitch(), eulerAnglesZyx.roll()).getUnique(),
                           AD::LimbEnum::LH_LEG);
  setOrientationThighToHip(EulerAnglesZyx(eulerAnglesZyx.yaw(), -eulerAnglesZyx.pitch(), -eulerAnglesZyx.roll()).getUnique(),
                           AD::LimbEnum::RH_LEG);
}

void AnymalParameters::setOrientationShankToThighSymmetrically(const EulerAnglesZyx& eulerAnglesZyx) {
  setOrientationShankToThigh(EulerAnglesZyx(eulerAnglesZyx.yaw(), eulerAnglesZyx.pitch(), eulerAnglesZyx.roll()).getUnique(),
                             AD::LimbEnum::LF_LEG);
  setOrientationShankToThigh(EulerAnglesZyx(eulerAnglesZyx.yaw(), eulerAnglesZyx.pitch(), -eulerAnglesZyx.roll()).getUnique(),
                             AD::LimbEnum::RF_LEG);
  setOrientationShankToThigh(EulerAnglesZyx(eulerAnglesZyx.yaw(), -eulerAnglesZyx.pitch(), eulerAnglesZyx.roll()).getUnique(),
                             AD::LimbEnum::LH_LEG);
  setOrientationShankToThigh(EulerAnglesZyx(eulerAnglesZyx.yaw(), -eulerAnglesZyx.pitch(), -eulerAnglesZyx.roll()).getUnique(),
                             AD::LimbEnum::RH_LEG);
}

void AnymalParameters::setOrientationFootToShankSymmetrically(const EulerAnglesZyx& eulerAnglesZyx) {
  setOrientationFootToShank(EulerAnglesZyx(eulerAnglesZyx.yaw(), eulerAnglesZyx.pitch(), eulerAnglesZyx.roll()).getUnique(),
                            AD::LimbEnum::LF_LEG);
  setOrientationFootToShank(EulerAnglesZyx(eulerAnglesZyx.yaw(), eulerAnglesZyx.pitch(), -eulerAnglesZyx.roll()).getUnique(),
                            AD::LimbEnum::RF_LEG);
  setOrientationFootToShank(EulerAnglesZyx(eulerAnglesZyx.yaw(), -eulerAnglesZyx.pitch(), eulerAnglesZyx.roll()).getUnique(),
                            AD::LimbEnum::LH_LEG);
  setOrientationFootToShank(EulerAnglesZyx(eulerAnglesZyx.yaw(), -eulerAnglesZyx.pitch(), -eulerAnglesZyx.roll()).getUnique(),
                            AD::LimbEnum::RH_LEG);
}

void AnymalParameters::setPositionBaseToHipInBaseFrame(const Position& positionBaseToHipInBaseFrame, AD::LimbEnum limb) {
  legKinematicParameters_.at(limb).setPositionBaseToHipInBaseFrame(positionBaseToHipInBaseFrame);
}

void AnymalParameters::setPositionHipToThighInHipFrame(const Position& positionHipToThighInHipFrame, AD::LimbEnum limb) {
  legKinematicParameters_.at(limb).setPositionHipToThighInHipFrame(positionHipToThighInHipFrame);
}

void AnymalParameters::setPositionThighToShankInThighFrame(const Position& positionThighToShankInThighFrame, AD::LimbEnum limb) {
  legKinematicParameters_.at(limb).setPositionThighToShankInThighFrame(positionThighToShankInThighFrame);
}

void AnymalParameters::setPositionShankToFootInShankFrame(const Position& positionShankToFootInShankFrame, AD::LimbEnum limb) {
  legKinematicParameters_.at(limb).setPositionShankToFootInShankFrame(positionShankToFootInShankFrame);
}

void AnymalParameters::setOrientationHipToBase(const EulerAnglesZyx& eulerAnglesZyx, AD::LimbEnum limb) {
  legKinematicParameters_.at(limb).setOrientationHipToBase(eulerAnglesZyx);
}

void AnymalParameters::setOrientationThighToHip(const EulerAnglesZyx& eulerAnglesZyx, AD::LimbEnum limb) {
  legKinematicParameters_.at(limb).setOrientationThighToHip(eulerAnglesZyx);
}

void AnymalParameters::setOrientationShankToThigh(const EulerAnglesZyx& eulerAnglesZyx, AD::LimbEnum limb) {
  legKinematicParameters_.at(limb).setOrientationShankToThigh(eulerAnglesZyx);
}

void AnymalParameters::setOrientationFootToShank(const EulerAnglesZyx& eulerAnglesZyx, AD::LimbEnum limb) {
  legKinematicParameters_.at(limb).setOrientationFootToShank(eulerAnglesZyx);
}

void AnymalParameters::printParameters() {
  std::cout << "Kinematic parameters." << std::endl;

  for (auto& limbKey : AD::getLimbKeys()) {
    const auto limb = limbKey.getEnum();
    const auto limbName = limbKey.getName();
    std::cout << "Limb name: " << limbName << std::endl;
    legKinematicParameters_.at(limb).printParameters();
  }
}

}  // namespace anymal_model
