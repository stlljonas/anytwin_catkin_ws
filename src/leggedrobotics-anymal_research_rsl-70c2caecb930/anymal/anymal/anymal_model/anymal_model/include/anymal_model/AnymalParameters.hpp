/**
 * @authors     Dario Bellicoso, Francisco Giraldez Gamez
 * @affiliation ETH Zurich, ANYbotics
 * @brief
 */

#pragma once

#include <analytical_inverse_kinematics/LegKinematicParameters.hpp>
#include <anymal_description/AnymalDescription.hpp>

#include "anymal_model/typedefs.hpp"

namespace anymal_model {

class AnymalParameters {
 public:
  using LegParameters = analytical_inverse_kinematics::LegKinematicParameters;
  AnymalParameters();
  virtual ~AnymalParameters() = default;

  AnymalParameters(const AnymalParameters&) = default;
  AnymalParameters(AnymalParameters&&) = default;

  AnymalParameters& operator=(const AnymalParameters&) = default;
  AnymalParameters& operator=(AnymalParameters&&) = default;

  /** @brief Initializes the kinematic parameters for all legs.
   *
   * @return true iff successful
   */
  bool initialize();

  //! Generic accessor for parameters
  const LegParameters& getLegKinematicParameters(AD::LimbEnum limb) const;

  /**
   * @brief Methods to get specific quantities per leg
   */
  ///@{
  const Position& getPositionBaseToHipInBaseFrameForLeg(AD::LimbEnum limb) const;
  const Position& getPositionHipToThighInHipFrameForLeg(AD::LimbEnum limb) const;
  const Position& getPositionThighToShankInThighFrameForLeg(AD::LimbEnum limb) const;
  const Position& getPositionShankToFootInShankFrameForLeg(AD::LimbEnum limb) const;
  const EulerAnglesZyx& getOrientationHipToBaseForLeg(AD::LimbEnum limb) const;
  const EulerAnglesZyx& getOrientationThighToHipForLeg(AD::LimbEnum limb) const;
  const EulerAnglesZyx& getOrientationShankToThighForLeg(AD::LimbEnum limb) const;
  const EulerAnglesZyx& getOrientationFootToShankForLeg(AD::LimbEnum limb) const;
  ///@}

  /**
   * @brief Methods to fill the whole param map with symmetrical parameters for all limbs.
   */
  ///@{
  void setPositionBaseToHipInBaseFrameSymmetrically(const Position& positionBaseToHipInBaseFrame);
  void setPositionHipToThighInHipFrameSymmetrically(const Position& positionHipToThighInHipFrame);
  void setPositionThighToShankInThighFrameSymmetrically(const Position& positionThighToShankInThighFrame);
  void setPositionShankToFootInShankFrameSymmetrically(const Position& positionShankToFootInShankFrame);
  void setOrientationHipToBaseSymmetrically(const EulerAnglesZyx& eulerAnglesZyx);
  void setOrientationThighToHipSymmetrically(const EulerAnglesZyx& eulerAnglesZyx);
  void setOrientationShankToThighSymmetrically(const EulerAnglesZyx& eulerAnglesZyx);
  void setOrientationFootToShankSymmetrically(const EulerAnglesZyx& eulerAnglesZyx);
  ///@}

  /**
   * @brief Methods to fill in the param map one limb at the time.
   * @param limb The specific limb
   */
  ///@{
  void setPositionBaseToHipInBaseFrame(const Position& positionBaseToHipInBaseFrame, AD::LimbEnum limb);
  void setPositionHipToThighInHipFrame(const Position& positionHipToThighInHipFrame, AD::LimbEnum limb);
  void setPositionThighToShankInThighFrame(const Position& positionThighToShankInThighFrame, AD::LimbEnum limb);
  void setPositionShankToFootInShankFrame(const Position& positionShankToFootInShankFrame, AD::LimbEnum limb);
  void setOrientationHipToBase(const EulerAnglesZyx& eulerAnglesZyx, AD::LimbEnum limb);
  void setOrientationThighToHip(const EulerAnglesZyx& eulerAnglesZyx, AD::LimbEnum limb);
  void setOrientationShankToThigh(const EulerAnglesZyx& eulerAnglesZyx, AD::LimbEnum limb);
  void setOrientationFootToShank(const EulerAnglesZyx& eulerAnglesZyx, AD::LimbEnum limb);
  ///@}

  void printParameters();

 private:
  std::map<AD::LimbEnum, analytical_inverse_kinematics::LegKinematicParameters> legKinematicParameters_;

  std::map<AD::LimbEnum, Vector> flipVector_;
};

}  // namespace anymal_model
