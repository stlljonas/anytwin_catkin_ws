/**
 * @authors     Dario Bellicoso, Francisco Giraldez Gamez
 * @affiliation ETH Zurich, ANYbotics
 * @brief       Class storing the necessary parameters and trigonometric coefficients for analytical IK of a point-foot 3DOF leg.
 */

#pragma once

#include "analytical_inverse_kinematics/typedefs.hpp"

namespace analytical_inverse_kinematics {

/** @class LegKinematicParameters
 *  @brief Stores the necessary parameters and trigonometric coefficients for analytical inverse kinematics of a 3DOF leg with a point foot.
 *
 *  The parameters are:
 *      * Relative positions of the links (positionParentToChildInParentFrame).
 *      * Relative orientations of the links (orientationChildToParent).
 *  The link frames are defined s.t. the X axis coincides with the previous joint's rotation axis.
 *
 *  The trigonometric coefficients are a, b, c in a*cos(x) + b*sin(x) = c, for HFE and KFE.
 *  The coefficients for HFE depend on the KFE position.
 */
class LegKinematicParameters {
 public:
  LegKinematicParameters() = default;
  virtual ~LegKinematicParameters() = default;

  LegKinematicParameters(const LegKinematicParameters&) = default;
  LegKinematicParameters(LegKinematicParameters&&) = default;

  LegKinematicParameters& operator=(const LegKinematicParameters&) = default;
  LegKinematicParameters& operator=(LegKinematicParameters&&) = default;

  //! @brief Initializes all derived parameters. Requires the relative positions and orientations (base params) to be set.
  void initialize();

  //! @brief Prints the parameters for a given leg, for debugging purposes.
  void printParameters();

  //! @brief Returns the position HIP -> FOOT in zero configuration.
  const Position& getDefaultPositionHipToFootInBaseFrame() const;

  //! Getters for relative link positions.
  ///@{
  const Position& getPositionBaseToHipInBaseFrame() const;
  const Position& getPositionHipToThighInHipFrame() const;
  const Position& getPositionThighToShankInThighFrame() const;
  const Position& getPositionShankToFootInShankFrame() const;
  ///@}

  //! Getters for relative link orientations.
  ///@{
  const EulerAnglesZyx& getOrientationHipToBase() const;
  const EulerAnglesZyx& getOrientationThighToHip() const;
  const EulerAnglesZyx& getOrientationShankToThigh() const;
  const EulerAnglesZyx& getOrientationFootToShank() const;
  ///@}

  //! Getters for trigonometric coefficients.
  ///@{
  double getTrigonometricEquationCoefficientHfeA(double qKFE) const;
  double getTrigonometricEquationCoefficientHfeB(double qKFE) const;
  double getTrigonometricEquationCoefficientHfeC(double qKFE) const;
  double getTrigonometricEquationCoefficientKfeA() const;
  double getTrigonometricEquationCoefficientKfeB() const;
  double getTrigonometricEquationCoefficientKfeC() const;
  ///@}

  //! Getters for relative link positions.
  ///@{
  void setPositionBaseToHipInBaseFrame(const Position& positionBaseToHipInBaseFrame);
  void setPositionHipToThighInHipFrame(const Position& positionHipToThighInHipFrame);
  void setPositionThighToShankInThighFrame(const Position& positionThighToShankInThighFrame);
  void setPositionShankToFootInShankFrame(const Position& positionShankToFootInShankFrame);
  ///@}

  //! Getters for relative link orientations.
  ///@{
  void setOrientationHipToBase(const EulerAnglesZyx& orientationHipToBase);
  void setOrientationThighToHip(const EulerAnglesZyx& orientationThighToHip);
  void setOrientationShankToThigh(const EulerAnglesZyx& orientationShankToThigh);
  void setOrientationFootToShank(const EulerAnglesZyx& orientationFootToShank);
  ///@}

 private:
  //! Computes the KFE trig. coeff - which are config-independent.
  void computeTrigonometricEquationCoefficientsKfe();

  //! Link relative positions (in zero config).
  Position positionBaseToHipInBaseFrame_;
  Position positionHipToThighInHipFrame_;
  Position positionThighToShankInThighFrame_;
  Position positionShankToFootInShankFrame_;

  //! Link relative orientations (in zero config).
  EulerAnglesZyx orientationHipToBase_;
  EulerAnglesZyx orientationThighToHip_;
  EulerAnglesZyx orientationShankToThigh_;
  EulerAnglesZyx orientationFootToShank_;

  //! Pre-computed trigonometric equation parameters.
  double aKFE_ = 0.0;
  double bKFE_ = 0.0;
  double cKFE_ = 0.0;

  //! Position hip to foot in zero configuration.
  Position defaultPositionHipToFootInBaseFrame_;
};

}  // namespace analytical_inverse_kinematics