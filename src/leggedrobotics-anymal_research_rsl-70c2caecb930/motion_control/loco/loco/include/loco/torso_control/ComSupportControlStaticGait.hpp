/*!
 * @file     ComSupportControlStaticGait.hpp
 * @author   C. Dario Bellicoso
 * @date     Oct 7, 2014
 * @brief
 */

#pragma once

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/gait_pattern/GaitPatternStaticGait.hpp"
#include "loco/torso_control/ComSupportControlBase.hpp"

// robot utils
#include "curves/PolynomialSplineContainer.hpp"
#include "robot_utils/function_approximators/catmullRomSplines/CatmullRomSpline.hpp"
#include "robot_utils/math/math.hpp"

// std utils
#include "std_utils/std_utils.hpp"

// boost
#include <boost/thread.hpp>

// stl
#include <memory>
#include <mutex>

namespace loco {

class ComSupportControlStaticGait : public ComSupportControlBase {
 public:
  using FeetConfiguration = Eigen::Matrix<double, 2, 4>;
  using SupportTriangle = Eigen::Matrix<double, 2, 3>;
  using Line = Eigen::Matrix<double, 2, 2>;
  using Pos2d = Eigen::Vector2d;

  ComSupportControlStaticGait(WholeBody& wholeBody, GaitPatternStaticGait& gaitPattern);
  ~ComSupportControlStaticGait() override = default;

  bool advance(double dt) override;
  bool initialize(double dt) override;

  const Position& getPositionWorldToDesiredCoMInWorldFrame() const override;

  const SupportTriangle& getSupportTriangleCurrent() const;
  const SupportTriangle& getSupportTriangleNext() const;
  const SupportTriangle& getSupportTriangleOverNext() const;

  const SupportTriangle& getSafeTriangleCurrent() const;
  const SupportTriangle& getSafeTriangleNext() const;
  const SupportTriangle& getSafeTriangleOverNext() const;

  bool setToInterpolated(const ComSupportControlBase& supportPolygon1, const ComSupportControlBase& supportPolygon2, double t) override;

  virtual void setFootHold(int legId, const Position& footHold);
  virtual const std::vector<Position>& getFootHolds() const;

  /*! Loads the parameters from the XML object
   * @param handle handle
   * @return  true if all parameters could be loaded
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  virtual void setIsInStandConfiguration(bool isInStandConfiguration);
  virtual bool isInStandConfiguration() const;

  const WholeBody& getWholeBody() const;
  WholeBody* getWholeBodyPtr() const;

  const TorsoBase& getTorso() const;
  TorsoBase* getTorsoPtr() const;

  bool addVariablesToLog(bool updateLogger);

 protected:
  WholeBody& wholeBody_;
  TorsoBase& torso_;
  GaitPatternStaticGait& gaitPattern_;

  robot_utils::catmull_rom::Trajectory1D comInterpolationFunction_;

  std::vector<Position> plannedFootHolds_;

  bool isInStandConfiguration_;
  bool didUpdateSafeTriangles_;

  Pos2d comTarget_;
  Pos2d comTargetPrevious_;

  SupportTriangle supportTriangleCurrent_, supportTriangleNext_, supportTriangleOverNext_;
  SupportTriangle safeTriangleCurrent_, safeTriangleNext_, safeTriangleOverNext_;

  FeetConfiguration feetConfigurationLastFullStance_;
  FeetConfiguration feetConfigurationCurrent_;
  FeetConfiguration feetConfigurationNext_;

  //! Distance between the safe triangle and the support triangle segments
  double delta_;

  curves::PolynomialSplineQuintic splineQuintic_;

  virtual void updateSafeSupportTriangles();

  //! Get the next stance feet positions based on the gait planner
  FeetConfiguration getNextStanceConfig(const FeetConfiguration& currentStanceConfig, int steppingFoot);

  //! Get safe triangle from support triangle
  SupportTriangle getSafeTriangle(const Eigen::Matrix<double, 2, 3>& supportTriangle);

  //! Find the intersection (if it exists) between two lines
  bool lineIntersect(const Line& l1, const Line& l2, Pos2d& intersection);

  std::vector<int> getDiagonalElements(int swingLeg) const;

  /*********************
   * CoG update method *
   *********************/
  enum CogUpdateMethod { CogUpdateStd = 0, None };
  CogUpdateMethod updateMethod_;
  virtual void updateCogPosition(bool forceShift);
  virtual void updateCogPositionStd(bool forceShift);
  /*********************/

  std_utils::HighResolutionClockTimer chronoTimer_;

  // Logged variables
  double comTrajectoryPhase_;
  double standTime_;

  bool didOptimizationSucceed_;

  Position positionWorldToDesiredCoMInWorldFrameOld_;

  unsigned int nextSwingLegIdAtStance_;
  unsigned int lastExecutedLegIdAtStance_;
};

} /* namespace loco */
