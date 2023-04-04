/*!
 * @file     FootPlacementStrategyStaticGait.hpp
 * @author   C. Dario Bellicoso, Christian Gehring
 * @date     Oct 6, 2014
 * @brief
 */

#pragma once

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyBase.hpp"
#include "loco/foothold_generation/FootholdGeneratorStaticGait.hpp"
#include "loco/gait_pattern/GaitPatternStaticGait.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp"
#include "loco/torso_control/ComSupportControlStaticGait.hpp"

class TiXmlHandle;

namespace loco {

class FootPlacementStrategyStaticGait : public FootPlacementStrategyBase {
 public:
  FootPlacementStrategyStaticGait(WholeBody& wholeBody, TerrainModelBase& terrain, GaitPatternStaticGait& gaitPattern,
                                  ComSupportControlStaticGait& comControl, SwingTrajectoryGeneratorBase& swingTrajectoryGenerator);
  ~FootPlacementStrategyStaticGait() override = default;

  /****************
   * Core methods *
   ****************/
  bool advance(double dt) override;
  bool initialize(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;
  /****************/

  bool setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1, const FootPlacementStrategyBase& footPlacementStrategy2,
                         double t) override;

  virtual bool addVariablesToLog(bool updateLogger);
  virtual void planFootholds();

  const SwingTrajectoryGeneratorBase& getSwingTrajectoryGenerator() const override;
  SwingTrajectoryGeneratorBase* getSwingTrajectoryGeneratorPtr() override;

 protected:
  virtual void regainContact(LegBase& leg, double dt);
  virtual void setFootTrajectory(LegBase& leg, double dt);
  void computeLegLoad(LegBase& leg, double startRampingDownAtStancePhase, double endRampingUpAtStancePhase, double minLegLoad);

  TorsoBase& torso_;
  Legs& legs_;
  TerrainModelBase& terrain_;
  GaitPatternStaticGait& gaitPattern_;
  ComSupportControlStaticGait& comControl_;
  SwingTrajectoryGeneratorBase& swingTrajectoryGenerator_;
  FootholdGeneratorStaticGait footholdGenerator_;

  bool didPlannedFootholdForNextPhase_;

  double maxStepLengthHeading_;
  double maxStepLengthLateral_;

  std::vector<double> regainFactor_;
  std::vector<bool> rampingDown_;
  std::vector<double> timeSinceSupport_;
  std::vector<double> legLoad_;

  double dt_;
  double maxDistanceHipToFoot_;
  double regainContactSpeed_;

  Eigen::Matrix<double, 3, 4> footprintAtLastStance_;

  bool regainIndefinitely_;
};

} /* namespace loco */
