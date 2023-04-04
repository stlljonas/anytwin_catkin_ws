/*
 * FootholdGeneratorInvertedPendulum.hpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumBase.hpp"

namespace loco {

class FootholdGeneratorInvertedPendulum : public FootholdGeneratorInvertedPendulumBase {
 public:
  FootholdGeneratorInvertedPendulum(WholeBody& wholeBody, TerrainModelBase& terrain);
  ~FootholdGeneratorInvertedPendulum() override = default;

  Position computeWorldToFootholdInWorldFrame(int legId) override;
  bool loadParameters(const TiXmlHandle& handle) override;

  friend std::ostream& operator<<(std::ostream& out, const FootholdGeneratorInvertedPendulum& fhGen);

  LinearVelocity computeLinearVelocityProjectionInControlFrame(const LegBase& leg) const override;

  LinearVelocity computeLinearVelocityErrorInControlFrame(const LegBase& leg) const override;

 protected:
  Position evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg) override;
  Position evaluateDesiredFootHoldOnTerrainFeedBackInControlFrame(const LegBase& leg) override;
};

} /* namespace loco */
