/*
 * PlanningData.hpp
 *
 *  Created on: Oct 16, 2017
 *      Author: Péter Fankhauser
 */

#pragma once

#include <locomotion_planner/common/type_defs.hpp>

#include <vector>

namespace locomotion_planner {

class PlanningData
{
 public:
  enum class FootholdValidityTypes {
    Terrain,
    Kinematic
  };

  PlanningData();
  virtual ~PlanningData();

  void clearAll();
  void addNominalFootsteps(const std::vector<Step>& plan);
  void addOptimizedFootsteps(const std::vector<Step>& plan);
  void addCandidateFoothold(const std::tuple<LimbEnum, Position, std::vector<FootholdValidityTypes>>& foothold);
  void addCandidateFoothold(const LimbEnum& limb, const Position& position, const std::vector<FootholdValidityTypes>& type);
  void addCandidateFootholds(const LimbEnum& limb, const std::vector<FootholdValidityTypes>& type,
                             const std::vector<Position>& positions);

  const std::vector<std::tuple<LimbEnum, Position>>& getNominalFootholds() const;
  const std::vector<std::tuple<LimbEnum, Position>>& getOptimizedFootholds() const;
  const std::vector<std::tuple<LimbEnum, Position, std::vector<FootholdValidityTypes>>>& getCandidateFootholds() const;

 private:
  Pose goalPose_;
  std::vector<std::tuple<LimbEnum, Position>> nominalFootholds_;
  std::vector<std::tuple<LimbEnum, Position>> optimizedFootholds_;
  std::vector<std::tuple<LimbEnum, Position, std::vector<FootholdValidityTypes>>> candidateFootholds_;
};

}
