/*
 * PlanningData.cpp
 *
 *  Created on: Oct 16, 2017
 *      Author: Péter Fankhauser
 */

#include <locomotion_planner/common/PlanningData.hpp>

#include <free_gait_core/free_gait_core.hpp>

namespace locomotion_planner {

PlanningData::PlanningData()
{
}

PlanningData::~PlanningData()
{
}

void PlanningData::clearAll()
{
  goalPose_.setIdentity();
  nominalFootholds_.clear();
  optimizedFootholds_.clear();
  candidateFootholds_.clear();
}

void PlanningData::addNominalFootsteps(const std::vector<Step>& plan)
{
  for (auto& step : plan) {
    if (step.hasLegMotion()) {
      for (auto& legMotion : step.getLegMotions()) {
        if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
          auto& footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);
          const Position targetPosition = footstep.getTargetPosition();
          nominalFootholds_.push_back(std::make_tuple(legMotion.first, targetPosition));
        }
      }
    }
  }
}

void PlanningData::addOptimizedFootsteps(const std::vector<Step>& plan)
{
  for (auto& step : plan) {
    if (step.hasLegMotion()) {
      for (auto& legMotion : step.getLegMotions()) {
        if (legMotion.second->getType() == free_gait::LegMotionBase::Type::Footstep) {
          auto& footstep = dynamic_cast<free_gait::Footstep&>(*legMotion.second);
          const Position targetPosition = footstep.getTargetPosition();
          optimizedFootholds_.push_back(std::make_tuple(legMotion.first, targetPosition));
        }
      }
    }
  }
}

void PlanningData::addCandidateFoothold(const std::tuple<LimbEnum, Position, std::vector<FootholdValidityTypes>>& foothold)
{
  candidateFootholds_.push_back(foothold);
}

void PlanningData::addCandidateFoothold(const LimbEnum& limb, const Position& position,
                                        const std::vector<FootholdValidityTypes>& type)
{
  addCandidateFoothold(std::make_tuple(limb, position, type));
}

void PlanningData::addCandidateFootholds(const LimbEnum& limb, const std::vector<FootholdValidityTypes>& type,
                                        const std::vector<Position>& positions)
{
  for (const auto& position : positions) {
    addCandidateFoothold(std::make_tuple(limb, position, type));
  }
}

const std::vector<std::tuple<LimbEnum, Position> >& PlanningData::getNominalFootholds() const
{
  return nominalFootholds_;
}

const std::vector<std::tuple<LimbEnum, Position> >& PlanningData::getOptimizedFootholds() const
{
  return optimizedFootholds_;
}

const std::vector<std::tuple<LimbEnum, Position, std::vector<PlanningData::FootholdValidityTypes>>>&
PlanningData::getCandidateFootholds() const
{
  return candidateFootholds_;
}

}
