/*
 * TaskRomo.hpp
 *
 *  Created on: May 11, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"

// whole_body_control
#include <whole_body_control/Task.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class TaskRomo : public whole_body_control::Task {
 protected:
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using LimbEnum = typename RD::LimbEnum;

 public:
  explicit TaskRomo(const WholeBodyState& wholeBodyState)
      :  whole_body_control::Task(), wholeBodyState_(wholeBodyState) {}

  ~TaskRomo() override = default;

 public:
  const WholeBodyState& getWholeBodyState() const { return wholeBodyState_; }

 protected:
  std::vector<std::string> getListFromCommaSeparatedString(std::string s) {
    std::vector<std::string> list;
    s.erase(std::remove_if(s.begin(), s.end(), [](char c) { return std::isspace(c); }), s.end());
    const std::string delimiter = ",";
    size_t pos = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
      list.emplace_back(s.substr(0, pos));
      s.erase(0, pos + delimiter.length());
    }
    list.emplace_back(s);
    return list;
  }

  std::vector<std::string> getLimbNamesFromCommaSeperatedString(std::string s) {
    std::vector<std::string> limbNames = getListFromCommaSeparatedString(s);
    // Accept all, legs and arms as abbreviations
    if (limbNames.front() == "all") {
      limbNames.clear();
      for (auto limb : this->wholeBodyState_.getWholeBody().getLimbs()) {
        limbNames.push_back(RD::template mapKeyIdToKeyName<LimbEnum>(limb->getLimbUInt()));
      }
    } else if (limbNames.front() == "legs") {
      limbNames.clear();
      for (auto leg : this->wholeBodyState_.getWholeBody().getLegs()) {
        limbNames.push_back(RD::template mapKeyIdToKeyName<LimbEnum>(leg->getLimbUInt()));
      }
    } else if (limbNames.front() == "arms") {
      limbNames.clear();
      for (auto arm : this->wholeBodyState_.getWholeBody().getArms()) {
        limbNames.push_back(RD::template mapKeyIdToKeyName<LimbEnum>(arm->getLimbUInt()));
      }
    }
    return limbNames;
  }

 private:
  const WholeBodyState& wholeBodyState_;

};

}  // namespace whole_body_control_romo
