/*
 * ParameterHandler.hpp
 *
 *  Created on: Mar 26, 2018
 *      Author: Fabian Jenelten
 */
#pragma once

// message logger
#include "message_logger/message_logger.hpp"

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

namespace loco {
namespace motion_generation {

template<typename Params_>
class ParameterHandler {
public:
  ParameterHandler() : params_(), mapGaitNameToId_() { }

  virtual ~ParameterHandler() = default;

  virtual void clear() {
    params_.clear();
    mapGaitNameToId_.clear();
  }

  virtual bool loadParameters(
      const TiXmlHandle& gaitHandle,
      unsigned int gaitIndex,
      const std::string& gaitPatternTypeStr) = 0;

  //! Get parameters by gait name.
  virtual const Params_& getParams(const std::string& gaitPatternTypeStr) const {
    return params_[mapGaitNameToId_.at(gaitPatternTypeStr)];
  }

  virtual Params_* getParamsPtr(const std::string& gaitPatternTypeStr) {
    return &params_[mapGaitNameToId_.at(gaitPatternTypeStr)];
  }

  bool check(const std::map<std::string, unsigned int>& mapGaitNameToId) {
    // Check if all the gaits loaded also exist.
    for (const auto& gaitNameIdPar : mapGaitNameToId_) {
      if (mapGaitNameToId.find(gaitNameIdPar.first) == mapGaitNameToId.end()) {
        std::cout << "Gait " << gaitNameIdPar.first << " is not required but was loaded.\n";
        return false;
      }
    }

    // Check if we have loaded all required gaits.
    for (const auto& gaitNameIdPar : mapGaitNameToId) {
      if (mapGaitNameToId_.find(gaitNameIdPar.first) == mapGaitNameToId_.end()) {
        std::cout << "Gait " << gaitNameIdPar.first << " does not exist but is required.\n";
        return false;
      }
    }
    return true;
  }

  bool addGait(unsigned int gaitIndex, const std::string& gaitPatternTypeStr) {
    // Check if gaitIndex already exists.
    for (const auto& gaitNameIdPar : mapGaitNameToId_) {
      if (gaitNameIdPar.second == gaitIndex) {
        mapGaitNameToId_.insert(std::make_pair(gaitPatternTypeStr, gaitIndex));
        return true;
      }
    }

    std::cout << "Gait index " << gaitIndex << " does not exist. Cannot add gait.\n";
    return false;

  }

protected:
  //! Add an empty set of parameters.
  virtual void emplace_params(unsigned int gaitIndex, const std::string& gaitPatternTypeStr) {
    params_.emplace_back(Params_());
    mapGaitNameToId_.insert(std::make_pair(gaitPatternTypeStr, gaitIndex));
  }

  //! Container for gait varying parameters.
  std::vector<Params_> params_;

  //! A list of names of available gaits. Gait indexes may not coincide with the gait pattern.
  std::map<std::string, unsigned int> mapGaitNameToId_;
};



} /* motion_generation */
} /* namespace loco */
