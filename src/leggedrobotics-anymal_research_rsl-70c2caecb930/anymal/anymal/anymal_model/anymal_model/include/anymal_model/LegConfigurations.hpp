/*!
 * @file     LegConfigurations.hpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Dec, 2014
 */
#pragma once

namespace anymal_model {

class LegConfigurations {
 public:
  constexpr LegConfigurations(bool isLeftForeLegBentNormal, bool isRightForeLegBentNormal, bool isLeftHindLegBentNormal,
                              bool isRightHindLegBentNormal)
      : isLeftForeLegBentNormal_(isLeftForeLegBentNormal),
        isRightForeLegBentNormal_(isRightForeLegBentNormal),
        isLeftHindLegBentNormal_(isLeftHindLegBentNormal),
        isRightHindLegBentNormal_(isRightHindLegBentNormal) {}

  virtual ~LegConfigurations() = default;

  bool operator[](int iLeg) const;

 private:
  bool isLeftForeLegBentNormal_;
  bool isRightForeLegBentNormal_;
  bool isLeftHindLegBentNormal_;
  bool isRightHindLegBentNormal_;
};

class LegConfigurationXX : public LegConfigurations {
 public:
  constexpr LegConfigurationXX() : LegConfigurations(true, true, false, false) {}
  ~LegConfigurationXX() override = default;
};

class LegConfigurationOO : public LegConfigurations {
 public:
  constexpr LegConfigurationOO() : LegConfigurations(false, false, true, true) {}
  ~LegConfigurationOO() override = default;
};

class LegConfigurationXO : public LegConfigurations {
 public:
  constexpr LegConfigurationXO() : LegConfigurations(true, true, true, true) {}
  ~LegConfigurationXO() override = default;
};

class LegConfigurationOX : public LegConfigurations {
 public:
  constexpr LegConfigurationOX() : LegConfigurations(false, false, false, false) {}
  ~LegConfigurationOX() override = default;
};

}  // namespace anymal_model
