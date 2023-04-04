/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Classes for limit checks. Derive from StateCheckBase and use these classes to implement a limit check.
 */

#pragma once

// stl
#include <string>

namespace anymal_roco {

class LimitCheckBase {

 public:
  //! Default Constructor
  LimitCheckBase() = default;

  //! Default Destructor
  virtual ~LimitCheckBase() = default;

  //! Check limit
  virtual bool checkLimit(double value) const = 0;

  //! Partial warn message (to be used in the derived class)
  virtual std::string warnMessage() const = 0;
};

class NoLimitCheck : public LimitCheckBase {

 public:
  //! Constructor
  NoLimitCheck() = default;

  //! Default Destructor
  ~NoLimitCheck() override = default;

  //! Check limit
  bool checkLimit(double value) const override { return true; };

  //! Partial warn message (to be used in the derived class)
  std::string warnMessage() const override { return "not checked for limits"; };

};

class LowerLimitCheck : public LimitCheckBase {

 public:
  //! Constructor
  LowerLimitCheck(double lowerLimit) : lowerLimit_(lowerLimit) {};

  //! Default Destructor
  ~LowerLimitCheck() override = default;

  //! Check limit
  bool checkLimit(double value) const override { return value >= lowerLimit_; };

  //! Partial warn message (to be used in the derived class)
  std::string warnMessage() const override { return "not larger equal than the lower limit " + std::to_string(lowerLimit_); };

 private:
  double lowerLimit_;
};

class UpperLimitCheck : public LimitCheckBase {

 public:
  //! Constructor
  UpperLimitCheck(double upperLimit) : upperLimit_(upperLimit) {};

  //! Default Destructor
  ~UpperLimitCheck() override = default;

  //! Check limit
  bool checkLimit(double value) const override { return value <= upperLimit_; };

  //! Partial warn message (to be used in the derived class)
  std::string warnMessage() const override { return "not smaller equal than the upper limit " + std::to_string(upperLimit_); };

 private:
  double upperLimit_;

};

class LowerAndUpperLimitCheck : public LimitCheckBase {

 public:
  //! Constructor
  LowerAndUpperLimitCheck(double lowerLimit, double upperLimit) : lowerLimit_(lowerLimit), upperLimit_(upperLimit) {};

  //! Default Destructor
  ~LowerAndUpperLimitCheck() override = default;

  //! Check limit
  bool checkLimit(double value) const override { return (value >= lowerLimit_) && (value <= upperLimit_); };

  //! Partial warn message (to be used in the derived class)
  std::string warnMessage() const override { return "not between the lower limit " + std::to_string(lowerLimit_) + " and the upper limit " + std::to_string(upperLimit_); };

 private:
  double lowerLimit_;
  double upperLimit_;
};

} /* namespace anymal_roco */
