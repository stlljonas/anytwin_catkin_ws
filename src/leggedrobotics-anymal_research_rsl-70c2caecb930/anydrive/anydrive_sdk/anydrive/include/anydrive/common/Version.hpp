#pragma once

#include <iostream>
#include <string>

namespace anydrive {
namespace common {

struct Version {
  int major_ = 0;
  int minor_ = 0;
  int patch_ = 0;

  /*!
   * Create a version number 0.0.0.
   */
  Version() = default;

  /*!
   * Create a version number by numbers.
   * @param major Major number.
   * @param minor Minor number.
   * @param patch Patch number.
   */
  Version(const int major, const int minor, const int patch);

  /*!
   * Create a version number by parsing a string of the form "major.minor.patch".
   * @param string String.
   */
  explicit Version(const std::string& string);

  /*!
   * Create a version number by parsing a string of the form "major.minor.patch".
   * @param string String.
   */
  Version& fromString(const std::string& string);

  /*!
   * Create a string from a version number.
   */
  std::string toString() const;

  bool operator==(const Version& other) const;
  bool operator!=(const Version& other) const;
  bool operator<(const Version& other) const;
  bool operator<=(const Version& other) const;
  bool operator>(const Version& other) const;
  bool operator>=(const Version& other) const;
};

std::ostream& operator<<(std::ostream& ostream, const Version& version);

}  // namespace common
}  // namespace anydrive
