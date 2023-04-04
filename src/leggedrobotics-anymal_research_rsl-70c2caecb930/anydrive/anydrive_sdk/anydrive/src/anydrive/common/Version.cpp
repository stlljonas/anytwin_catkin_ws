#include <sstream>

#include "anydrive/common/Version.hpp"

namespace anydrive {
namespace common {

Version::Version(const int major, const int minor, const int patch) : major_(major), minor_(minor), patch_(patch) {}

Version::Version(const std::string& string) {
  sscanf(string.c_str(), "%d.%d.%d", &major_, &minor_, &patch_);
}

Version& Version::fromString(const std::string& string) {
  *this = Version(string);
  return *this;
}

std::string Version::toString() const {
  std::stringstream ss;
  ss << major_ << "." << minor_ << "." << patch_;
  return ss.str();
}

bool Version::operator==(const Version& other) const {
  return (major_ == other.major_ && minor_ == other.minor_ && patch_ == other.patch_);
}

bool Version::operator!=(const Version& other) const {
  return !(*this == other);
}

bool Version::operator<(const Version& other) const {
  if (major_ < other.major_) {
    return true;
  } else if (major_ > other.major_) {
    return false;
  } else if (minor_ < other.minor_) {
    return true;
  } else if (minor_ > other.minor_) {
    return false;
  }
  return patch_ < other.patch_;
}

bool Version::operator<=(const Version& other) const {
  return (*this == other || *this < other);
}

bool Version::operator>(const Version& other) const {
  return !(*this <= other);
}

bool Version::operator>=(const Version& other) const {
  return !(*this < other);
}

std::ostream& operator<<(std::ostream& ostream, const Version& version) {
  ostream << version.toString();
  return ostream;
}

}  // namespace common
}  // namespace anydrive
