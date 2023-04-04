// Copied from Paul Furgale's Schweizer-Messer

#pragma once

#include <string>
#include <iostream>
#include <sstream>
// A class and macro that gives you the current file position.

namespace numopt_common {

  class source_file_pos
  {
  public:
    std::string function;
    std::string file;
    int line;

    source_file_pos(std::string function, std::string file, int line) :
      function(function), file(file), line(line) {}

    operator std::string()
    {
      return toString();
    }

    std::string toString() const
    {
      std::stringstream s;
      s << file << ":" << line << ": " << function << "()";
      return s.str();
    }

  };

}// namespace numopt_common

inline std::ostream & operator<<(std::ostream & out, const numopt_common::source_file_pos & sfp)
{
  out << sfp.file << ":" << sfp.line << ": " << sfp.function << "()";
  return out;
}


#define NUMOPT_SOURCE_FILE_POS numopt_common::source_file_pos(__FUNCTION__,__FILE__,__LINE__)
