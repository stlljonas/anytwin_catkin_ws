#include "libdynamixel/exceptions/IOException.h"

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  IOException::IOException(const std::string& msg, const std::string& filename,
      size_t line, const std::string& function) :
      Exception(msg, filename, line, function) {
  }

  IOException::IOException(const IOException& other) throw() :
      Exception(other) {
  }

  IOException& IOException::operator = (const IOException& other) throw() {
    if (this != &other) {
      Exception::operator=(other);
    }
    return *this;
  }

}
