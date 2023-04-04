#include "libdynamixel/exceptions/SystemException.h"

#include <cstring>

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  SystemException::SystemException(int errNo, const std::string& msg, const
      std::string& filename, size_t line, const std::string& function) :
      Exception(msg + ": " + std::string(strerror(errNo)), filename, line,
      function) {
  }

  SystemException::SystemException(const SystemException& other) throw() :
      Exception(other) {
  }

  SystemException& SystemException::operator =
      (const SystemException& other) throw() {
    if (this != &other) {
      Exception::operator=(other);
    }
    return *this;
  }

}
