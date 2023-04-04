
#include <sstream>

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  template <typename X>
  BadArgumentException<X>::BadArgumentException(const X& argument, const
      std::string& msg, const std::string& filename, size_t line, const
      std::string& function) :
      Exception(msg, filename, line, function) {
    std::stringstream stream;
    stream << "[argument = " << argument << "]";
    outputMessage_.append(stream.str());
  }

  template <typename X>
  BadArgumentException<X>::BadArgumentException(const BadArgumentException&
      other) throw() :
      Exception(other) {
  }

  template <typename X>
  BadArgumentException<X>& BadArgumentException<X>::operator =
      (const BadArgumentException& other) throw() {
    if (this != &other) {
      Exception::operator=(other);
    }
    return *this;
  }

}
