#include "libdynamixel/exceptions/Exception.h"

#include <sstream>

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  Exception::Exception(const std::string& msg, const std::string& filename,
      size_t line, const std::string& function) :
      msg_(msg),
      filename_(filename),
      function_(function),
      line_(line) {
    std::stringstream stream;
    if (function != " ")
      stream << function << ": ";
    stream << msg;
    if (filename != " ")
      stream << " [file = " << filename << "]";
    if (line)
      stream << "[line = " << line << "]";
    outputMessage_ = stream.str();
  }

  Exception::Exception(const Exception& other) throw() :
      msg_(other.msg_),
      filename_(other.filename_),
      function_(other.function_),
      line_(other.line_),
      outputMessage_(other.outputMessage_) {
  }

  Exception& Exception::operator = (const Exception& other) throw() {
    if (this != &other) {
      msg_ = other.msg_;
      filename_ = other.filename_;
      function_ = other.function_;
      line_ = other.line_;
      outputMessage_ = other.outputMessage_;
    }
    return *this;
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  const char* Exception::what() const throw() {
    return outputMessage_.c_str();
  }

}
