/** \file IOException.h
    \brief This file defines the IOException class, which represents
           input/output exceptions.
  */

#ifndef LIBDYNAMIXEL_EXCEPTIONS_IO_EXCEPTION_H
#define LIBDYNAMIXEL_EXCEPTIONS_IO_EXCEPTION_H

#include <cstddef>

#include <string>

#include "libdynamixel/exceptions/Exception.h"

namespace dynamixel {

  /** The class IOException represents input/output exceptions.
      \brief Input/output exceptions
    */
  class IOException :
    public Exception {
  public:
    /** \name Constructors/Destructor
      @{
      */
    /// Constructs exception
    IOException(const std::string& msg = "", const std::string&
      filename = " ", size_t line = 0, const std::string& function = " ");
    /// Copy constructor
    IOException(const IOException& other) throw ();
    /// Assignment operator
    IOException& operator = (const IOException& other) throw();
    /// Destructor
    virtual ~IOException() throw () = default;
    /** @}
      */

  protected:

  };

}

#endif // LIBDYNAMIXEL_EXCEPTIONS_IO_EXCEPTION_H
