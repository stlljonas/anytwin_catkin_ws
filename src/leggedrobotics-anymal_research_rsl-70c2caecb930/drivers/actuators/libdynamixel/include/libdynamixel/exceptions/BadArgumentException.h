
/** \file BadArgumentException.h
    \brief This file defines the BadArgumentException class, which is thrown
           whenever the arguments of a function are invalid.
  */

#ifndef LIBDYNAMIXEL_EXCEPTIONS_BAD_ARGUMENT_EXCEPTION_H
#define LIBDYNAMIXEL_EXCEPTIONS_BAD_ARGUMENT_EXCEPTION_H

#include <cstddef>

#include <string>

#include "libdynamixel/exceptions/Exception.h"

namespace dynamixel {

  /** The class BadArgumentException represents any exceptions occuring when the
      arguments passed to a function are invalid.
      \brief Bad argument exception
    */
  template <typename X> class BadArgumentException :
    public Exception {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructs exception from argument and string
    BadArgumentException(const X& argument, const std::string& msg, const
      std::string& filename = " ", size_t line = 0, const std::string&
      function = " ");
    /// Copy constructor
    BadArgumentException(const BadArgumentException& other) throw();
    /// Assignment operator
    BadArgumentException& operator = (const BadArgumentException& other)
      throw();
    /// Destructor
    virtual ~BadArgumentException() throw() = default;
    /** @}
      */

  protected:

  };

}

#include "libdynamixel/exceptions/BadArgumentException.tpp"

#endif // LIBDYNAMIXEL_EXCEPTIONS_BAD_ARGUMENT_EXCEPTION_H
