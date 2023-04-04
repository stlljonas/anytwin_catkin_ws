/** \file SystemException.h
    \brief This file defines the SystemException class, which represents
           low-level system exceptions.
  */

#ifndef LIBDYNAMIXEL_EXCEPTIONS_SYSTEM_EXCEPTION_H
#define LIBDYNAMIXEL_EXCEPTIONS_SYSTEM_EXCEPTION_H

#include <cstddef>

#include <string>

#include "libdynamixel/exceptions/Exception.h"

namespace dynamixel {

  /** The class SystemException represents system exceptions.
      \brief System exceptions
    */
  class SystemException :
    public Exception {
  public:
    /** \name Constructors/Destructor
      @{
      */
    /// Constructs exception
    SystemException(int errNo, const std::string& msg = "", const std::string&
      filename = " ", size_t line = 0, const std::string& function = " ");
    /// Copy constructor
    SystemException(const SystemException& other) throw ();
    /// Assignment operator
    SystemException& operator = (const SystemException& other) throw();
    /// Destructor
    virtual ~SystemException() throw () = default;
    /** @}
      */

  protected:

  };

}

#endif // LIBDYNAMIXEL_EXCEPTIONS_SYSTEM_EXCEPTION_H
