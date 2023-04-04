/**
 * @file   numopt_common_assert_macros.hpp
 * @author Paul Furgale <paul.furgale@gmail.com>, Christian Gehring
 * @date   Mon Dec 12 11:22:20 2011
 *
 * @brief  Assert macros to facilitate rapid prototyping. Use them early and often.
 *
 *
 */

#pragma once

#include <stdexcept>
#include <sstream>
#include <typeinfo>
#include "numopt_source_file_pos.hpp"

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define NUMOPT_DEFINE_EXCEPTION(exceptionName, exceptionParent)       \
  class exceptionName : public exceptionParent {            \
  public:                               \
  exceptionName(const char * message) : exceptionParent(message) {}   \
  exceptionName(std::string const & message) : exceptionParent(message) {} \
  virtual ~exceptionName() throw() {}                 \
  };


namespace numopt_common {
namespace common {
  namespace internal {

    template<typename NUMOPT_EXCEPTION_T>
    inline void NUMOPT_throw_exception(std::string const & exceptionType, numopt_common::source_file_pos sfp, std::string const & message)
    {
      std::stringstream NUMOPT_assert_stringstream;
#ifdef _WIN32
      // I have no idea what broke this on Windows but it doesn't work with the << operator.
      NUMOPT_assert_stringstream << exceptionType <<  sfp.toString() << " " << message;
#else
      NUMOPT_assert_stringstream << exceptionType <<  sfp << " " << message;
#endif
      throw(NUMOPT_EXCEPTION_T(NUMOPT_assert_stringstream.str()));
    }

    template<typename NUMOPT_EXCEPTION_T>
    inline void NUMOPT_throw_exception(std::string const & exceptionType, std::string const & function, std::string const & file,
                   int line, std::string const & message)
    {
      NUMOPT_throw_exception<NUMOPT_EXCEPTION_T>(exceptionType, numopt_common::source_file_pos(function,file,line),message);
    }


  } // namespace internal

  template<typename NUMOPT_EXCEPTION_T>
  inline void NUMOPT_assert_throw(bool assert_condition, std::string message, numopt_common::source_file_pos sfp) {
    if(!assert_condition)
      {
    internal::NUMOPT_throw_exception<NUMOPT_EXCEPTION_T>("", sfp,message);
      }
  }


} // namespace common
} // namespace rm




#define NUMOPT_THROW(exceptionType, message) {                \
    std::stringstream NUMOPT_assert_stringstream;             \
    NUMOPT_assert_stringstream << message;                  \
    numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, NUMOPT_assert_stringstream.str()); \
  }


#define NUMOPT_THROW_SFP(exceptionType, SourceFilePos, message){      \
    std::stringstream NUMOPT_assert_stringstream;             \
    NUMOPT_assert_stringstream << message;                  \
    numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", SourceFilePos, NUMOPT_assert_stringstream.str()); \
  }

#define NUMOPT_ASSERT_TRUE(exceptionType, condition, message)       \
  if(!(condition))                            \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #condition << ") failed: " << message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, NUMOPT_assert_stringstream.str()); \
    }

#define NUMOPT_ASSERT_FALSE(exceptionType, condition, message)        \
  if((condition))                           \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert( not " << #condition << ") failed: " << message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_GE_LT(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_LT(exceptionType, value, upperBound, message)     \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }

#define NUMOPT_ASSERT_GE(exceptionType, value, lowerBound, message)     \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_LE(exceptionType, value, upperBound, message)     \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }

#define NUMOPT_ASSERT_GT(exceptionType, value, lowerBound, message)     \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_EQ(exceptionType, value, testValue, message)      \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }

#define NUMOPT_ASSERT_NE(exceptionType, value, testValue, message)      \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }

#define NUMOPT_ASSERT_NEAR(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#ifndef NDEBUG

#define NUMOPT_THROW_DBG(exceptionType, message){             \
    std::stringstream NUMOPT_assert_stringstream;             \
    NUMOPT_assert_stringstream << message;                  \
    numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, NUMOPT_assert_stringstream.str()); \
  }



#define NUMOPT_ASSERT_TRUE_DBG(exceptionType, condition, message)     \
  if(!(condition))                            \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #condition << ") failed: " << message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, NUMOPT_assert_stringstream.str()); \
    }

#define NUMOPT_ASSERT_FALSE_DBG(exceptionType, condition, message)      \
  if((condition))                           \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert( not " << #condition << ") failed: " << message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__, NUMOPT_assert_stringstream.str()); \
    }


#define NUMOPT_ASSERT_DBG_RE( condition, message) NUMOPT_ASSERT_DBG(std::runtime_error, condition, message)

#define NUMOPT_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message) \
  if((value) < (lowerBound) || (value) >= (upperBound))             \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #lowerBound << " <= " << #value << " < " << #upperBound << ") failed [" << (lowerBound) << " <= " << (value) << " < " << (upperBound) << "]: " << message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_LT_DBG(exceptionType, value, upperBound, message)   \
  if((value) >= (upperBound))                       \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #value << " < " << #upperBound << ") failed [" << (value) << " < " << (upperBound) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)   \
  if((value) < (lowerBound))                        \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #value << " >= " << #lowerBound << ") failed [" << (value) << " >= " << (lowerBound) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_LE_DBG(exceptionType, value, upperBound, message)   \
  if((value) > (upperBound))                        \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #value << " <= " << #upperBound << ") failed [" << (value) << " <= " << (upperBound) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }

#define NUMOPT_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)   \
  if((value) <= (lowerBound))                       \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #value << " > " << #lowerBound << ") failed [" << (value) << " > " << (lowerBound) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_EQ_DBG(exceptionType, value, testValue, message)    \
  if((value) != (testValue))                        \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }


#define NUMOPT_ASSERT_NE_DBG(exceptionType, value, testValue, message)    \
  if((value) == (testValue))                        \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #value << " != " << #testValue << ") failed [" << (value) << " != " << (testValue) << "]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }



#define NUMOPT_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message) \
  if(!(fabs((testValue) - (value)) <= fabs(abs_error)))           \
    {                                 \
      std::stringstream NUMOPT_assert_stringstream;             \
      NUMOPT_assert_stringstream << "debug assert(" << #value << " == " << #testValue << ") failed [" << (value) << " == " << (testValue) << " (" << fabs((testValue) - (value)) << " > " << fabs(abs_error) << ")]: " <<  message; \
      numopt_common::common::internal::NUMOPT_throw_exception<exceptionType>("[" #exceptionType "] ", __FUNCTION__,__FILE__,__LINE__,NUMOPT_assert_stringstream.str()); \
    }


#define NUMOPT_OUT(X) std::cout << #X << ": " << (X) << std::endl

#else

#define NUMOPT_OUT(X)
#define NUMOPT_THROW_DBG(exceptionType, message)
#define NUMOPT_ASSERT_TRUE_DBG(exceptionType, condition, message)
#define NUMOPT_ASSERT_FALSE_DBG(exceptionType, condition, message)
#define NUMOPT_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, message)
#define NUMOPT_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define NUMOPT_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define NUMOPT_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define NUMOPT_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define NUMOPT_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define NUMOPT_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define NUMOPT_ASSERT_NEAR_DBG(exceptionType, value, testValue, abs_error, message)
#endif
