#pragma once


// c++
#include <iostream>

// message logger
#include <message_logger/message_logger.hpp>


namespace conditional_state_machine {


class Macros
{
public:
  static inline std::string extractFunctionName(const std::string& prettyFunction)
  {
    std::string functionName = prettyFunction;

    // remove function arguments
    const size_t argsBegin = functionName.find_first_of("(");
    functionName = functionName.substr(0, argsBegin);

    // remove template parameters
    while (functionName.find_last_of("<") != functionName.npos)
    {
      const size_t tempBegin = functionName.find_last_of("<");
      const size_t tempLenth = functionName.substr(tempBegin, functionName.npos).find_first_of(">") + 1;
      functionName = functionName.substr(0, tempBegin) +
                     functionName.substr(tempBegin + tempLenth, functionName.npos);
    }

    // remove return values
    const size_t classBegin = functionName.find_last_of(" ") + 1;
    return functionName.substr(classBegin, functionName.npos);
  }
};


} // conditional_state_machine


#define _CONDITIONAL_STATE_MACHINE_ADD_FUNCTION_NAME(message) "[" << conditional_state_machine::Macros::extractFunctionName(__PRETTY_FUNCTION__) << "] " << message

#define CONDITIONAL_STATE_MACHINE_DEBUG(message) {MELO_DEBUG_STREAM(_CONDITIONAL_STATE_MACHINE_ADD_FUNCTION_NAME(message));}
#define CONDITIONAL_STATE_MACHINE_INFO(message)  {MELO_INFO_STREAM(_CONDITIONAL_STATE_MACHINE_ADD_FUNCTION_NAME(message));}
#define CONDITIONAL_STATE_MACHINE_WARN(message)  {MELO_WARN_STREAM(_CONDITIONAL_STATE_MACHINE_ADD_FUNCTION_NAME(message));}
#define CONDITIONAL_STATE_MACHINE_ERROR(message) {MELO_ERROR_STREAM(_CONDITIONAL_STATE_MACHINE_ADD_FUNCTION_NAME(message));}
#define CONDITIONAL_STATE_MACHINE_FATAL(message) {MELO_FATAL_STREAM(_CONDITIONAL_STATE_MACHINE_ADD_FUNCTION_NAME(message));}
