/** \file Instructions.h
    \brief This file defines the Instructions namespace which contains the
           different instruction codes.
  */

#ifndef LIBDYNAMIXEL_SENSOR_INSTRUCTIONS_H
#define LIBDYNAMIXEL_SENSOR_INSTRUCTIONS_H

#include <cstdint>

namespace dynamixel {

  /** The namespace Instructions contains the different instruction codes.
      \brief Instruction codes.
    */
  namespace Instructions {
    /** \name Public members
      @{
      */
    /// Ping instruction
    static constexpr uint8_t PING = 0x01;
    /// Read data instruction
    static constexpr uint8_t READ_DATA = 0x02;
    /// Write data instruction
    static constexpr uint8_t WRITE_DATA = 0x03;
    /// Register write instruction
    static constexpr uint8_t REG_WRITE = 0x04;
    /// Action instruction
    static constexpr uint8_t ACTION = 0x05;
    /// Reset instruction
    static constexpr uint8_t RESET = 0x06;
    /// Synchronous write instruction
    static constexpr uint8_t SYNC_WRITE = 0x07;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_SENSOR_INSTRUCTIONS_H
