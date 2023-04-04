/** \file BinaryWriter.h
    \brief This file defines the BinaryWriter class which is an interface
           for writing basic types to a binary stream.
  */

#ifndef LIBDYNAMIXEL_COM_BINARY_WRITER_H
#define LIBDYNAMIXEL_COM_BINARY_WRITER_H

#include <cstdint>
#include <cstdlib>

namespace dynamixel {

  /** The BinaryWriter class is an interface for writing basic types to a binary
      stream.
      \brief Binary writer
    */
  class BinaryWriter {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Default constructor
    BinaryWriter() = default;
    /// Copy constructor
    BinaryWriter(const BinaryWriter& other) = delete;
    /// Copy assignment operator
    BinaryWriter& operator = (const BinaryWriter& other) = delete;
    /// Move constructor
    BinaryWriter(BinaryWriter&& other) = delete;
    /// Move assignment operator
    BinaryWriter& operator = (BinaryWriter&& other) = delete;
    /// Destructor
    virtual ~BinaryWriter() = default;
    /** @}
      */

    /** \name Operators
      @{
      */
    /// Writes 8-bit signed integer
    BinaryWriter& operator << (int8_t value);
    /// Writes 8-bit unsigned integer
    BinaryWriter& operator << (uint8_t value);
    /// Writes 16-bit signed integer
    BinaryWriter& operator << (int16_t value);
    /// Writes 16-bit unsigned integer
    BinaryWriter& operator << (uint16_t value);
    /// Writes 32-bit signed integer
    BinaryWriter& operator << (int32_t value);
    /// Writes 32-bit unsigned integer
    BinaryWriter& operator << (uint32_t value);
    /// Writes 64-bit signed integer
    BinaryWriter& operator << (int64_t value);
    /// Writes 64-bit unsigned integer
    BinaryWriter& operator << (uint64_t value);
    /// Writes 32-bit floating point
    BinaryWriter& operator << (float value);
    /// Writes 64-bit floating point
    BinaryWriter& operator << (double value);
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Performs write on the stream
    virtual void write(const char* buffer, size_t numBytes) = 0;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_COM_BINARY_WRITER_H
