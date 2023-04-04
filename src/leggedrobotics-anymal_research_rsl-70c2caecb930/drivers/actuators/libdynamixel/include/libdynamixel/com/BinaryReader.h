
/** \file BinaryReader.h
    \brief This file defines the BinaryReader class which is an interface
           for reading basic types from a binary stream.
  */

#ifndef LIBDYNAMIXEL_COM_BINARY_READER_H
#define LIBDYNAMIXEL_COM_BINARY_READER_H

#include <cstdint>
#include <cstdlib>

namespace dynamixel {

  /** The BinaryReader class is an interface for reading basic types from a
      binary stream.
      \brief Binary reader
    */
  class BinaryReader {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Default constructor
    BinaryReader() = default;
    /// Copy constructor
    BinaryReader(const BinaryReader& other) = delete;
    /// Copy assignment operator
    BinaryReader& operator = (const BinaryReader& other) = delete;
    /// Move constructor
    BinaryReader(BinaryReader&& other) = delete;
    /// Move assignment operator
    BinaryReader& operator = (BinaryReader&& other) = delete;
    /// Destructor
    virtual ~BinaryReader() = default;
    /** @}
      */

    /** \name Operators
      @{
      */
    /// Reads 8-bit signed integer
    BinaryReader& operator >> (int8_t& value);
    /// Reads 8-bit unsigned integer
    BinaryReader& operator >> (uint8_t& value);
    /// Reads 16-bit signed integer
    BinaryReader& operator >> (int16_t& value);
    /// Reads 16-bit unsigned integer
    BinaryReader& operator >> (uint16_t& value);
    /// Reads 32-bit signed integer
    BinaryReader& operator >> (int32_t& value);
    /// Reads 32-bit unsigned integer
    BinaryReader& operator >> (uint32_t& value);
    /// Reads 64-bit signed integer
    BinaryReader& operator >> (int64_t& value);
    /// Reads 64-bit unsigned integer
    BinaryReader& operator >> (uint64_t& value);
    /// Reads 32-bit floating point
    BinaryReader& operator >> (float& value);
    /// Reads 64-bit floating point
    BinaryReader& operator >> (double& value);
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Perform read on the stream
    virtual void read(char* buffer, size_t numBytes) = 0;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_COM_BINARY_READER_H
