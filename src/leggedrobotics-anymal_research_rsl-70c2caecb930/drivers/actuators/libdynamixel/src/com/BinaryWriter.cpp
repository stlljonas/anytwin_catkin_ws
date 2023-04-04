#include "libdynamixel/com/BinaryWriter.h"

namespace dynamixel {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  BinaryWriter& BinaryWriter::operator << (int8_t value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (uint8_t value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (int16_t value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (uint16_t value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (int32_t value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (uint32_t value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (int64_t value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (uint64_t value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (float value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

  BinaryWriter& BinaryWriter::operator << (double value) {
    write(reinterpret_cast<const char*>(&value), sizeof(value));
    return *this;
  }

}
