#include "libdynamixel/com/BinaryReader.h"

namespace dynamixel {

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  BinaryReader& BinaryReader::operator >> (int8_t& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (uint8_t& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (int16_t& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (uint16_t& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (int32_t& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (uint32_t& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (int64_t& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (uint64_t& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (float& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

  BinaryReader& BinaryReader::operator >> (double& value) {
    read(reinterpret_cast<char*>(&value), sizeof(value));
    return *this;
  }

}
