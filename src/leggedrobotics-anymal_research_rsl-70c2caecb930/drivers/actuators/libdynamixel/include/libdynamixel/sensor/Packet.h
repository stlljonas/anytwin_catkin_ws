/** \file Packet.h
    \brief This file defines the Packet class which represents a packet
           transmitted/received to/from a Dynamixel device.
  */

#ifndef LIBDYNAMIXEL_SENSOR_PACKET_H
#define LIBDYNAMIXEL_SENSOR_PACKET_H

#include <cstdint>

#include <vector>

namespace dynamixel {

  class BinaryReader;
  class BinaryWriter;

  /** The class Packet represents a packet transmitted/received to/from a
      Dynamixel device. The packet format is not compatible with the models
      XL-320 and Dynamixel Pro.
      \brief Dynamixel packet.
    */
  class Packet {
    /// Read stream operator
    friend BinaryReader& operator >> (BinaryReader& stream, Packet& obj);
    /// Write stream operator
    friend BinaryWriter& operator << (BinaryWriter& stream, const Packet& obj);
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Default constructor
    Packet() = default;
    /// Copy constructor
    Packet(const Packet& other) = delete;
    /// Copy assignment operator
    Packet& operator = (const Packet& other) = delete;
    /// Move constructor
    Packet(Packet&& other) = delete;
    /// Move assignment operator
    Packet& operator = (Packet&& other) = delete;
     /// Destructor
    ~Packet() = default;
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns identifier
    uint16_t getIdentifier() const {return identifier_;}
    /// Returns packet id
    uint8_t getId() const {return id_;}
    /// Sets packet id
    void setId(uint8_t id) {id_ = id;}
    /// Returns packet length
    uint8_t getLength() const {return length_;}
    /// Sets packet length
    void setLength(uint8_t length) {length_ = length;}
    /// Returns instruction/error
    uint8_t getInstructionOrError() const {return instructionOrError_;}
    /// Sets instruction/error
    void setInstructionOrError(uint8_t instructionOrError) {
      instructionOrError_ = instructionOrError;}
    /// Returns parameters
    const std::vector<uint8_t>& getParameters() const {return parameters_;}
    /// Returns parameters
    std::vector<uint8_t>& getParameters() {return parameters_;}
    /// Sets parameters
    void setParameters(const std::vector<uint8_t>& parameters) {
      parameters_ = parameters;}
    /// Returns checksum
    uint8_t getChecksum() const {return checksum_;}
    /// Sets checksum
    void setChecksum(uint8_t checksum) {checksum_ = checksum;}
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Returns the computed checksum
    uint8_t computeChecksum() const;
    /** @}
      */

  private:
    /** \name Private methods
      @{
      */
    /// Reads a packet from a stream
    void read(BinaryReader& stream);
    /// Writes a packet from a stream
    void write(BinaryWriter& stream) const;
    /** @}
      */

    /** \name Private members
      @{
      */
    /// Packet identifier
    static constexpr uint16_t identifier_ = 0xFFFF;
    /// Device ID
    uint8_t id_;
    /// Packet length
    uint8_t length_;
    /// Instruction/error
    uint8_t instructionOrError_;
    /// Parameters
    std::vector<uint8_t> parameters_;
    /// Checksum
    uint8_t checksum_;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_SENSOR_PACKET_H
