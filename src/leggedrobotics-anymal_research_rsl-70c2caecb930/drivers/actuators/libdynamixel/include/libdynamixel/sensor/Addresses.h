/** \file Addresses.h
    \brief This file defines the Addresses namespace which contains the
           different Dynamixel addresses.
  */

#ifndef LIBDYNAMIXEL_SENSOR_ADDRESSES_H
#define LIBDYNAMIXEL_SENSOR_ADDRESSES_H

#include <cstdint>

namespace dynamixel {

  /** The namespace Addresses contains the different Dynamixel addresses.
      \brief Dynamixel addresses.
    */
  namespace Addresses {
    /** \name EEPROM addresses
      @{
      */
    /// Lowest byte of model number (R)
    static constexpr uint8_t modelNumberLow = 0x00;
    /// Highest byte of model number (R)
    static constexpr uint8_t modelNumberHigh = 0x01;
    /// Information on the version of firmware (R)
    static constexpr uint8_t firmwareVersion = 0x02;
    /// ID of Dynamixel (RW)
    static constexpr uint8_t id = 0x03;
    /// Baud Rate of Dynamixel (RW)
    static constexpr uint8_t baudRate = 0x04;
    /// Return Delay Time (RW)
    static constexpr uint8_t returnDelayTime = 0x05;
    /// Lowest byte of clockwise Angle Limit (RW)
    static constexpr uint8_t CWAngleLimitLow = 0x06;
    /// Highest byte of clockwise Angle Limit (RW)
    static constexpr uint8_t CWAngleLimitHigh = 0x07;
    /// Lowest byte of counterclockwise Angle Limit (RW)
    static constexpr uint8_t CCWAngleLimitLow = 0x08;
    /// Highest byte of counterclockwise Angle Limit (RW)
    static constexpr uint8_t CCWAngleLimitHigh = 0x09;
    /// Drive Mode (RW): EX-106+, MX-106
    static constexpr uint8_t driveMode = 0x0A;
    /// Internal Limit Temperature (RW)
    static constexpr uint8_t highestLimitTemperature = 0x0B;
    /// Lowest Limit Voltage (RW)
    static constexpr uint8_t lowestLimitVoltage = 0x0C;
    /// Highest Limit Voltage (RW)
    static constexpr uint8_t highestLimitVoltage = 0x0D;
    /// Lowest byte of Max. Torque (RW)
    static constexpr uint8_t maxTorqueLow = 0x0E;
    /// Highest byte of Max. Torque (RW)
    static constexpr uint8_t maxTorqueHigh = 0x0F;
    /// Status Return Level (RW)
    static constexpr uint8_t statusReturnLevel = 0x10;
    /// LED for Alarm (RW)
    static constexpr uint8_t alarmLED = 0x11;
    /// Shutdown for Alarm (RW)
    static constexpr uint8_t alarmShutdown = 0x12;
    /// Multi-turn offset least significant byte (RW): MX series
    static constexpr uint8_t multiTurnOffsetLow = 0x14;
    /// Multi-turn offset most significant byte (RW): MX series
    static constexpr uint8_t multiTurnOffsetHigh = 0x15;
    /// Resolution divider (RW): MX series
    static constexpr uint8_t resolutionDivider = 0x16;
    /** @}
      */

    /** \name RAM addresses
      @{
      */
    /// Torque On/Off (RW)
    static constexpr uint8_t torqueEnable = 0x18;
    /// LED On/Off (RW)
    static constexpr uint8_t led = 0x19;
    /// Derivative Gain (RW): MX series
    static constexpr uint8_t dGain = 0x1A;
    /// Clockwise Compliance Margin (RW): EX/RX/AX/DX series
    static constexpr uint8_t cwComplianceMargin = 0x1A;
    /// Integral Gain (RW): MX series
    static constexpr uint8_t iGain = 0x1B;
    /// Counterclockwise Compliance Margin (RW): EX/RX/AX/DX series
    static constexpr uint8_t ccwComplianceMargin = 0x1B;
    /// Proportional Gain (RW): MX series
    static constexpr uint8_t pGain = 0x1C;
    /// Clockwise Compliance Slope (RW): EX/RX/AX/DX series
    static constexpr uint8_t cwComplianceSlope = 0x1C;
    /// Counterclockwise Compliance Slope (RW): EX/RX/AX/DX series
    static constexpr uint8_t ccwComplianceSlope = 0x1D;
    /// Lowest byte of Goal Position (RW)
    static constexpr uint8_t goalPositionLow = 0x1E;
    /// Highest byte of Goal Position (RW)
    static constexpr uint8_t goalPositionHigh = 0x1F;
    /// Lowest byte of Moving Speed (RW)
    static constexpr uint8_t movingSpeedLow = 0x20;
    /// Highest byte of Moving Speed (RW)
    static constexpr uint8_t movingSpeedHigh = 0x21;
    /// Lowest byte of Torque Limit (RW)
    static constexpr uint8_t torqueLimitLow = 0x22;
    /// Highest byte of Torque Limit (RW)
    static constexpr uint8_t torqueLimitHigh = 0x23;
    /// Lowest byte of Current Position (R)
    static constexpr uint8_t presentPositionLow = 0x24;
    /// Highest byte of Current Position (R)
    static constexpr uint8_t presentPositionHigh = 0x25;
    /// Lowest byte of Current Speed (R)
    static constexpr uint8_t presentSpeedLow = 0x26;
    /// Highest byte of Current Speed (R)
    static constexpr uint8_t presentSpeedHigh = 0x27;
    /// Lowest byte of Current Load (R)
    static constexpr uint8_t presentLoadLow = 0x28;
    /// Highest byte of Current Load (R)
    static constexpr uint8_t presentLoadHigh = 0x29;
    /// Current voltage (R)
    static constexpr uint8_t presentVoltage = 0x2A;
    /// Current temperature (R)
    static constexpr uint8_t presentTemperature = 0x2B;
    /// Means if Instruction is registered (R)
    static constexpr uint8_t registered = 0x2C;
    /// Means if there is any movement (R)
    static constexpr uint8_t moving = 0x2E;
    /// Locking EEPROM (RW)
    static constexpr uint8_t lock = 0x2F;
    /// Lowest byte of Punch (RW)
    static constexpr uint8_t punchLow = 0x30;
    /// Highest byte of Punch (RW)
    static constexpr uint8_t punchHigh = 0x31;
    /// Lowest byte of Consuming Current (R): EX-106+
    static constexpr uint8_t sensedCurrentLow = 0x38;
    /// Highest byte of Consuming Current (R): EX-106
    static constexpr uint8_t sensedCurrentHigh = 0x39;
    /// Lowest byte of Consuming Current (RW): MX-64, MX-106
    static constexpr uint8_t currentLow = 0x44;
    /// Highest byte of Consuming Current (RW): MX-64, MX-106
    static constexpr uint8_t currentHigh = 0x45;
    /// Torque control mode on/off (RW): MX-64, MX-106
    static constexpr uint8_t torqueControlModeEnable = 0x46;
    /// Lowest byte of goal torque value (RW): MX-64, MX-106
    static constexpr uint8_t goalTorqueLow = 0x47;
    /// Highest byte of goal torque value (RW): MX-64, MX-106
    static constexpr uint8_t goalTorqueHigh = 0x48;
    /// Goal Acceleration (RW): MX series
    static constexpr uint8_t goalAcceleration = 0x49;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_SENSOR_ADDRESSES_H
