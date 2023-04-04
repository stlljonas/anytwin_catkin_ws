/** \file Controller.h
    \brief This file defines the Controller class which abstracts a
           Dynamixel controller.
  */

#ifndef LIBDYNAMIXEL_SENSOR_CONTROLLER_H
#define LIBDYNAMIXEL_SENSOR_CONTROLLER_H

#include <cstdint>
#include <cmath>

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>

#include "libdynamixel/com/BinaryReader.h"
#include "libdynamixel/com/BinaryWriter.h"
#include "libdynamixel/sensor/Models.h"

namespace dynamixel {

  class SerialPort;
  class Packet;

  /** The class Controller represents a Dynamixel controller.
      \brief Dynamixel controller.
    */
  class Controller :
    public BinaryReader, BinaryWriter {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructs the Dynamixel controller with its associated serial port
    Controller(const std::shared_ptr<SerialPort>& serialPort);
    /// Copy constructor
    Controller(const Controller& other) = delete;
    /// Copy assignment operator
    Controller& operator = (const Controller& other) = delete;
    /// Move constructor
    Controller(Controller&& other) = delete;
    /// Move assignment operator
    Controller& operator = (Controller&& other) = delete;
     /// Destructor
    virtual ~Controller() = default;
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Return the serial port
    const std::shared_ptr<SerialPort>& getSerialPort() {return serialPort_;}
    /** @}
      */

    /** \name Low-level protocol
      @{
      */
    /// Ping command
    bool ping(uint8_t id);
    /// Reset to factory settings
    bool reset(uint8_t id);
    /// Action command
    bool action(uint8_t id);
    /// Synchronous writes data at the given address
    std::shared_ptr<Packet> syncWriteData(uint8_t address, const
      std::unordered_map<uint8_t, std::vector<uint8_t> >& data);
    /// Writes data at the given address
    std::shared_ptr<Packet> writeData(uint8_t id, uint8_t address, const
      std::vector<uint8_t>& data);
    /// Register writes data at the given address
    std::shared_ptr<Packet> regWriteData(uint8_t id, uint8_t address, const
      std::vector<uint8_t>& data);
    /// Reads data at the given address
    std::shared_ptr<Packet> readData(uint8_t id, uint8_t address, uint8_t
      numBytes);
    /// Returns error strings
    std::string getErrorString(uint8_t errorCode) const;
    /** @}
      */

    /** \name Convenience methods applicable to all servo motors
      @{
      */
    /// Returns the model number
    uint16_t getModelNumber(uint8_t id);
    /// Returns the firmware version
    uint8_t getFirmwareVersion(uint8_t id);
    /// Returns the Id
    uint8_t getId(uint8_t id);
    /// Sets the Id
    void setId(uint8_t id, uint8_t newId, bool registered = false);
    /// Returns the baud rate
    uint8_t getBaudRate(uint8_t id);
    /// Sets the baud rate
    void setBaudRate(uint8_t id, uint8_t baudRate, bool registered = false);
    /// Returns the return delay time
    uint8_t getReturnDelayTime(uint8_t id);
    /// Sets the return delay time
    void setReturnDelayTime(uint8_t id, uint8_t returnDelayTime, bool
      registered = false);
    /// Returns the clockwise angle limit
    uint16_t getCwAngleLimit(uint8_t id);
    /// Sets the clockwise angle limit
    void setCwAngleLimit(uint8_t id, uint16_t cwAngleLimit, bool
      registered = false);
    /// Returns the counter-clockwise angle limit
    uint16_t getCcwAngleLimit(uint8_t id);
    /// Sets the counter-clockwise angle limit
    void setCcwAngleLimit(uint8_t id, uint16_t ccwAngleLimit, bool
      registered = false);
    /// Returns the angle limits
    void getAngleLimits(uint8_t id, uint16_t& cwAngleLimit, uint16_t&
      ccwAngleLimit);
    /// Sets the angle limits
    void setAngleLimits(uint8_t id, uint16_t cwAngleLimit, uint16_t
      ccwAngleLimit, bool registered = false);
    /// Returns the highest limit temperature
    uint8_t getHighestLimitTemperature(uint8_t id);
    /// Sets the highest limit temperature (use with caution!)
    void setHighestLimitTemperature(uint8_t id, uint8_t temperature, bool
      registered = false);
    /// Returns the highest limit voltage
    uint8_t getHighestLimitVoltage(uint8_t id);
    /// Sets the highest limit voltage
    void setHighestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered = false);
    /// Returns the lowest limit voltage
    uint8_t getLowestLimitVoltage(uint8_t id);
    /// Sets the lowest limit voltage
    void setLowestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered = false);
    /// Returns the maximum torque
    uint16_t getMaxTorque(uint8_t id);
    /// Sets the maximum torque
    void setMaxTorque(uint8_t id, uint16_t torque, bool registered = false);
    /// Returns the status return level
    uint8_t getStatusReturnLevel(uint8_t id);
    /// Sets the status return level
    void setStatusReturnLevel(uint8_t id, uint8_t level, bool
      registered = false);
    /// Returns the alarm LED
    uint8_t getAlarmLed(uint8_t id);
    /// Sets the alarm LED
    void setAlarmLed(uint8_t id, uint8_t code, bool registered = false);
    /// Returns the alarm shutdown
    uint8_t getAlarmShutdown(uint8_t id);
    /// Sets the alarm shutdown
    void setAlarmShutdown(uint8_t id, uint8_t code, bool registered = false);
    /// Returns torque enable status
    bool isTorqueEnable(uint8_t id);
    /// Sets torque enable status
    void setTorqueEnable(uint8_t id, bool enable, bool registered = false);
    /// Returns LED status
    bool isLed(uint8_t id);
    /// Sets LED status
    void setLed(uint8_t id, bool enable, bool registered = false);
    /// Returns the goal position
    uint16_t getGoalPosition(uint8_t id);
    /// Sets the goal position
    void setGoalPosition(uint8_t id, uint16_t position, bool
      registered = false);
    /// Returns the moving speed
    uint16_t getMovingSpeed(uint8_t id);
    /// Sets the moving speed
    void setMovingSpeed(uint8_t id, uint16_t speed, bool registered = false);
    /// Returns the goal position and the moving speed
    void getGoalPositionSpeed(uint8_t id, uint16_t& position, uint16_t&
      speed);
    /// Sets the goal position and the moving speed
    void setGoalPositionSpeed(uint8_t id, uint16_t position, uint16_t speed,
      bool registered = false);
    /// Returns the torque limit
    uint16_t getTorqueLimit(uint8_t id);
    /// Sets the torque limit
    void setTorqueLimit(uint8_t id, uint16_t torque, bool registered = false);
    /// Returns the goal position, the moving speed, and the torque limit
    void getGoalPositionSpeedTorque(uint8_t id, uint16_t& position, uint16_t&
      speed, uint16_t& torque);
    /// Sets the goal position, the moving speed, and the torque limit
    void setGoalPositionSpeedTorque(uint8_t id, uint16_t position, uint16_t
      speed, uint16_t torque, bool registered = false);
    /// Returns the present position
    uint16_t getPresentPosition(uint8_t id);
    /// Returns the present speed
    uint16_t getPresentSpeed(uint8_t id);
    /// Returns the present load
    uint16_t getPresentLoad(uint8_t id);
    /// Returns the present position, speed, and load
    void getPresentPositionSpeedLoad(uint8_t id, uint16_t& position, uint16_t&
      speed, uint16_t& load);
    /// Returns the present voltage
    uint8_t getPresentVoltage(uint8_t id);
    /// Returns the present temperature
    uint8_t getPresentTemperature(uint8_t id);
    /// Returns registered instruction status
    bool isInstructionRegistered(uint8_t id);
    /// Returns moving status
    bool isMoving(uint8_t id);
    /// Returns the state
    void getState(uint8_t id, uint16_t& position, uint16_t& speed, uint16_t&
      load, uint8_t& voltage, uint8_t& temperature, bool& registered, bool&
      moving);
    /// Returns EEPROM lock status
    bool isEEPROMLock(uint8_t id);
    /// Sets EEPROM lock status
    void setEEPROMLock(uint8_t id, bool enable, bool registered = false);
    /// Returns the punch
    uint16_t getPunch(uint8_t id);
    /// Sets the punch
    void setPunch(uint8_t id, uint16_t punch, bool registered = false);
    /** @}
      */

    /** \name Convenience methods specific to MX series
      @{
      */
    /// Returns the multi-turn offset
    uint16_t getMultiTurnOffset(uint8_t id);
    /// Sets the multi-turn offset
    void setMultiTurnOffset(uint8_t id, uint16_t offset, bool
      registered = false);
    /// Returns the resolution divider
    uint8_t getResolutionDivider(uint8_t id);
    /// Sets the resolution divider
    void setResolutionDivider(uint8_t id, uint8_t divider, bool
      registered = false);
    /// Returns the D gain
    uint8_t getDGain(uint8_t id);
    /// Sets the D gain
    void setDGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the I gain
    uint8_t getIGain(uint8_t id);
    /// Sets the I gain
    void setIGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the P gain
    uint8_t getPGain(uint8_t id);
    /// Sets the P gain
    void setPGain(uint8_t id, uint8_t gain, bool registered = false);
    /// Returns the PID gains
    void getPIDGains(uint8_t id, uint8_t& pGain, uint8_t& iGain,
      uint8_t& dGain);
    /// Sets the PID gains
    void setPIDGains(uint8_t id, uint8_t pGain, uint8_t iGain, uint8_t dGain,
      bool registered = false);
    /// Returns the goal acceleration
    uint8_t getGoalAcceleration(uint8_t id);
    /// Sets the goal acceleration
    void setGoalAcceleration(uint8_t id, uint8_t acceleration, bool
      registered = false);
    /** @}
      */

    /** \name Convenience methods specific to MX-64 and MX-106
      @{
      */
    /// Returns the consuming current
    uint16_t getCurrent(uint8_t id);
    /// Returns torque control mode enable status
    bool isTorqueControlModeEnable(uint8_t id);
    /// Sets torque control mode enable status
    void setTorqueControlModeEnable(uint8_t id, bool enable, bool
      registered = false);
    /// Returns the goal torque
    uint16_t getGoalTorque(uint8_t id);
    /// Sets the goal torque
    void setGoalTorque(uint8_t id, uint16_t torque, bool registered = false);
    /** @}
      */

    /** \name Convenience methods specific to EX/RX/AX/DX series
      @{
      */
    /// Returns the clockwise compliance margin
    uint8_t getCwComplianceMargin(uint8_t id);
    /// Sets the clockwise compliance margin
    void setCwComplianceMargin(uint8_t id, uint8_t margin, bool
      registered = false);
    /// Returns the counterclockwise compliance margin
    uint8_t getCcwComplianceMargin(uint8_t id);
    /// Sets the counterclockwise compliance margin
    void setCcwComplianceMargin(uint8_t id, uint8_t margin, bool
      registered = false);
    /// Returns the clockwise compliance slope
    uint8_t getCwComplianceSlope(uint8_t id);
    /// Sets the clockwise compliance slope
    void setCwComplianceSlope(uint8_t id, uint8_t slope, bool
      registered = false);
    /// Returns the counterclockwise compliance slope
    uint8_t getCcwComplianceSlope(uint8_t id);
    /// Sets the counterclockwise compliance slope
    void setCcwComplianceSlope(uint8_t id, uint8_t slope, bool
      registered = false);
    /// Returns the compliance parameters
    void getCompliance(uint8_t id, uint8_t& cwComplianceMargin, uint8_t&
      ccwComplianceMargin, uint8_t& cwComplianceSlope, uint8_t&
      ccwComplianceSlope);
    /// Sets the compliance parameters
    void setCompliance(uint8_t id, uint8_t cwComplianceMargin, uint8_t
      ccwComplianceMargin, uint8_t cwComplianceSlope, uint8_t
      ccwComplianceSlope, bool registered = false);
    /** @}
      */

    /** \name Convenience methods specific to EX-106+
      @{
      */
    /// Returns the sensed current
    uint16_t getSensedCurrent(uint8_t id);
    /** @}
      */

    /** \name Convenience methods specific to EX-106+ and MX-106
      @{
      */
    /// Returns the drive mode
    uint8_t getDriveMode(uint8_t id);
    /// Sets the drive mode
    void setDriveMode(uint8_t id, uint8_t mode, bool registered = false);
    /** @}
      */

    /** \name Helper methods
      @{
      */
    /// Converts revolutions per minute to radians per second
    static double rpm2rps(double rpm) {
      return rpm / 60.0 * 2 * M_PI;
    }
    /// Converts radians per second to revolutions per minute
    static double rps2rpm(double rps) {
      return rps * 30 / M_PI;
    }
    /// Converts degree to radian
    static double deg2rad(double deg) {
      return deg * M_PI / 180.0;
    }
    /// Converts radian to degree
    static double rad2deg(double rad) {
      return rad * 180.0 / M_PI;
    }
    /// Converts angle to interval [0, 2*PI)
    static double wrapAngle(double angle) {
      while (angle < 0)
        angle += 2*M_PI;
      while (angle >= 2*M_PI)
        angle -= 2*M_PI;
      return angle;
    }
    /// Converts tick to angle
    static double tick2angle(uint16_t tick, double range = 2 * M_PI, uint16_t
        maxTicks = 4095) {
      return tick / static_cast<double>(maxTicks) * range;
    }
    /// Converts angle to tick
    static uint16_t angle2tick(double angle, double range = 2 * M_PI, uint16_t
        maxTicks = 4095) {
      return std::round(angle / range * maxTicks);
    }
    /// Converts raw time to microsecond time
    static uint16_t raw2us(uint8_t rawTime, uint8_t resolution = 2) {
      return static_cast<uint16_t>(rawTime) * resolution;
    }
    /// Converts microsecond time to raw time
    static uint8_t us2raw(uint16_t usTime, uint8_t resolution = 2) {
      return static_cast<uint8_t>(usTime / resolution);
    }
    /// Converts raw voltage to volt
    static double raw2volt(uint8_t rawVoltage, double resolution = 0.1) {
      return rawVoltage * resolution;
    }
    /// Converts volt to raw voltage
    static uint8_t volt2raw(double voltage, double resolution = 0.1) {
      return std::round(voltage / resolution);
    }
    /// Converts raw torque to torque ratio
    static double raw2torqueRatio(uint16_t rawTorque, uint16_t
        maxValue = 1023) {
      return rawTorque & 0x0400 ? (rawTorque & 0x03FF) /
        static_cast<double>(maxValue) * (-1.0) :
        (rawTorque & 0x03FF) / static_cast<double>(maxValue);
    }
    /// Converts torque ratio to raw torque
    static uint16_t torqueRatio2raw(double torqueRatio, uint16_t
        maxValue = 1023) {
      return torqueRatio > 0 ? std::round(torqueRatio * maxValue) :
        static_cast<uint16_t>(std::round(std::fabs(torqueRatio) * maxValue)) |
        0x0400;
    }
    /// Converts raw speed to revolutions per minute
    static double raw2rpm(uint16_t rawSpeed, double rpmPerTick = 0.114) {
      return rawSpeed & 0x0400 ? (rawSpeed & 0x03FF) * rpmPerTick * (-1.0) :
        (rawSpeed & 0x03FF)* rpmPerTick;
    }
    /// Converts revolutions per minute to raw speed
    static uint16_t rpm2raw(double rpm, double rpmPerTick = 0.114) {
      return rpm > 0 ? std::round(rpm / rpmPerTick) :
        static_cast<uint16_t>(std::round(std::fabs(rpm) / rpmPerTick)) | 0x0400;
    }
    /// Converts raw acceleration to radians per second^2
    static double raw2rps2(uint8_t rawAcceleration, uint8_t maxValue = 254,
        double maxAcc = 2180.0) {
      return rawAcceleration / static_cast<double>(maxValue) * deg2rad(maxAcc);
    }
    /// Converts radians per second^2 to raw acceleration
    static uint8_t rps22raw(double rps2, uint8_t maxValue = 254,
        double maxAcc = 2180.0) {
      return std::round(rps2 * maxValue / deg2rad(maxAcc));
    }
    /// Converts raw current to amper
    static double raw2amp(uint16_t rawCurrent, uint16_t valueIdle = 2048, double
        resolution = 0.0045) {
      return resolution * (rawCurrent - valueIdle);
    }
    /// Converts amper to raw current
    static uint16_t amp2raw(double current, uint16_t valueIdle = 2048, double
        resolution = 0.0045) {
      return std::round(current / resolution + valueIdle);
    }
    /// Converts raw torque to torque in amper
    static double rawtorque2amp(uint16_t rawTorque, double
        resolution = 0.0045) {
      return rawTorque & 0x0400 ? (rawTorque & 0x03FF) * resolution * (-1.0) :
        (rawTorque & 0x03FF) * resolution;
    }
    /// Converts torque in amper to raw torque
    static uint16_t amp2rawtorque(double torque, double resolution = 0.0045) {
      return torque > 0 ? std::round(torque / resolution) :
        static_cast<uint16_t>(std::round(std::fabs(torque) / resolution)) |
        0x0400;
    }
    /// Converts raw P gain to K_p
    static double raw2Kp(uint8_t rawPGain, double factor = 1.0 / 8.0) {
      return rawPGain * factor;
    }
    /// Converts K_p to raw P gain
    static uint8_t Kp2raw(double Kp, double factor = 1.0 / 8.0) {
      return std::round(Kp / factor);
    }
    /// Converts raw I gain to K_i
    static double raw2Ki(uint8_t rawIGain, double factor = 1000.0 / 2048.0) {
      return rawIGain * factor;
    }
    /// Converts K_i to raw I gain
    static uint8_t Ki2raw(double Ki, double factor = 1000.0 / 2048.0) {
      return std::round(Ki / factor);
    }
    /// Converts raw D gain to K_d
    static double raw2Kd(uint8_t rawDGain, double factor = 4.0 / 1000.0) {
      return rawDGain * factor;
    }
    /// Converts K_d to raw D gain
    static uint8_t Kd2raw(double Kd, double factor = 4.0 / 1000.0) {
      return std::round(Kd / factor);
    }
    /// Get model information
    static Model getModelInformation(uint16_t modelNumber);
    /// Is the model supported by the controller
    static bool isModelSupported(uint16_t modelNumber) {
      return Models::table.find(modelNumber) != Models::table.end();
    }
    /// Is the model an MX series
    static bool isModelMX(uint16_t modelNumber) {
      return std::find(Models::mxSeries.begin(), Models::mxSeries.end(),
        modelNumber) != Models::mxSeries.end();
    }
    /// Is the model torque controllable
    static bool isModelTorqueControl(uint16_t modelNumber) {
      return std::find(Models::torqueControlModels.begin(),
        Models::torqueControlModels.end(),
        modelNumber) != Models::torqueControlModels.end();
    }
    /** @}
      */

    /** \name Public members
      @{
      */
    /// Broadcasting ID
    static constexpr uint8_t broadcastingId = 0xFE;
    /** @}
      */

  private:
    /** \name Private methods
      @{
      */
    /// Performs a write on the Dynamixel device
    virtual void write(const char* buffer, size_t numBytes);
    /// Performs a read on the Dynamixel device
    virtual void read(char* buffer, size_t numBytes);
    /// Writes a packet to the Dynamixel device
    void writePacket(const std::shared_ptr<Packet>& packet);
    /// Reads a packet from the Dynamixel device
    std::shared_ptr<Packet> readPacket(uint8_t id);
    /** @}
      */

    /** \name Private members
      @{
      */
    /// Serial port
    std::shared_ptr<SerialPort> serialPort_;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_SENSOR_CONTROLLER_H
