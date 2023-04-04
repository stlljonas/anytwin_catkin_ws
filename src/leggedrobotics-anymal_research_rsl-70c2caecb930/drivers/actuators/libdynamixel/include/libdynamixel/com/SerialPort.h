/** \file SerialPort.h
    \brief This file defines the SerialPort class which interfaces a serial
           port.
  */

#ifndef LIBDYNAMIXEL_COM_SERIAL_PORT_H
#define LIBDYNAMIXEL_COM_SERIAL_PORT_H

#include <string>

namespace dynamixel {

  /** The class SerialPort is an interface for serial communication.
      \brief Serial communication interface.
    */
  class SerialPort {
  public:
    /** \name Types definitions
      @{
      */
    /// The enum SerialParity represents the different kinds of serial parity.
    enum SerialParity {
      /// No parity
      none = 0,
      /// Odd parity
      odd = 1,
      /// Even parity
      even = 2
    };

    /// The enum FlowControl represents the different kinds of flow controls.
    enum FlowControl {
      /// No flow control
      no = 0,
      /// Hardware flow control
      hardware = 1,
      /// Software flow control
      software = 2
    };
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructs the serial port from parameters
    SerialPort(const std::string& device = "/dev/ttyUSB0", unsigned int
      baudRate = 115200, size_t dataBits = 8, size_t stopBits = 1, SerialParity
      parity = none, FlowControl flowControl = no, double timeout = 0.2);
    /// Copy constructor
    SerialPort(const SerialPort& other) = delete;
    /// Copy assignment operator
    SerialPort& operator = (const SerialPort& other) = delete;
    /// Move constructor
    SerialPort(SerialPort&& other) = delete;
    /// Move assignment operator
    SerialPort& operator = (SerialPort&& other) = delete;
     /// Destructor
    ~SerialPort();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the path to the device
    const std::string& getDevice() const;
    /// Returns the baudrate
    unsigned int getBaudrate() const;
    /// Returns the databits
    size_t getDatabits() const;
    /// Returns the stopbits
    size_t getStopbits() const;
    /// Returns the parity
    SerialParity getParity() const;
    /// Returns the flow control
    FlowControl getFlowControl() const;
    /// Returns the timeout
    double getTimeout() const;
    /// Sets the timeout
    void setTimeout(double timeout);
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Opens the serial port
    void open();
    /// Closes the serial port
    void close();
    /// Check if the port is open
    bool isOpen() const;
    /// Reads a buffer of bytes from the serial port
    void read(char* buffer, size_t numBytes);
    /// Writes a buffer of bytes to the serial port
    void write(const char* buffer, size_t numBytes);
    /** @}
      */

  private:

    /** \name Private members
      @{
      */
    /// Device
    std::string device_;
    /// Baudrate
    unsigned int baudRate_;
    /// Data bits size
    size_t dataBits_;
    /// Stop bits number
    size_t stopBits_;
    /// Parity of the device
    SerialParity parity_;
    /// Flow control of the device
    FlowControl flowControl_;
    /// Timeout for read/write operations in seconds
    double timeout_;
    /// Handle on the device
    int deviceHandle_;
    /** @}
      */

  };

}

#endif // LIBDYNAMIXEL_COM_SERIAL_PORT_H_H
