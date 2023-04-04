#include "libdynamixel/com/SerialPort.h"

#include <ratio>

#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "libdynamixel/exceptions/SystemException.h"
#include "libdynamixel/exceptions/IOException.h"
#include "libdynamixel/exceptions/BadArgumentException.h"

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  SerialPort::SerialPort(const std::string& device, unsigned int baudRate,
      size_t dataBits, size_t stopBits, SerialParity parity, FlowControl
      flowControl, double timeout) :
      device_(device),
      baudRate_(baudRate),
      dataBits_(dataBits),
      stopBits_(stopBits),
      parity_(parity),
      flowControl_(flowControl),
      timeout_(timeout),
      deviceHandle_(0) {
  }

  SerialPort::~SerialPort() {
    if (isOpen())
      close();
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  const std::string& SerialPort::getDevice() const {
    return device_;
  }

  unsigned int SerialPort::getBaudrate() const {
    return baudRate_;
  }

  size_t SerialPort::getDatabits() const{
    return dataBits_;
  }

  size_t SerialPort::getStopbits() const {
    return stopBits_;
  }

  SerialPort::SerialParity SerialPort::getParity() const {
    return parity_;
  }

  SerialPort::FlowControl SerialPort::getFlowControl() const {
    return flowControl_;
  }

  double SerialPort::getTimeout() const {
    return timeout_;
  }
  void SerialPort::setTimeout(double timeout) {
    timeout_ = timeout;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void SerialPort::open() {
    if (isOpen())
      return;
    struct termios tios;
    memset(&tios, 0, sizeof(struct termios));
    speed_t speed;
    switch (baudRate_) {
      case 0L     : speed = B0;
                    break;
      case 50L    : speed = B50;
                    break;
      case 75L    : speed = B75;
                    break;
      case 110L   : speed = B110;
                    break;
      case 134L   : speed = B134;
                    break;
      case 150L   : speed = B150;
                    break;
      case 200L   : speed = B200;
                    break;
      case 300L   : speed = B300;
                    break;
      case 600L   : speed = B600;
                    break;
      case 1200L  : speed = B1200;
                    break;
      case 1800L  : speed = B1800;
                    break;
      case 2400L  : speed = B2400;
                    break;
      case 4800L  : speed = B4800;
                    break;
      case 9600L  : speed = B9600;
                    break;
      case 19200L : speed = B19200;
                    break;
      case 38400L : speed = B38400;
                    break;
      case 57600L : speed = B57600;
                    break;
      case 115200L: speed = B115200;
                    break;
      case 230400L: speed = B230400;
                    break;
      case 1000000L: speed = B1000000;
                    break;
      default     :
        throw BadArgumentException<size_t>(baudRate_,
          "SerialPort::open(): invalid baudrate");
    }
    if (cfsetspeed(&tios, speed))
      throw SystemException(errno, "SerialPort::open()");
    tios.c_cflag &= ~CSIZE;
    switch (dataBits_) {
      case 5 : tios.c_cflag |= CS5;
               break;
      case 6 : tios.c_cflag |= CS6;
               break;
      case 7 : tios.c_cflag |= CS7;
               break;
      case 8 : tios.c_cflag |= CS8;
               break;
      default:
        throw BadArgumentException<size_t>(dataBits_,
          "SerialPort::open(): invalid databits");
    }
    tios.c_cflag &= ~CSTOPB;
    switch (stopBits_) {
      case 1 : break;
      case 2 : tios.c_cflag |= CSTOPB;
               break;
      default:
        throw BadArgumentException<size_t>(stopBits_,
          "SerialPort::open(): invalid stopbits");
    }
    tios.c_cflag &= ~PARENB;
    tios.c_cflag &= ~PARODD;
    switch (parity_) {
      case none: break;
      case even: tios.c_cflag |= PARENB;
                 break;
      case odd : tios.c_cflag |= PARENB | PARODD;
                 break;
      default  :
        throw BadArgumentException<size_t>(parity_,
          "SerialPort::open(): invalid parity");
    }
    tios.c_cflag &= ~CRTSCTS;
    if (flowControl_ == hardware)
      tios.c_cflag |= CRTSCTS;
    tios.c_cflag |= (CLOCAL | CREAD);
    tios.c_iflag |= INPCK;
    tios.c_iflag &= ~(IXON | IXOFF | IXANY);
    if (flowControl_ == software)
      tios.c_iflag |= (IXON | IXOFF | IXANY);
    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tios.c_oflag &= ~OPOST;
    deviceHandle_ = ::open(device_.c_str(), O_RDWR | O_NDELAY | O_NOCTTY);
    if (deviceHandle_ == -1) {
      deviceHandle_ = 0;
      throw SystemException(errno, "SerialPort::open()::open()");
    }
    if (tcflush(deviceHandle_, TCIOFLUSH)) {
      close();
      throw SystemException(errno, "SerialPort::open()::tcflush()");
    }
    if (tcsetattr(deviceHandle_, TCSANOW, &tios)) {
      close();
      throw SystemException(errno, "SerialPort::open()::tcsetattr()");
    }
  }

  void SerialPort::close() {
    if (deviceHandle_) {
      if (tcdrain(deviceHandle_))
        throw SystemException(errno, "SerialPort::close()::tcdrain()");
      if (tcflush(deviceHandle_, TCIOFLUSH))
        throw SystemException(errno, "SerialPort::close()::tcflush()");
      if (::close(deviceHandle_))
        throw SystemException(errno, "SerialPort::close()::close()");
      deviceHandle_ = 0;
    }
  }

  bool SerialPort::isOpen() const {
    return (deviceHandle_ != 0);
  }

  void SerialPort::read(char* buffer, size_t numBytes) {
    if (!isOpen())
      open();
    double intPart;
    const double fractPart = std::modf(timeout_, &intPart);
    struct timeval waitd;
    waitd.tv_sec = intPart;
    waitd.tv_usec = fractPart * std::micro::den;
    size_t bytesRead = 0;
    fd_set readFlags;
    while (bytesRead < numBytes) {
      FD_ZERO(&readFlags);
      FD_SET(deviceHandle_, &readFlags);
      int res = select(deviceHandle_ + 1, &readFlags,
        reinterpret_cast<fd_set*>(0), reinterpret_cast<fd_set*>(0), &waitd);
      if(res < 0)
        throw SystemException(errno, "SerialPort::read()::select()");
      if (FD_ISSET(deviceHandle_, &readFlags)) {
        FD_CLR(deviceHandle_, &readFlags);
        res = ::read(deviceHandle_, &buffer[bytesRead], numBytes - bytesRead);
        if (res < 0)
          throw SystemException(errno, "SerialPort::read()::read()");
        bytesRead += static_cast<size_t>(res);
      }
      else
        throw IOException("SerialPort::read(): timeout occured");
    }
  }

  void SerialPort::write(const char* buffer, size_t numBytes) {
    if (!isOpen())
      open();
    double intPart;
    const double fractPart = std::modf(timeout_, &intPart);
    struct timeval waitd;
    waitd.tv_sec = intPart;
    waitd.tv_usec = fractPart * std::micro::den;
    size_t bytesWritten = 0;
    fd_set writeFlags;
    while (bytesWritten < numBytes) {
      FD_ZERO(&writeFlags);
      FD_SET(deviceHandle_, &writeFlags);
      int res = select(deviceHandle_ + 1, reinterpret_cast<fd_set*>(0),
        &writeFlags, reinterpret_cast<fd_set*>(0), &waitd);
      if(res < 0)
        throw SystemException(errno, "SerialPort::write()::select()");
      if (FD_ISSET(deviceHandle_, &writeFlags)) {
        FD_CLR(deviceHandle_, &writeFlags);
        res = ::write(deviceHandle_, &buffer[bytesWritten],
          numBytes - bytesWritten);
        if (res < 0)
          throw SystemException(errno, "SerialPort::write()::write()");
        bytesWritten += res;
      }
      else
        throw IOException("SerialPort::write(): timeout occured");
    }
  }

}
