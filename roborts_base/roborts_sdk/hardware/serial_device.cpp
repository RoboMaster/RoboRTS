/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "serial_device.h"

namespace roborts_sdk {
SerialDevice::SerialDevice(std::string port_name,
                           int baudrate) :
    port_name_(port_name),
    baudrate_(baudrate),
    data_bits_(8),
    parity_bits_('N'),
    stop_bits_(1) {}

SerialDevice::~SerialDevice() {
  CloseDevice();
}

bool SerialDevice::Init() {

  DLOG_INFO << "Attempting to open device " << port_name_ << " with baudrate " << baudrate_;
  if (port_name_.c_str() == nullptr) {
    port_name_ = "/dev/ttyUSB0";
  }
  if (OpenDevice() && ConfigDevice()) {
    FD_ZERO(&serial_fd_set_);
    FD_SET(serial_fd_, &serial_fd_set_);
    DLOG_INFO << "...Serial started successfully.";
    return true;
  } else {
    DLOG_ERROR << "...Failed to start serial "<<port_name_;
    CloseDevice();
    return false;
  }
}

bool SerialDevice::OpenDevice() {

#ifdef __arm__
  serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NONBLOCK);
#elif __x86_64__
  serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#else
  serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#endif

  if (serial_fd_ < 0) {
    DLOG_ERROR << "cannot open device " << serial_fd_ << " " << port_name_;
    return false;
  }

  return true;
}

bool SerialDevice::CloseDevice() {
  close(serial_fd_);
  serial_fd_ = -1;
  return true;
}

bool SerialDevice::ConfigDevice() {
  int st_baud[] = {B4800, B9600, B19200, B38400,
                   B57600, B115200, B230400, B921600};
  int std_rate[] = {4800, 9600, 19200, 38400, 57600, 115200,
                    230400, 921600, 1000000, 1152000, 3000000};
  int i, j;
  /* save current port parameter */
  if (tcgetattr(serial_fd_, &old_termios_) != 0) {
    DLOG_ERROR << "fail to save current port";
    return false;
  }
  memset(&new_termios_, 0, sizeof(new_termios_));

  /* config the size of char */
  new_termios_.c_cflag |= CLOCAL | CREAD;
  new_termios_.c_cflag &= ~CSIZE;

  /* config data bit */
  switch (data_bits_) {
    case 7:new_termios_.c_cflag |= CS7;
      break;
    case 8:new_termios_.c_cflag |= CS8;
      break;
    default:new_termios_.c_cflag |= CS8;
      break; //8N1 default config
  }
  /* config the parity bit */
  switch (parity_bits_) {
    /* odd */
    case 'O':
    case 'o':new_termios_.c_cflag |= PARENB;
      new_termios_.c_cflag |= PARODD;
      break;
      /* even */
    case 'E':
    case 'e':new_termios_.c_cflag |= PARENB;
      new_termios_.c_cflag &= ~PARODD;
      break;
      /* none */
    case 'N':
    case 'n':new_termios_.c_cflag &= ~PARENB;
      break;
    default:new_termios_.c_cflag &= ~PARENB;
      break; //8N1 default config
  }
  /* config baudrate */
  j = sizeof(std_rate) / 4;
  for (i = 0; i < j; ++i) {
    if (std_rate[i] == baudrate_) {
      /* set standard baudrate */
      cfsetispeed(&new_termios_, st_baud[i]);
      cfsetospeed(&new_termios_, st_baud[i]);
      break;
    }
  }
  /* config stop bit */
  if (stop_bits_ == 1)
    new_termios_.c_cflag &= ~CSTOPB;
  else if (stop_bits_ == 2)
    new_termios_.c_cflag |= CSTOPB;
  else
    new_termios_.c_cflag &= ~CSTOPB; //8N1 default config

/* config waiting time & min number of char */
  new_termios_.c_cc[VTIME] = 1;
  new_termios_.c_cc[VMIN] = 18;

  /* using the raw data mode */
  new_termios_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  new_termios_.c_oflag &= ~OPOST;

  /* flush the hardware fifo */
  tcflush(serial_fd_, TCIFLUSH);

  /* activite the configuration */
  if ((tcsetattr(serial_fd_, TCSANOW, &new_termios_)) != 0) {
    DLOG_ERROR << "failed to activate serial configuration";
    return false;
  }
  return true;

}

int SerialDevice::Read(uint8_t *buf, int len) {
  int ret = -1;

  if (NULL == buf) {
    return -1;
  } else {
    ret = read(serial_fd_, buf, len);
    DLOG_INFO<<"Read once length: "<<ret;
    while (ret == 0) {
      LOG_ERROR << "Connection closed, try to reconnect.";
      while (!Init()) {
        usleep(500000);
      }
      LOG_INFO << "Reconnect Success.";
      ret = read(serial_fd_, buf, len);
    }
    return ret;
  }
}

int SerialDevice::Write(const uint8_t *buf, int len) {
  return write(serial_fd_, buf, len);
}
}
