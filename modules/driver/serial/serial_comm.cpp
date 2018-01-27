/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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

#include "modules/driver/serial/serial_comm.h"

namespace rrts {
namespace driver {
namespace serial {

SerialComm::SerialComm(std::string name) : rrts::common::RRTS::RRTS(name) {}

void SerialComm::Initialization() {
  SerialPortConfig serial_port_config_;
  CHECK(rrts::common::ReadProtoFromTextFile(
      "modules/driver/serial/config/serial_comm_config.prototxt",
      &serial_port_config_)) << "Error to load serial port config file";
  CHECK(serial_port_config_.has_serial_port())
  << "Serial port name not defined!";
  CHECK(serial_port_config_.has_serial_boudrate())
  << "Serial boudrate not defined!";
  SerialInitialization(serial_port_config_.serial_port(),
                       serial_port_config_.serial_boudrate());
}

void SerialComm::SerialInitialization(std::string port, int boudrate) {
  SerialInitialization(port, boudrate, 0, 8, 1, 'N');
}

void SerialComm::SerialInitialization(std::string port,
                                      int boudrate,
                                      int flow_control,
                                      int data_bits,
                                      int stop_bits,
                                      int parity) {
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  PCHECK(fd_ != -1) << "Serial port open failed!";
  PCHECK(fcntl(fd_, F_SETFL, 0) >= 0) << "Fcntl failed!";
  PCHECK(tcgetattr(fd_, &termios_options_) == 0) << "Setup serial error!";
  ConfigBoudrate(boudrate);
  termios_options_.c_cflag |= CLOCAL;
  termios_options_.c_cflag |= CREAD;
  switch (flow_control) {
    case 0 :
      termios_options_.c_cflag &= ~CRTSCTS;
      break;
    case 1 :
      termios_options_.c_cflag |= CRTSCTS;
      break;
    case 2 :
      termios_options_.c_cflag |= IXON | IXOFF | IXANY;
      break;
    default: termios_options_.c_cflag &= ~CRTSCTS;
      break;
  }
  termios_options_.c_cflag &= ~CSIZE;
  switch (data_bits) {
    case 5    : termios_options_.c_cflag |= CS5;
      break;
    case 6    : termios_options_.c_cflag |= CS6;
      break;
    case 7    : termios_options_.c_cflag |= CS7;
      break;
    case 8    : termios_options_.c_cflag |= CS8;
      break;
    default: LOG_FATAL << "Unsupported data size";
      return;
  }
  switch (parity) {
    case 'n':
    case 'N':
      termios_options_.c_cflag &= ~PARENB;
      termios_options_.c_iflag &= ~INPCK;
      break;
    case 'o':
    case 'O':
      termios_options_.c_cflag |= (PARODD | PARENB);
      termios_options_.c_iflag |= INPCK;
      break;
    case 'e':
    case 'E':
      termios_options_.c_cflag |= PARENB;
      termios_options_.c_cflag &= ~PARODD;
      termios_options_.c_iflag |= INPCK;
      break;
    case 's':
    case 'S':
      termios_options_.c_cflag &= ~PARENB;
      termios_options_.c_cflag &= ~CSTOPB;
      break;
    default: LOG_FATAL << "Unsupported parity";
      return;
  }
  switch (stop_bits) {
    case 1: termios_options_.c_cflag &= ~CSTOPB;
      break;
    case 2: termios_options_.c_cflag |= CSTOPB;
      break;
    default: LOG_FATAL << "Unsupported stop bits";
      return;
  }
  termios_options_.c_oflag &= ~OPOST;
  termios_options_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termios_options_.c_cc[VTIME] = 1;
  termios_options_.c_cc[VMIN] = 1;
  tcflush(fd_, TCIFLUSH);
  CHECK_EQ(tcsetattr(fd_, TCSANOW, &termios_options_), 0)
    << "Serial com set error!";
  LOG_INFO << "Com config success";
}

void SerialComm::ConfigBoudrate(int boudrate) {
  int i;
  int speed_arr[] = {B921600, B115200, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {921600, 115200, 19200, 9600, 4800, 2400, 1200, 300};
  for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (boudrate == name_arr[i]) {
      cfsetispeed(&termios_options_, speed_arr[i]);
      cfsetospeed(&termios_options_, speed_arr[i]);
      return;
    }
  }
  LOG_ERROR << "Boudrate set error.";
}

unsigned char SerialComm::GetCrcOctCheckSum(unsigned char *message, unsigned int length, unsigned char crc) {
  unsigned char index;
  while (length--) {
    index = crc ^ (*message++);
    crc = kCrcOctTable[index];
  }
  return (crc);
}

uint16_t SerialComm::VerifyCrcOctCheckSum(uint8_t *message, uint16_t length) {
  unsigned char expected = 0;
  if ((message == 0) || (length <= 2)) {
    LOG_WARNING << "Verify CRC8 Return 0";
    return 0;
  }
  expected = GetCrcOctCheckSum(message, length - 1, kCrc8);
  return (expected == message[length - 1]);
}

void SerialComm::AppendCrcOctCheckSum(uint8_t *message, uint16_t length) {
  unsigned char crc = 0;
  if ((message == 0) || (length <= 2)) {
    LOG_WARNING << "Append CRC8 NULL";
    return;
  };
  crc = GetCrcOctCheckSum(message, length - 1, kCrc8);
  message[length - 1] = crc;
}

uint16_t SerialComm::GetCreHexCheckSum(uint8_t *message, uint32_t length, uint16_t crc) {
  uint8_t data;
  if (message == NULL) {
    return 0xFFFF;
  }
  while (length--) {
    data = *message++;
    (crc) = ((uint16_t) (crc) >> 8) ^ kCrcTable[((uint16_t) (crc) ^ (uint16_t) (data)) & 0x00ff];
  }
  return crc;
}

uint32_t SerialComm::VerifyCrcHexCheckSum(uint8_t *message, uint32_t length) {
  uint16_t expected = 0;

  if ((message == NULL) || (length <= 2)) {
    LOG_WARNING << "Verify CRC16 bad";
    return 0;
  }
  expected = GetCreHexCheckSum(message, length - 2, kCrc);
  return ((expected & 0xff) == message[length - 2] && ((expected >> 8) & 0xff) == message[length - 1]);
}

void SerialComm::AppendCrcHexCheckSum(uint8_t *message, uint32_t length) {
  uint16_t crc = 0;
  if ((message == NULL) || (length <= 2)) {
    LOG_WARNING << "Append CRC 16 NULL";
    return;
  }
  crc = GetCreHexCheckSum(message, length - 2, kCrc);
  message[length - 2] = (uint8_t) (crc & 0x00ff);
  message[length - 1] = (uint8_t) ((crc >> 8) & 0x00ff);
}

} //namespace serial
} //namespace driver
} //namespace rrts