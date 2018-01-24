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
      "modules/serial_comm/proto/serial_comm_config.prototxt",
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
                                      int databits,
                                      int stopbits,
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
  switch (databits) {
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
  switch (stopbits) {
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

unsigned char SerialComm::Get_crc8_check_sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8) {
  unsigned char ucIndex;
  while (dwLength--) {
    ucIndex = ucCRC8 ^ (*pchMessage++);
    ucCRC8 = CRC8_TAB[ucIndex];
  }
  return (ucCRC8);
}

uint16_t SerialComm::Verify_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength) {
  unsigned char ucExpected = 0;
  if ((pchMessage == 0) || (dwLength <= 2)) {
    LOG_WARNING << "Verify CRC8 Return 0";
    return 0;
  }
  ucExpected = Get_crc8_check_sum(pchMessage, dwLength - 1, CRC8_INIT);
  return (ucExpected == pchMessage[dwLength - 1]);
}

void SerialComm::Append_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength) {
  unsigned char ucCRC = 0;
  if ((pchMessage == 0) || (dwLength <= 2)) {
    LOG_WARNING << "Append CRC8 NULL";
    return;
  };
  ucCRC = Get_crc8_check_sum(pchMessage, dwLength - 1, CRC8_INIT);
  pchMessage[dwLength - 1] = ucCRC;
}

uint16_t SerialComm::Get_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC) {
  uint8_t chData;
  if (pchMessage == NULL) {
    return 0xFFFF;
  }
  while (dwLength--) {
    chData = *pchMessage++;
    (wCRC) = ((uint16_t) (wCRC) >> 8) ^ wCRC_Table[((uint16_t) (wCRC) ^ (uint16_t) (chData)) & 0x00ff];
  }
  return wCRC;
}

uint32_t SerialComm::Verify_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wExpected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) {
    LOG_WARNING << "Verify CRC16 bad";
    return 0;
  }
  wExpected = Get_crc16_check_sum(pchMessage, dwLength - 2, CRC_INIT);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void SerialComm::Append_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength) {
  uint16_t wCRC = 0;
  if ((pchMessage == NULL) || (dwLength <= 2)) {
    LOG_WARNING << "Append CRC 16 NULL";
    return;
  }
  wCRC = Get_crc16_check_sum(pchMessage, dwLength - 2, CRC_INIT);
  pchMessage[dwLength - 2] = (uint8_t) (wCRC & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t) ((wCRC >> 8) & 0x00ff);
}

} //namespace serial
} //namespace driver
} //namespace rrts