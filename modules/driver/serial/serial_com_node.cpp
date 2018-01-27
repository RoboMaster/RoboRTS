#include <cmake-build-debug/devel/include/messages/EnemyPos.h>
#include "modules/driver/serial/serial_com_node.h"
#include "infantry_info.h"
namespace rrts {
namespace driver {
namespace serial {
SerialComNode::SerialComNode(std::string module_name)
    : rrts::common::RRTS::RRTS(module_name), fd_(0), is_open_(false), stop_receive_(true) {
  SerialPortConfig serial_port_config;
  CHECK(rrts::common::ReadProtoFromTextFile("modules/driver/serial/config/serial_comm_config.prototxt",
                                            &serial_port_config))
  << "Error loading proto file for serial.";
  CHECK(serial_port_config.has_serial_port()) << "Not set port.";
  CHECK(serial_port_config.has_serial_boudrate()) << "Not set baudrate.";
  baudrate_ = serial_port_config.serial_boudrate();
  port_ = serial_port_config.serial_port();
  CHECK(Initialization()) << "Initialization error.";
  is_open_ = true;
  stop_receive_ = false;
}

bool SerialComNode::Initialization() {
  return SerialInitialization(port_);
}

bool SerialComNode::SerialInitialization(std::string port) {
  return SerialInitialization(port, 0, 8, 1, 'N');
}

bool SerialComNode::SerialInitialization(std::string port,
                                         int flow_control,
                                         int data_bits,
                                         int stop_bits,
                                         int parity) {
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  CHECK(fd_ != -1) << "Serial port open failed!";
  CHECK(fcntl(fd_, F_SETFL, 0) >= 0) << "Fcntl failed!";
  CHECK(tcgetattr(fd_, &termios_options_) == 0) << "Get serial attributes error!";
  ConfigBaudrate(baudrate_);
  termios_options_.c_cflag |= CLOCAL;
  termios_options_.c_cflag |= CREAD;
  switch (flow_control) {
    case 0 :termios_options_.c_cflag &= ~CRTSCTS;
      break;
    case 1 :termios_options_.c_cflag |= CRTSCTS;
      break;
    case 2 :termios_options_.c_cflag |= IXON | IXOFF | IXANY;
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
      return false;
  }
  switch (parity) {
    case 'n':
    case 'N':termios_options_.c_cflag &= ~PARENB;
      termios_options_.c_iflag &= ~INPCK;
      break;
    case 'o':
    case 'O':termios_options_.c_cflag |= (PARODD | PARENB);
      termios_options_.c_iflag |= INPCK;
      break;
    case 'e':
    case 'E':termios_options_.c_cflag |= PARENB;
      termios_options_.c_cflag &= ~PARODD;
      termios_options_.c_iflag |= INPCK;
      break;
    case 's':
    case 'S':termios_options_.c_cflag &= ~PARENB;
      termios_options_.c_cflag &= ~CSTOPB;
      break;
    default: LOG_FATAL << "Unsupported parity";
      return false;
  }
  switch (stop_bits) {
    case 1: termios_options_.c_cflag &= ~CSTOPB;
      break;
    case 2: termios_options_.c_cflag |= CSTOPB;
      break;
    default: LOG_FATAL << "Unsupported stop bits";
      return false;
  }
  termios_options_.c_oflag &= ~OPOST;
  termios_options_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termios_options_.c_cc[VTIME] = 1;
  termios_options_.c_cc[VMIN] = 1;
  tcflush(fd_, TCIFLUSH);
  CHECK_EQ(tcsetattr(fd_, TCSANOW, &termios_options_), 0)
    << "Set serial attributes error!";
  LOG_INFO << "Com config success";
  return true;
}

bool SerialComNode::ConfigBaudrate(int baudrate) {
  int i;
  int speed_arr[] = {B921600, B115200, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {921600, 115200, 19200, 9600, 4800, 2400, 1200, 300};
  for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (baudrate == name_arr[i]) {
      cfsetispeed(&termios_options_, speed_arr[i]);
      cfsetospeed(&termios_options_, speed_arr[i]);
      return true;
    }
  }
  LOG_ERROR << "Baudrate set error.";
  return false;
}

void SerialComNode::Run() {
  receive_loop_thread_ = new std::thread(boost::bind(&SerialComNode::ReceiveLoop, this));
  sub_cmd_gim_ = nh_.subscribe("enemy_pos", 1, &SerialComNode::GimbalControlCallback, this);
  sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &SerialComNode::ChassisControlCallback, this);
}

void SerialComNode::ReceiveLoop() {
  while (!stop_receive_ && is_open_) {
    read_buff_index_ = 0;
    read_len_ = ReceiveData(fd_, UART_BUFF_SIZE);
    while (read_len_--) {
      byte_ = rx_buf_[read_buff_index_++];
      switch (unpack_step_e_) {
        case STEP_HEADER_SOF: {
          if (byte_ == UP_REG_ID) {
            protocol_packet_[index_++] = byte_;
            unpack_step_e_ = STEP_LENGTH_LOW;
          } else {
            index_ = 0;
          }
        }
          break;
        case STEP_LENGTH_LOW: {
          data_length_ = byte_;
          protocol_packet_[index_++] = byte_;
          unpack_step_e_ = STEP_LENGTH_HIGH;
        }
          break;
        case STEP_LENGTH_HIGH: {
          data_length_ |= (byte_ << 8);
          protocol_packet_[index_++] = byte_;
          if (data_length_ < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CMD_LEN - CRC_LEN)) {
            unpack_step_e_ = STEP_FRAME_SEQ;
          } else {
            unpack_step_e_ = STEP_HEADER_SOF;
            index_ = 0;
          }
        }
          break;
        case STEP_FRAME_SEQ: {
          protocol_packet_[index_++] = byte_;
          unpack_step_e_ = STEP_HEADER_CRC8;
        }
          break;
        case STEP_HEADER_CRC8: {
          protocol_packet_[index_++] = byte_;
          if ((index_ == HEADER_LEN) && VerifyCrcOctCheckSum(protocol_packet_, HEADER_LEN)) {
            unpack_step_e_ = STEP_DATA_CRC16;
          } else {
            unpack_step_e_ = STEP_HEADER_SOF;
            index_ = 0;
          }
        }
          break;
        case STEP_DATA_CRC16: {
          if (index_ < (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
            protocol_packet_[index_++] = byte_;
          } else if (index_ > (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
            LOG_WARNING << "Index Beyond";
          }
          if (index_ == (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
            unpack_step_e_ = STEP_HEADER_SOF;
            index_ = 0;
            if (VerifyCrcHexCheckSum(protocol_packet_, HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
              DataHandle();
            } else {
              LOG_WARNING << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>CRC16 INVALID<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
            }
          }
        }
          break;
        default: {
          LOG_WARNING << "Unpack not well";
          unpack_step_e_ = STEP_HEADER_SOF;
          index_ = 0;
        }
          break;
      }
    }
  }
}

int SerialComNode::ReceiveData(int fd, int data_length) {
  int received_length, selected;
  fd_set fs_read;
  struct timeval time;
  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);
  time.tv_sec = 10;
  time.tv_usec = 0;
  selected = select(fd + 1, &fs_read, NULL, NULL, &time);
  if (selected > 0) {
    received_length = read(fd, rx_buf_, data_length);
  } else if (selected == 0) {
    LOG_WARNING_EVERY(10000) << "Uart Timeout";
  } else {
    LOG_ERROR << "Select function error";
  }
  return received_length;
}

void SerialComNode::DataHandle() {
  std::lock_guard<std::mutex> guard(mutex_receive_);
  auto *p_header = (FrameHeader *) protocol_packet_;
  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id = *(uint16_t *) (protocol_packet_ + HEADER_LEN);
  uint8_t *data_addr = protocol_packet_ + HEADER_LEN + CMD_LEN;
  switch (cmd_id) {
    case GAME_INFO_ID: memcpy(&game_information_, data_addr, data_length);
      //use it to control pipeline.
      break;
    case REAL_BLOOD_DATA_ID: memcpy(&robot_hurt_data_, data_addr, data_length);
      break;
    case REAL_SHOOT_DATA_ID: memcpy(&real_shoot_data_, data_addr, data_length);
      break;
    case REAL_RFID_DATA_ID: memcpy(&rfid_data_, data_addr, data_length);
      break;
    case GAME_RESULT_ID: memcpy(&game_result_data_, data_addr, data_length);
      break;
    case GAIN_BUFF_ID: memcpy(&get_buff_data_, data_addr, data_length);
      break;
    case SERVER_TO_USER_ID: memcpy(&student_download_data_, data_addr, data_length);
      break;
    case CHASSIS_DATA_ID: {
      ros::Time current_time = ros::Time::now();
      memcpy(&chassis_information_, data_addr, data_length);
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      double x = chassis_information_.x_position / 1000, y = chassis_information_.y_position / 1000;
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(
          chassis_information_.gyro_angle * 0.5 + chassis_information_.ecd_angle * 0.5);
      odom.pose.pose.orientation = q;
      odom.twist.twist.linear.x = (double) chassis_information_.x_speed / 1000;
      odom.twist.twist.linear.y = (double) chassis_information_.y_speed / 1000;
      odom.twist.twist.angular.z =
          (double) (chassis_information_.gyro_palstance + chassis_information_.ecd_palstance) / 2;
      odom_pub_.publish(odom);
      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header.frame_id = "odom";
      odom_tf.header.stamp = current_time;
      odom_tf.child_frame_id = "base_link";
      odom_tf.transform.translation.x = x;
      odom_tf.transform.translation.y = y;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation = q;
      tf_broadcaster_.sendTransform(odom_tf);
    }
      break;

    case GIMBAL_DATA_ID: memcpy(&gimbal_information_, data_addr, data_length);
      printf("-->%d\n", gimbal_information_.ctrl_mode);
      break;

    case SHOOT_TASK_DATA_ID: memcpy(&shoot_task_data_, data_addr, data_length);
      printf("-->%d\n", shoot_task_data_.fric_wheel_run);
      break;

    case INFANTRY_ERR_ID: memcpy(&global_error_data_, data_addr, data_length);
      printf("-->%d\n", global_error_data_.err_sta);
      break;

    case CONFIG_RESPONSE_ID: memcpy(&config_response_data_, data_addr, data_length);
      printf("-->%d\n", config_response_data_.chassis_config);
      break;

    case CALI_RESPONSE_ID: memcpy(&cali_response_data_, data_addr, data_length);
      printf("-->%d\n", cali_response_data_.type);
      break;

    case REMOTE_CTRL_INFO_ID: memcpy(&rc_info_data_, data_addr, data_length);
      printf("-->%d\n", rc_info_data_.ch1);
      break;

    case BOTTOM_VERSION_ID: memcpy(&version_info_data_, data_addr, data_length);
      printf("-->%d\n", version_info_data_.num[0]);
      break;

    default:LOG_WARNING << "Cannot Get Command ID";
      break;
  }
}

void SerialComNode::GimbalControlCallback(const messages::EnemyPosConstPtr &msg) {
  gimbal_control_data_.ctrl_mode = GIMBAL_POSITION_MODE;
  gimbal_control_data_.pit_ref = msg->enemy_pitch;
  gimbal_control_data_.yaw_ref = msg->enemy_yaw;
  gimbal_control_data_.visual_valid = 1;
  if (gimbal_control_data_.visual_valid && !stop_send_ && is_open_) {
    int length = sizeof(GimbalControl);
    SendDataHandle(GIMBAL_CTRL_ID, (uint8_t *) &gimbal_control_data_, length);
    if (SendData(length) != length) {
      LOG_WARNING << "Gimbal control data sent error";
    }
  }
}

void SerialComNode::ChassisControlCallback(const geometry_msgs::Twist::ConstPtr &vel) {
  chassis_control_data_.ctrl_mode = AUTO_FOLLOW_GIMBAL;
  chassis_control_data_.x_speed = vel->linear.x * 1000.0;
  chassis_control_data_.y_speed = vel->linear.y * 1000.0;
  chassis_control_data_.w_info.x_offset = 0;
  chassis_control_data_.w_info.y_offset = 0;
  chassis_control_data_.w_info.w_speed = vel->angular.z;
  int length = sizeof(ChassisControl);
  SendDataHandle(CHASSIS_CTRL_ID, (uint8_t *) &chassis_control_data_, length);
  if (SendData(length) != length) {
    LOG_WARNING << "Chassis control data sent error";
  }
}

void SerialComNode::SendDataHandle(uint16_t cmd_id, uint8_t *p_data, uint16_t len) {
  FrameHeader *p_header = (FrameHeader *) tx_buf_;
  p_header->sof = UP_REG_ID;
  p_header->data_length = len;
  memcpy(&tx_buf_[HEADER_LEN], (uint8_t *) &cmd_id, CMD_LEN);
  AppendCrcOctCheckSum(tx_buf_, HEADER_LEN);
  memcpy(&tx_buf_[HEADER_LEN + CMD_LEN], p_data, len);
  AppendCrcHexCheckSum(tx_buf_, HEADER_LEN + CMD_LEN + len + CRC_LEN);
}

int SerialComNode::SendData(int data_len) {
  int length = 0;
  length = write(fd_, tx_buf_, data_len);
  if (length == data_len) {
    return length;
  } else {
    tcflush(fd_, TCOFLUSH);
    LOG_FATAL << "Serial write error";
    return -1;
  }
}

SerialComNode::~SerialComNode() {
  if (receive_loop_thread_ != nullptr) {
    stop_receive_ = true;
    receive_loop_thread_->join();
    delete receive_loop_thread_;
  }
  stop_send_ = true;
  close(fd_);
}

void SerialComNode::Stop() {
  stop_send_ = true;
  stop_receive_ = true;
}

void SerialComNode::Resume() {
  stop_send_ = false;
  stop_receive_ = false;
}

uint8_t SerialComNode::GetCrcOctCheckSum(uint8_t *message, uint32_t length, uint8_t crc) {
  uint8_t index;
  while (length--) {
    index = crc ^ (*message++);
    crc = kCrcOctTable[index];
  }
  return (crc);
}

bool SerialComNode::VerifyCrcOctCheckSum(uint8_t *message, uint16_t length) {
  uint8_t expected = 0;
  if ((message == 0) || (length <= 2)) {
    LOG_WARNING << "Verify CRC8 false";
    return false;
  }
  expected = GetCrcOctCheckSum(message, length - 1, kCrc8);
  return (expected == message[length - 1]);
}

void SerialComNode::AppendCrcOctCheckSum(uint8_t *message, uint16_t length) {
  uint8_t crc = 0;
  if ((message == 0) || (length <= 2)) {
    LOG_WARNING << "Append CRC8 NULL";
    return;
  };
  crc = GetCrcOctCheckSum(message, length - 1, kCrc8);
  message[length - 1] = crc;
}

uint16_t SerialComNode::GetCrcHexCheckSum(uint8_t *message, uint32_t length, uint16_t crc) {
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

bool SerialComNode::VerifyCrcHexCheckSum(uint8_t *message, uint32_t length) {
  uint16_t expected = 0;
  if ((message == NULL) || (length <= 2)) {
    LOG_WARNING << "Verify CRC16 bad";
    return false;
  }
  expected = GetCrcHexCheckSum(message, length - 2, kCrc);
  return ((expected & 0xff) == message[length - 2] && ((expected >> 8) & 0xff) == message[length - 1]);
}

void SerialComNode::AppendCrcHexCheckSum(uint8_t *message, uint32_t length) {
  uint16_t crc = 0;
  if ((message == NULL) || (length <= 2)) {
    LOG_WARNING << "Append CRC 16 NULL";
    return;
  }
  crc = GetCrcHexCheckSum(message, length - 2, kCrc);
  message[length - 2] = (uint8_t) (crc & 0x00ff);
  message[length - 1] = (uint8_t) ((crc >> 8) & 0x00ff);
}

} //namespace serial
} //namespace driver
} //namespace rrts
MAIN(rrts::driver::serial::SerialComNode, "serial_com_node")