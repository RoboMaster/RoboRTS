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

#ifndef ROBORTS_SDK_PROTOCOL_H
#define ROBORTS_SDK_PROTOCOL_H
#include <memory>
#include <cstring>

#include "../hardware/serial_device.h"
#include "../utilities/memory_pool.h"
#include "../utilities/circular_buffer.h"
#include "../utilities/crc.h"
#include <map>
#include <atomic>
#include <thread>

namespace roborts_sdk {
/*************************** Package Format **************************/
/**
 * @brief Package header used to resolve package
 */
typedef struct Header {
  uint32_t sof : 8;
  uint32_t length : 10;
  uint32_t version : 6;
  uint32_t session_id : 5;
  uint32_t is_ack : 1;
  uint32_t reserved0 : 2; // Always 0
  uint32_t sender: 8;
  uint32_t receiver: 8;
  uint32_t reserved1 : 16;
  uint32_t seq_num : 16;
  uint32_t crc : 16;
} Header;

/*************************** Package Infomation **********************/
/**
 * @brief Key information that a protocol command should include,
 * @details Used as an interface with dispatch layer.
 */
typedef struct CommandInfo {
  //! command set for different module, i.e. gimbal, chassis
  uint8_t cmd_set;
  //! command id for different commands in the module
  uint8_t cmd_id;
  bool need_ack;
  uint8_t sender;
  uint8_t receiver;
  uint16_t length;
} CommandInfo;

/**
 * @brief Message header
 * @details Used as an interface with dispatch layer.
 */
typedef struct MessageHeader {
  uint16_t seq_num;
  uint8_t session_id;
  bool is_ack;
} MessageHeader;

/**
 * @brief Message data
 * @details Used as an interface with dispatch layer.
 */
typedef union MessageData {
  uint8_t raw_data[1024];
} MessageData;

/************************* Session Information ***********************/
/**
 * @brief Status for ack session
 */
enum class ACKSessionStatus : uint8_t {
  ACK_SESSION_IDLE = 0,     ///<ack session is free without receiving ack needed command
  ACK_SESSION_PROCESS = 1,  ///<ack session is in process of dealing with ack feedback after receiving ack needed command
  ACK_SESSION_USING = 2,    ///<ack session has finished sending the ack
};

/**
 * @brief Status for ack session
 */
enum class CMDSessionMode : uint8_t {
  CMD_SESSION_0 = 0,      ///<cmd session does not need ack
  CMD_SESSION_1 = 1,      ///<cmd session need ack but not a must
  CMD_SESSION_AUTO = 32,  ///<cmd session need ack and wait the ack arriving until timeout
};

/**
 * @brief Information for command session
 */
typedef struct CMDSession {
  //! session id used to distinguish the session mode
  uint8_t session_id;
  //! whether the cmd session is used
  bool usage_flag;
  //! memory block which holds the package for the command session
  MemoryBlock *memory_block_ptr;
  //! command set for different module, i.e. gimbal, chassis
  uint8_t cmd_set;
  //! command id for different commands in the module
  uint8_t cmd_id;

  //! times already retry sending
  uint16_t sent;
  //! times need to retry sending in total
  uint16_t retry_time;
  //! timeout for checking arriving status of corresponding ack, unit is ms
  std::chrono::milliseconds ack_timeout;
  //! last time point for checking arriving status of corresponding ack
  std::chrono::steady_clock::time_point pre_time_stamp;

  //! last sequence number
  uint32_t pre_seq_num;
} CMDSession;

/**
 * @brief Information for ack session
 */
typedef struct ACKSession {
  //! session id used to distinguish the session mode
  uint8_t session_id;
  //! session status used to distinguish the process for corresponding command arriving and ack sending.
  ACKSessionStatus session_status;
  //! memory block which holds the package for the ack session
  MemoryBlock *memory_block_ptr;
} ACKSession;

/************************* Receive Container ***************************/

/**
 * @brief Receive Stream
 * @details Used for preprocessing the byte stream to find a full package
 */
typedef struct RecvStream {
  //! index of stream reuse
  uint32_t reuse_index;
  //! count of stream reuse
  uint32_t reuse_count;
  //! index of stream receiving
  uint32_t recv_index;
  //! an array buffer for stream receiving
  uint8_t *recv_buff;
} RecvStream;

/**
 * @brief Receive Stream
 * @details Used as an container after resolving the package and an interface with dispatch layer
 */
typedef struct RecvContainer {
  //! command information
  CommandInfo command_info;
  //! message header
  MessageHeader message_header;
  //! message data
  MessageData message_data;
} RecvContainer;

/**
 * @brief Class for protocol layer.
 */
class Protocol {
 public:
  /**
   * @brief Constructor for protocol
   * @param serial_device_ptr Pointer for serial device
   */
  explicit Protocol(std::shared_ptr<SerialDevice> serial_device_ptr);
  /**
   * @brief Destructor for protocol
   */
  ~Protocol();
  /***************************** Interface ****************************/
  /**
   * @brief Initialize memory pool, stream, container and session,
   *        start the automatic repeat sending thread and receiving pool thread
   * @return True if success
   */
  bool Init();
  /**
   * @brief Check whether the sent command with need for ack gets its ack back and automatic retry sending command
   * @details An endless loop to check ack status for the command every timeout duration,
   *          to resend the command until the sent times reach the given retry times.
   */
  void AutoRepeatSendCheck();
  /**
   * @brief An endless loop for receiving package and push the package into a circular buffer
   * @details 1. Get the package container,
   *          2. Check if the pair of command set and id for receiving package exists using a map,
   *             if not create one circular buffer for this,
   *          3. Push the package container to the circular buffer
   *          Dispath layer is interacted with protocol layer
   *          by the map from the pair of command set and id to circular buffer.
   */
  void ReceivePool();
  /**
   * @brief An interface function for dispatch layer to take the message/package from the circular buffer in the protocol layer
   * @param command_info Input expected command information
   * @param message_header Output message Header
   * @param message_data Output message data
   * @return True if the message/package is taken successfully from buffer,
   *         false if the buffer is empty or
   *         input command information mismatches the information with the same command set and id already in the circular buffer
   */
  bool Take(const CommandInfo *command_info,
            MessageHeader *message_header,
            void *message_data);
  /**
   * @brief An interface function for dispatch layer to send ack in the protocol layer
   * @param command_info Input command information
   * @param message_header Input message header
   * @param message_data Input message data
   * @return true if ack is successfully allocated and sent by protocol layer
   */
  bool SendResponse(const CommandInfo *command_info,
                    const MessageHeader *message_header,
                    void *message_data);
  /**
  * @brief An interface function for dispatch layer to send cmd with need for ack in the protocol layer
  * @param command_info Input command information
  * @param message_header Get message header
  * @param message_data Input message data
  * @return true if command is successfully allocated and sent by protocol layer
  */
  bool SendRequest(const CommandInfo *command_info,
                   MessageHeader *message_header,
                   void *message_data);
  /**
  * @brief An interface function for dispatch layer to send cmd without need for ack in the protocol layer
  * @param command_info Input command information
  * @param message_data Input message data
  * @return True if command is successfully allocated and sent by protocol layer
  */
  bool SendMessage(const CommandInfo *command_info,
                   void *message_data);
  /*************************** Send Pipline ***************************/
  /**
   * @brief Assign and send command in the protocol layer
   * @param cmd_set Command set for different modules
   * @param cmd_id  Command id for different commands
   * @param receiver Receiver address
   * @param data_ptr Pointer for the data head address
   * @param data_length Length of data
   * @param session_mode Session mode to distinguish whether the command need for ack
   * @param message_header Return message header, if necessary
   * @param ack_timeout Timeout duration to check ack status in AutoRepeatSendCheck. Invalid if no need for ack
   * @param retry_time Retry time given to retry sending command in AutoRepeatSendCheck. Invalid if no need for ack
   * @return True if command is successfully allocated and sent
   */
  bool SendCMD(uint8_t cmd_set, uint8_t cmd_id, uint8_t receiver,
               void *data_ptr, uint16_t data_length,
               CMDSessionMode session_mode, MessageHeader* message_header = nullptr,
               std::chrono::milliseconds ack_timeout = std::chrono::milliseconds(50), int retry_time = 5);
  /**
   * @brief Assign and send ack in the protocol layer
   * @param session_id Session id for allocate the ack session corresponding to the command session id
   * @param seq_num sequence number, same with the command package
   * @param receiver Receiver address, same with the command package sender
   * @param ack_ptr Pointer for the ack data head address
   * @param ack_length Length of ack data
   * @return True if ack is successfully allocated and sent
   */
  bool SendACK(uint8_t session_id, uint16_t seq_num, uint8_t receiver,
               void *ack_ptr, uint16_t ack_length);
  /**
   * @brief Use hardware interface in the hardware layer to send the data
   * @param buf pointer for the buffer head
   * @return True if the buffer is successfully sent, blocked to retry connection
   *         if the hardware device is disconnected after connection.
   */
  bool DeviceSend(uint8_t *buf);

  /*************************** Recv Pipline ***************************/
  /**
   * @brief Get the byte from the device receive interface into the receive buffer and put it into byte handler, return if a full frame is got
   * @details Receive process consists of following process
   *          1. Get the Byte from the device read
   *          2. Push it into the stream
   *          3. Verify the stream for the header first.If validated,
   *             then verify the whole package when stream length equal header->length
   *          4. If whole package is validated, then get the stream into container handler for package resolving
   *             to get the receiving container in terms of ack and command
   *          5. After whole package resolving, prepare the stream for the next loop,
   *          6. Return the container if resolving successfully
   * @return Receive Container
   */
  RecvContainer *Receive();
  /**
   * @brief Input the byte and put it into stream handler with reuse mechanism
   * @param byte Input byte from Receive()
   * @return True if a full frame is got
   */
  bool ByteHandler(const uint8_t byte);
  /**
   * @brief Input the byte from the byte handler, put it into the reuse stream and check if the stream valid for a frame
   * @param byte Input byte from ByteHandler()
   * @return True if a full frame is got
   */
  bool StreamHandler(uint8_t byte);
  /**
   * @brief Check the stream to verify the header and data for the certain length
   * @return True if a full frame is got
   */
  bool CheckStream();
  /**
   * @brief Verify if it is a header.
   * @details Validate the sof, version, receiver, length and header crc. If success, just go ahead. If not, shift the stream
   * @return True if a full frame is got (Currently always false because there is no header only package)
   */
  bool VerifyHeader();
  /**
   * @brief Verify if it is a full package.
   * @details Validate the data crc for whole package. If success, just put it into conatiner handler and then prepare the stream.
   *          If not, reuse the stream
   * @return True if a full frame is got
   */
  bool VerifyData();
  /**
   * @brief Resolve the package from the stream, classify the ack and command, and get the container.
   * @return True if a full frame is got
   */
  bool ContainerHandler();

  /*************************** Stream Management**********************/
  /**
   * @brief Prepare the stream to only keep last (header length - 1) data remaining
   * @details Used after resolving the package and Getting the container
   */
  void PrepareStream();
  /**
   * @brief Shift the stream to throw the oldest byte in the stream
   * @details Used after the header is validated failed
   */
  void ShiftStream();
  /**
   * @brief this function will move the data part to buffer end,
   * @details head part will move left
   *          1. there no re-use data
   *          |------------------------------------------| <= cache
   *                               ^
   *                               reuse_index
   *          [12345678][ data Part ]--------------------| 1. stream_ptr
   *          [12345678]---------------------[ data Part ] 2. move data to end
   *          [2345678]----------------------[ data Part ] 3. forward head
   *          [2345678]------------[ data need to re-use ] 4. final mem layout
   *
   *          2. already has re-use data
   *          |---------------------------------[rev data] <= cache
   *                          ^
   *                          reuse_index, the data already used
   *          [12345678][ data Part ]-----------[rev data] 1. stream_ptr
   *          [12345678]-----------[ data Part ][rev data] 2. move data to end
   *          [2345678]------------[ data Part ][rev data] 3. forward head
   *          [2345678]------------[ data need to re-use ] 4. final mem layout
   *
   *          the re-use data will loop later
   */
  void ReuseStream();
  /************************ Session Management ***********************/
  /**
   * @brief Setup the command and ack session for initialization
   */
  void SetupSession();
  /**
   * @brief Allocate the command session
   * @param session_mode Input session mode
   * @param size Input the size
   * @return Pointer of the command session
   */
  CMDSession *AllocCMDSession(CMDSessionMode session_mode, uint16_t size);
  /**
   * @brief Free the command session
   * @param session Input the pointer of command session to be freed
   */
  void FreeCMDSession(CMDSession *session);
  /**
   * @brief Allocate the ack session
   * @param receiver Input the receiver address
   * @param session_id Input the session id
   * @param size Input the size
   * @return Pointer of the ack session
   */
  ACKSession *AllocACKSession(uint8_t receiver, uint16_t session_id, uint16_t size);
  /**
 * @brief Free the ack session
 * @param session Input the pointer of ack session to be freed
 */
  void FreeACKSession(ACKSession *session);

  /******************* CRC Calculationns ***************************/
  /**
   * @brief Update CRC16
   * @param crc Input CRC16 to be updated
   * @param ch Input data byte
   * @return Updated CRC16
   */
  uint16_t CRC16Update(uint16_t crc, uint8_t ch);
  /**
   * @brief Update CRC32
   * @param crc Input CRC32 to be updated
   * @param ch Input data byte
   * @return Updated CRC32
   */
  uint32_t CRC32Update(uint32_t crc, uint8_t ch);
  /**
   * @brief Calculate CRC16 with input data
   * @param data_ptr Input pointer of data head
   * @param length Input data length
   * @return CRC16
   */
  uint16_t CRC16Calc(const uint8_t *data_ptr, size_t length);
  /**
 * @brief Calculate CRC32 with input data
 * @param data_ptr Input pointer of data head
 * @param length Input data length
 * @return CRC32
 */
  uint32_t CRC32Calc(const uint8_t *data_ptr, size_t length);
  /**
   * @brief Check if the calculated header CRC16 is same with CRC16 in the header
   * @param data_ptr Input pointer of data head
   * @param length Input data length
   * @return True if header CRC16 is validated successfully
   */
  bool CRCHeadCheck(uint8_t *data_ptr, size_t length);
  /**
   * @brief Check if the calculated header CRC32 is same with CRC32 in the package tail
   * @param data_ptr Input pointer of data head
   * @param length Input data length
   * @return True if tail CRC32 is validated successfully
   */
  bool CRCTailCheck(uint8_t *data_ptr, size_t length);
  /******************* Const List ***************************/

  //! rate of buffer reading
  static const int    READING_RATE = 8000;
  //! size of receive buffer used to read from hardware device
  static const size_t BUFFER_SIZE = 4096;
  //! max Size of package
  static const size_t MAX_PACK_SIZE = 4096;
  //! session number for a sender/receiver
  static const size_t SESSION_TABLE_NUM = 32;
  //! length of header
  static const size_t HEADER_LEN = sizeof(Header);
  //! length of CRC16
  static const size_t CRC_HEAD_LEN = sizeof(uint16_t);
  //! length of CRC32
  static const size_t CRC_DATA_LEN = sizeof(uint32_t);
  //! length of the pair of command id and command set
  static const size_t CMD_SET_PREFIX_LEN = 2 * sizeof(uint8_t);

  //! SOF
  static const uint8_t SOF = 0xAA;
  //! version
  static const uint8_t VERSION = 0x00;
  //! local device address
  static const uint8_t DEVICE = 0x00;
  //! max number of receiver address
  static const uint8_t RECEIVER_NUM = 6;

 private:
  //! shared pointer of serial device
  std::shared_ptr<SerialDevice> serial_device_ptr_;
  //! shared pointer of memory pool
  std::shared_ptr<MemoryPool> memory_pool_ptr_;

  //! sequence number
  uint16_t seq_num_;
  //! minimum of ack timeout according to the transmission delay between two devices
  std::chrono::milliseconds poll_tick_;

  //! command session table
  CMDSession cmd_session_table_[SESSION_TABLE_NUM];
  //! ack session table
  ACKSession ack_session_table_[RECEIVER_NUM][SESSION_TABLE_NUM - 1];

  //! pointer of receive buffer
  uint8_t *recv_buff_ptr_;
  //! read position in the receive buff
  uint16_t recv_buff_read_pos_;
  //! length of data read in the receive buffer
  uint16_t recv_buff_read_len_;

  //! whether or not support large data that is larger than length of receive buffer
  bool is_large_data_protocol_;
  //! reuse buffer
  bool reuse_buffer_;

  //! pointer of receive stream
  RecvStream *recv_stream_ptr_;
  //! pointer of receive container
  RecvContainer *recv_container_ptr_;

  //! map from the pair of command set and id, to the circular buffer of receive container
  std::map<std::pair<uint8_t, uint8_t>, std::shared_ptr<CircularBuffer<RecvContainer>>> buffer_pool_map_;
  //! if receive pool should run
  std::atomic<bool> running_;

  //! automatic repeat send thread
  std::thread send_poll_thread_;
  //! receive pool thread
  std::thread receive_pool_thread_;
  //! mutex for map
  std::mutex mutex_;
};
}
#endif //ROBORTS_SDK_PROTOCOL_H
