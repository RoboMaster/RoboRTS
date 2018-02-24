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

#ifndef MODULES_DRIVER_SERIAL_INFANTRY_INFO_H
#define MODULES_DRIVER_SERIAL_INFANTRY_INFO_H

#include <stdio.h>
#include <stdint.h>

namespace rrts {
namespace driver {
namespace serial {

#define UP_REG_ID    0xA0
#define DN_REG_ID    0xA5
#define HEADER_LEN   sizeof(FrameHeader)
#define CMD_LEN      2
#define CRC_LEN      2
#define PROTOCAL_FRAME_MAX_SIZE  200
#define UART_BUFF_SIZE           2000
#define PACK_MAX_SIZE            200
#define COMPRESS_TIME            1
/**
  * @brief  command id
  */
typedef enum {
  GAME_INFO_ID = 0x0001,
  REAL_BLOOD_DATA_ID = 0x0002,
  REAL_SHOOT_DATA_ID = 0x0003,
  REAL_RFID_DATA_ID = 0x0005,
  GAME_RESULT_ID = 0x0006,
  GAIN_BUFF_ID = 0x0007,

  CHASSIS_DATA_ID = 0x0010,
  GIMBAL_DATA_ID = 0x0011,
  SHOOT_TASK_DATA_ID = 0x0012,
  INFANTRY_ERR_ID = 0x0013,
  CONFIG_RESPONSE_ID = 0x0014,
  CALI_RESPONSE_ID = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID = 0x0017,

  CHASSIS_CTRL_ID = 0x00A0,
  GIMBAL_CTRL_ID = 0x00A1,
  SHOOT_CTRL_ID = 0x00A2,
  ERROR_LEVEL_ID = 0x00A3,
  INFANTRY_STRUCT_ID = 0x00A4,
  CALI_GIMBAL_ID = 0x00A5,

  CLIENT_SHOW_ID = 0x0100,
  USER_TO_SERVER_ID = 0x0101,
  SERVER_TO_USER_ID = 0x0102,
} CommandID;

/* enumeration type data */

typedef enum {
  STEP_HEADER_SOF = 0,
  STEP_LENGTH_LOW = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16 = 5,
} UnpackStep;

typedef enum {
  DEVICE_NORMAL = 0,
  ERROR_EXIST = 1,
  UNKNOWN_STATE = 2,
} BottomError;

typedef enum {
  BOTTOM_DEVICE_NORMAL = 0,
  GIMBAL_GYRO_OFFLINE = 1,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE = 3,
  CHASSIS_M2_OFFLINE = 4,
  CHASSIS_M3_OFFLINE = 5,
  CHASSIS_M4_OFFLINE = 6,
  REMOTE_CTRL_OFFLINE = 7,
  JUDGE_SYS_OFFLINE = 8,
  GIMBAL_YAW_OFFLINE = 9,
  GIMBAL_PIT_OFFLINE = 10,
  TRIGGER_MOTO_OFFLINE = 11,
  BULLET_JAM = 12,
  CHASSIS_CONFIG_ERR = 13,
  GIMBAL_CONFIG_ERR = 14,
  ERROR_LIST_LENGTH = 15,
} ErrorID;

typedef enum {
  GLOBAL_NORMAL = 0,
  SOFTWARE_WARNING = 1,
  SOFTWARE_ERROR = 2,
  SOFTWARE_FATAL_ERROR = 3,
  GIMBAL_ERROR = 4,
  CHASSIS_ERROR = 5,
  HARAWARE_ERROR = 6,
} ErrorLevel;

typedef enum {
  NO_CONFIG = 0,
  DEFAULT_CONFIG = 1,
  CUSTOM_CONFIG = 3,
} StructConfig;

typedef enum {
  GIMBAL_RELAX = 0,
  GIMBAL_INIT = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO = 3,
  GIMBAL_TRACK_ARMOR = 4,
  GIMBAL_PATROL_MODE = 5,
  GIMBAL_SHOOT_BUFF = 6,
  GIMBAL_POSITION_MODE = 7,
} GimbalMode;

typedef enum {
  CHASSIS_RELAX = 0,
  CHASSIS_STOP = 1,
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL = 3,
  DODGE_MODE = 4,
  AUTO_SEPARATE_GIMBAL = 5,
  AUTO_FOLLOW_GIMBAL = 6,
} ChassisMode;

typedef enum {
  SHOT_DISABLE = 0,
  REMOTE_CTRL_SHOT = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT = 4,
} ShootMode;

/**
  * @brief  frame header structure definition
  */
typedef struct {
  uint8_t sof;
  uint16_t data_length;
  uint8_t seq;
  uint8_t crc8;
} __attribute__((packed)) FrameHeader;


/********************** the judgement system from bottom main control **********************/

/**
  * @brief  GPS state structures definition
  */
typedef struct {
  uint8_t valid_flag;
  float x;
  float y;
  float z;
  float yaw;
} __attribute__((packed)) Position;

/**
  * @brief  game information structures definition(0x0001)
  */
typedef struct {
  uint16_t stage_remain_time;
  uint8_t game_process;
  /* current race stage
   0 not start
   1 preparation stage
   2 self-check stage
   3 5 seconds count down
   4 fighting stage
   5 result computing stage */
  uint8_t reserved;
  uint16_t remain_hp;
  uint16_t max_hp;
  Position position;
} __attribute__((packed)) GameInfo;

/**
  * @brief  real time blood volume change data(0x0002)
  */
typedef struct {
  uint8_t armor_type:4;
  /* 0-3bits: the attacked armor id:
     0x00: 0 front
     0x01: 1 left
     0x02: 2 behind
     0x03: 3 right
     others reserved*/
  uint8_t hurt_type:4;
  /* 4-7bits: blood volume change type
     0x00: armor attacked
     0x01: module offline
     0x02: bullet over speed
     0x03: bullet over frequency */
} __attribute__((packed)) HurtData;

/**
  * @brief  real time shooting data(0x0003)
  */
typedef struct {
  uint8_t reserved1;
  uint8_t bullet_freq;
  float bullet_speed;
  float reserved2;
} __attribute__((packed)) ShootData;

/**
  * @brief  rfid detect data(0x0005)
  */
typedef struct {
  uint8_t card_type;
  uint8_t card_idx;
} __attribute__((packed)) RfidData;

/**
  * @brief  game result data(0x0006)
  */
typedef struct {
  uint8_t winner;
} __attribute__((packed)) GameResult;

/**
  * @brief  the data of get field buff(0x0007)
  */
typedef struct {
  uint8_t buff_type;
  uint8_t buff_addition;
} __attribute__((packed)) GameBuff;

/**
  * @brief  student custom data(0x0100)
  */
typedef struct {
  float data1;
  float data2;
  float data3;
} __attribute__((packed)) ClientData;

/**
  * @brief  student custom data(0x0101)
  */
typedef struct {
  uint8_t data[64];
} __attribute__((packed)) UserToServer;

/**
  * @brief  student custom data(0x0102)
  */
typedef struct {
  uint8_t data[32];
} __attribute__((packed)) ServerToUser;


/********************** the infantry information from bottom main control **********************/

/**
  * @brief  chassis information(0x0010)
  */
typedef struct {
  uint8_t ctrl_mode;      /* chassis control mode */
  float gyro_palstance; /* chassis palstance(degree/s) from gyroscope */
  float gyro_angle;     /* chassis angle(degree) relative to ground from gyroscope */
  float ecd_palstance;  /* chassis palstance(degree/s) from chassis motor encoder calculated */
  float ecd_angle; /* chassis angle(degree) relative to ground from chassis motor encoder calculated */
  int16_t x_speed;        /* chassis x-axis move speed(mm/s) from chassis motor encoder calculated */
  int16_t y_speed;        /* chassis y-axis move speed(mm/s) from chassis motor encoder calculated */
  int32_t x_position;     /* chassis x-axis position(mm) relative to the starting point */
  int32_t y_position;     /* chassis y-axis position(mm) relative to the starting point */
} __attribute__((packed)) ChassisInfo;

/**
  * @brief  gimbal information(0x0011)
  */
typedef struct {
  uint8_t ctrl_mode;          /* gimbal control mode */
  float pit_relative_angle; /* pitch angle(degree) relative to the gimbal center */
  float yaw_relative_angle; /* yaw angle(degree) relative to the gimbal center */
  float pit_absolute_angle; /* pitch angle(degree) relative to ground */
  float yaw_absolute_angle; /* yaw angle(degree) relative to ground */
  float pit_palstance;      /* pitch axis palstance(degree/s) */
  float yaw_palstance;      /* yaw axis palstance(degree/s) */
} __attribute__((packed)) GimbalInfo;

/**
  * @brief  shoot information(0x0012)
  */
typedef struct {
  int16_t remain_bullet;  /* the member of remain bullets */
  int16_t sent_bullet;    /* the member of bullets that have been sent */
  uint8_t fric_wheel_run; /* friction run or not */
} __attribute__((packed)) ShootInfo;

/**
  * @brief  infantry error information(0x0013)
  */
typedef struct {
  BottomError err_sta;                 /* bottom error state */
  BottomError err[ERROR_LIST_LENGTH];  /* device error list */
} __attribute__((packed)) InfantryError;

/**
  * @brief  infantry structure config response(0x0014)
  */
typedef struct {
  StructConfig chassis_config;
  StructConfig gimbal_config;
} __attribute__((packed)) ConfigMessage;

/**
  * @brief  gimbal calibrate response(0x0015)
  */
typedef struct {
  uint8_t type;     //0x01 success 0x00 fault
  int16_t yaw_offset;
  int16_t pitch_offset;
} __attribute__((packed)) CalibrateResponse;


/**
  * @brief  remote control information(0x0016)
  */
typedef struct {
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* Mouse movement and button information */
  struct {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } __attribute__((packed)) Mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    } __attribute__((packed)) Bit;
  } __attribute__((packed)) Keyboard;
} __attribute__((packed)) RcInfo;

/**
  * @brief  bottom software version information(0x0017)
  */
typedef struct {
  uint8_t num[4];
} __attribute__((packed)) VersionInfo;

/************************ the control information from computer ************************/

typedef struct {
  int16_t x_offset;   /* offset(mm) relative to the x-axis of the chassis center */
  int16_t y_offset;   /* offset(mm) relative to the y-axis of the chassis center */
  float w_speed;    /* rotation speed(degree/s) of chassis */
} __attribute__((packed)) ChassisRotate;

/**
  * @brief  chassis control information(0x00A0)
  */
typedef struct {
  uint8_t ctrl_mode; /* chassis control mode */
  int16_t x_speed;   /* x-axis move speed(mm/s) of chassis */
  int16_t y_speed;   /* y-axis move speed(mm/s) of chassis */
  ChassisRotate w_info;    /* rotation control of chassis */
} __attribute__((packed)) ChassisControl;

/**
  * @brief  gimbal control information(0x00A1)
  */
typedef struct {
  uint8_t ctrl_mode;    /* gimbal control mode */
  float pit_ref;      /* gimbal pitch reference angle(degree) */
  float yaw_ref;      /* gimbal yaw reference angle(degree) */
  uint8_t visual_valid; /* visual information valid or not */
} __attribute__((packed)) GimbalControl;

/**
  * @brief  shoot control information(0x00A2)
  */
typedef struct {
  uint8_t shoot_cmd;      /* single shoot command */
  uint8_t c_shoot_cmd;    /* continuous shoot command */
  uint8_t fric_wheel_run; /* friction run or not */
  uint8_t fric_wheel_spd; /* fricrion wheel speed */
} __attribute__((packed)) ShootControl;

/**
  * @brief  robot system error level(0x00A3)
  */
typedef struct {
  ErrorLevel err_level;  /* the error level is included in ErrorLevel enumeration */
} __attribute__((packed)) GlobalErrorLevel;

/**
  * @brief  infantry structure configuration information(0x00A4)
  */
typedef struct {
  StructConfig chassis_config;  /* chassis structure config state */
  uint16_t wheel_perimeter; /* the perimeter(mm) of wheel */
  uint16_t wheel_track;     /* wheel track distance(mm) */
  uint16_t wheel_base;      /* wheelbase distance(mm) */
  StructConfig gimbal_config;   /* gimbal structure config state */
  int16_t gimbal_x_offset; /* gimbal offset(mm) relative to the x-axis of the chassis center */
  int16_t gimbal_y_offset; /* gimbal offset(mm) relative to the y-axis of the chassis center */
} __attribute__((packed)) InfantryStructure;

/**
  * @brief  gimbal calibrate command(0x00A5)
  */
typedef struct {
  uint8_t type;        /* 0x01 calibrate gimbal center, 0x02 calibrate camera */
} __attribute__((packed)) CalibrateCommand;
} //namespace serial
} //namespace driver
} //namespace rrts
#endif //MODULES_DRIVER_SERIAL_INFANTRY_INFO_H