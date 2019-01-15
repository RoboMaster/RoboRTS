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
#ifndef ROBORTS_SDK_PROTOCOL_DEFINE_H
#define ROBORTS_SDK_PROTOCOL_DEFINE_H

namespace roborts_sdk {
//DEVICE_ADDRESS
#define MANIFOLD2_ADDRESS       (0x00u)
#define CHASSIS_ADDRESS         (0X01u)
#define GIMBAL_ADDRESS          (0X02u)
#define BROADCAST_ADDRESS       (0Xffu)

//CMD_SET
#define UNIVERSAL_CMD_SET       (0x00u)
#define REFEREE_CMD_SET         (0x01u)
#define CHASSIS_CMD_SET         (0x02u)
#define GIMBAL_CMD_SET          (0x03u)
#define TEST_CMD_SET            (0xFFu)

/*----------------------------UNIVERSAL_CMD------------------------------*/
#define OPEN_SDK_MODE           (0X00u)
#define GET_MODULE_STATUS       (0X01u)
#define PUSH_STATUS_INFO        (0X02u)
#define GET_CAN_BANDWIDTH       (0X03u)

/*-----------------------------REFEREE_CMD-------------------------------*/
#define GET_REFEREE_VERSION     (0X00u)
#define GET_GAME_CONFIG         (0X01u)
#define SUBSCRIBE_REFEREE_INFO  (0X02u)
#define PUSH_REFEREE_INFO       (0X03u)


/*-----------------------------CHASSIS_CMD-------------------------------*/

#define GET_CHASSIS_VERSION     (0x00u)
#define SET_CHASSIS_PARAM       (0x01u)
#define GET_CHASSIS_PARAM       (0X02u)
typedef struct {
  uint16_t wheel_perimeter;
  uint16_t wheel_track;
  uint16_t wheel_base;
  int16_t gimbal_x_offset;
  int16_t gimbal_y_offset;
} p_chassis_param_t;
#define SUBSCRIBE_CHASSIS_INFO  (0X03u)
#define PUSH_CHASSIS_INFO       (0X04u)
typedef struct {
  uint8_t mode;
  float yaw_motor_angle;
  float gyro_angle;
  float gyro_palstance;

  int32_t position_x_mm;
  int32_t position_y_mm;
  int32_t angle_deg;
  int16_t v_x_mm;
  int16_t v_y_mm;
  int16_t palstance_deg;
} p_chassis_info_t;

#define SET_CHASSIS_MODE        (0X05u)
typedef enum {
  FOLLOW_GIMBAL,
  SEPARATE_GIMBAL,
  SEARCH_MODE,
  C_MODE_MAX_NUM,
} chassis_mode_e;

#define CTRL_CHASSIS_SPEED      (0X06u)
typedef struct {
  float vx;
  float vy;
  float vw;
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
  uint16_t res;
} p_chassis_speed_t;

#define PUSH_UWB_INFO           (0X07u)
typedef struct {
  int16_t x;
  int16_t y;
  uint16_t yaw;
  int16_t distance[6];
  uint16_t error;
  uint16_t res;
} p_uwb_data_t;

#define CTRL_CHASSIS_SPEED_ACC   (0X08u)
typedef struct {
  float vx;
  float vy;
  float vw;

  float ax;
  float ay;
  float wz;
} p_chassis_spd_acc_t;

/*-----------------------------GIMBAL_CMD-------------------------------*/

#define GET_GIMBAL_VERSION      (0x00u)
#define SET_GIMBAL_PARAM        (0x01u)
#define GET_GIMBAL_PARAM        (0X02u)
#define SUBSCRIBE_GIMBAL_INFO   (0X03u)
#define PUSH_GIMBAL_INFO        (0X04u)
typedef struct {
  uint8_t mode;
  uint16_t fric_spd;
  uint32_t shoot_num;

  float pit_relative_angle;
  float yaw_relative_angle;
  float pit_gyro_angle;
  float yaw_gyro_angle;

  float yaw_palstance;
  float pit_palstance;
} p_gimbal_info_t;

#define ADJUST_GIMBAL_CENTER    (0X05u)
#define SET_GIMBAL_MODE         (0X06u)
typedef enum {
  GYRO_CONTROL,
  CODE_CONTROL,
  G_MODE_MAX_NUM,
} gimbal_mode_e;

#define CTRL_GIMBAL_RATE       (0X07u)
typedef struct {
  float pit_rate;
  uint16_t pit_time;
  float yaw_rate;
  uint16_t yaw_time;
} p_gimbal_speed_t;

//#define CTRL_GIMBAL_ANGLE      (0X08u)
//typedef struct {
//  float pit;
//  float yaw;
//} p_gimbal_angle_t;

#define CTRL_FRIC_WHEEL_SPEED   (0X09u)
typedef uint16_t p_fric_wheel_speed_t;

#define CTRL_GIMBAL_SHOOT       (0x0Au)
typedef enum {
  SHOOT_STOP = 0,
  SHOOT_ONCE,
  SHOOT_CONTINUOUS,
} shoot_cmd_e;

typedef struct {
  uint8_t shoot_cmd;
  uint32_t shoot_add_num;
  uint16_t shoot_freq;
} p_gimbal_shoot_t;

#define CTRL_GIMBAL_ANGLE       (0x0Bu)
typedef struct{
  union{
    uint8_t flag;
    struct {
      uint8_t yaw_mode:   1;//0 means absolute, 1 means relative;
      uint8_t pitch_mode: 1;
    } bit;
  } ctrl;
  float pit_absolute;
  float pit_relative;
  float yaw_absolute;
  float yaw_relative;
}p_gimbal_angle_t;

//TEST_CMD
#define TEXT_ECHO_TRANSMIT      (0x00u)

}
#endif //ROBORTS_SDK_PROTOCOL_DEFINE_H
