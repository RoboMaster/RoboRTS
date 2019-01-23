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

#pragma pack(push, 1)
//DEVICE_ADDRESS
#define MANIFOLD2_ADDRESS              (0x00u)
#define CHASSIS_ADDRESS                (0X01u)
#define GIMBAL_ADDRESS                 (0X02u)
#define BROADCAST_ADDRESS              (0Xffu)
                                     
//CMD_SET                            
#define UNIVERSAL_CMD_SET              (0x00u)
#define REFEREE_CMD_SET                (0x01u)
#define CHASSIS_CMD_SET                (0x02u)
#define GIMBAL_CMD_SET                 (0x03u)
#define COMPATIBLE_CMD_SET             (0x04u)
#define TEST_CMD_SET                   (0xFFu)

/*----------------------------UNIVERSAL_CMD--- 0x00 ---------------------*/


/*-----------------------------REFEREE_CMD---- 0x01 ---------------------*/


/*-----------------------------CHASSIS_CMD---- 0x02 ---------------------*/

#define CMD_PUSH_CHASSIS_INFO          (0X04u)
typedef struct {
  int16_t gyro_angle;
  int16_t gyro_rate;
  int32_t position_x_mm;
  int32_t position_y_mm;
  int16_t angle_deg;
  int16_t v_x_mm;
  int16_t v_y_mm;
} cmd_chassis_info;

#define CMD_SET_CHASSIS_SPEED          (0X06u)
typedef struct {
  int16_t vx;
  int16_t vy;
  int16_t vw;
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
} cmd_chassis_speed;

#define CMD_GET_CHASSIS_PARAM          (0X07u)
typedef struct {
  uint16_t wheel_perimeter;
  uint16_t wheel_track;
  uint16_t wheel_base;
  int16_t gimbal_x_offset;
  int16_t gimbal_y_offset;
} cmd_chassis_param;

#define CMD_SET_CHASSIS_SPD_ACC        (0X08u)
typedef struct {
  int16_t vx;
  int16_t vy;
  int16_t vw;
  int16_t ax;
  int16_t ay;
  int16_t wz;
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
} cmd_chassis_spd_acc;

/*-----------------------------GIMBAL_CMD---- 0x03 ---------------------*/

#define CMD_PUSH_GIMBAL_INFO           (0X04u)
typedef struct {
  uint8_t mode;
  int16_t pitch_ecd_angle;
  int16_t yaw_ecd_angle;
  int16_t pitch_gyro_angle;
  int16_t yaw_gyro_angle;
  int16_t yaw_rate;
  int16_t pitch_rate;
} cmd_gimbal_info;

#define CMD_SET_GIMBAL_MODE            (0X06u)
typedef enum {
  GYRO_CONTROL,
  CODE_CONTROL,
  G_MODE_MAX_NUM,
} gimbal_mode_e;

#define CMD_SET_GIMBAL_ANGLE           (0x08u)
typedef struct{
  union{
    uint8_t flag;
    struct {
      uint8_t yaw_mode:   1;//0 means absolute, 1 means relative;
      uint8_t pitch_mode: 1;
    } bit;
  } ctrl;
  int16_t pitch;
  int16_t yaw;
}cmd_gimbal_angle;

#define CMD_SET_FRIC_WHEEL_SPEED       (0X09u)
typedef struct{
  uint16_t left;
  uint16_t right;
} cmd_fric_wheel_speed;

#define CMD_SET_SHOOT_INFO             (0x0Au)
typedef enum {
  SHOOT_STOP = 0,
  SHOOT_ONCE,
  SHOOT_CONTINUOUS,
} shoot_cmd_e;

typedef struct {
  uint8_t shoot_cmd;
  uint32_t shoot_add_num;
  uint16_t shoot_freq;
} cmd_shoot_info;

/*------------------------COMPATIBLE_CMD---- 0x04 -------------------*/
#define CMD_RC_DATA_FORWARD            (0X01u)

#define CMD_PUSH_UWB_INFO              (0X02u)
typedef struct {
  int16_t x;
  int16_t y;
  uint16_t yaw;
  int16_t distance[6];
  uint16_t error;
  uint16_t res;
} cmd_uwb_info;

/*-----------------------------TEST_CMD---- 0xFF ---------------------*/
#define TEXT_ECHO_TRANSMIT             (0x00u)
#pragma pack(pop)
}
#endif //ROBORTS_SDK_PROTOCOL_DEFINE_H
