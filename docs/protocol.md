## Protocol Specifications

### Protocol Data

Protocol data can be divided into two major categories in terms of communication direction:

#### I. Data transmitted from an underlying layer to an upper layer:

1. Feedback data, including feedback data from sensors of each mechanism and data calculated from an underlying layer;

2. Underlying layer status data, including run time status of an underlying device and some responses of an underlying layer to upper layer data;

3. Forward data, including all data of the Referee System as well as customized data on the server side;

#### II. Data received by an underlying layer from an upper layer:

1. Control data: data used by an upper layer to control 3 actuating mechanisms on an underlying layer;

2. Configuration data: structure configuration data such as the wheel track, wheel base and gimbal position of a robot on an upper layer, as well as both the run time status of an upper layer program, and underlying layer response levels;

3. Forward data: user-customized data that must be forwarded to the Referee System from the underlying layer and finally displayed on the client side;

### Functional Modules

#### Manual mode:

Allows basic controls in remote control and mouse & keyboard mode. If some auxiliary functions, such as buff and auxiliary launching, are enabled in Manual mode, upper layer protocol communication must be enabled to obtain related data from an upper layer PC to implement these functions.

Some upper layer debug interfaces are provided in Manual mode to facilitate testing and verification when debugging upper layer PC programs.

#### Full Auto mode:

In this mode, a chassis, gimbal and launching mechanism are fully controlled by an upper layer PC, including controlling these actuating mechanisms in a specific physical unit and also mode switching of these actuating mechanisms.

#### Introduction to operating gears:

##### 1. Manual gear

Control data, debug, and automatic gear isolation (place the driving lever to the upper right)

Control with a remote controller: place the driving lever to upper right

- Turn on/off a friction wheel
- Single shot, automatic firing

Control with a mouse & keyboard: (place the driving lever to left center, upper right)

- Turn on/off a friction wheel
- Single shot, automatic firing
- Twist the waist to dodge

##### 2. Debug gear

Used during robot debugging (place the driving lever to right center)

- Twist the waist (upper left)
- Armor tracking, not with the chassis (left center)
- Armor tracking, with the chassis (lower left)
- A friction wheel inherits the status in manual or auto mode
- Upper layer control of launching

##### 3. Automatic gear

Used in competition (place the driving lever to the lower right)

- Full automatic control (3 mechanisms completely trust control from an upper layer except for when the upper layer encounters a fatal_error)




## Introduction to Operating Mode

*Note:*

The parts in bold refer to operating modes of modules available in auto mode. Note the initial value of a mode when control data is sent from a PC.

### Gimbal

```c
typedef enum
{
  GIMBAL_RELAX         = 0,
  GIMBAL_INIT          = 1,
  GIMBAL_NO_ARTI_INPUT = 2,
  GIMBAL_FOLLOW_ZGYRO  = 3,
  GIMBAL_TRACK_ARMOR   = 4,
  GIMBAL_PATROL_MODE   = 5,
  GIMBAL_SHOOT_BUFF    = 6,
  GIMBAL_POSITION_MODE = 7,
} gimbal_mode_e;
```

| Gimbal Mode              | Function                                 |
| ------------------------ | ---------------------------------------- |
| GIMBAL_RELAX             | Gimbal powers off                        |
| GIMBAL_INIT              | Gimbal is being restored from the power off status |
| GIMBAL_NO_ARTI_INPUT     | No manual control data input mode available |
| GIMBAL_FOLLOW_ZGYRO      | The mode in which the gimbal follows the chassis |
| **GIMBAL_TRACK_ARMOR**   | Gimbal tracks armor                      |
| **GIMBAL_PATROL_MODE**   | Patrol mode, the gimbal yaws periodically, pitch uncontrolled |
| **GIMBAL_SHOOT_BUFF**    | Shooting buff mode                       |
| **GIMBAL_POSITION_MODE** | Gimbal position mode, angle between two axes controlled on an upper layer |

### Chassis

```c
typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL   = 3,
  DODGE_MODE             = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
} chassis_mode_e;
```

| Chassis mode             | Function                                 |
| ------------------------ | ---------------------------------------- |
| CHASSIS_RELAX            | Chassis powers off                       |
| CHASSIS_STOP             | Chassis stops/brakes                     |
| MANUAL_SEPARATE_GIMBAL   | The chassis is separated from the gimbal in manual mode |
| MANUAL_FOLLOW_GIMBAL     | The chassis follows the gimbal in manual mode |
| **DODGE_MODE**           | Chassis dodge mode, the chassis rotates around a fixed point while panning is uncontrolled |
| **AUTO_SEPARATE_GIMBAL** | The chassis is separated from the gimbal, and rotation and panning are controlled on an upper layer |
| **AUTO_FOLLOW_GIMBAL**   | Automatically follow the gimbal; panning is controlled on an upper layer |

### Launching Mechanism

```c
typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,
} shoot_mode_e;
```

| Launching Mechanism Mode | Function                                 |
| ------------------------ | ---------------------------------------- |
| SHOT_DISABLE             | Launching mechanism power off            |
| REMOTE_CTRL_SHOT         | A remote control is used to control a launching mechanism |
| KEYBOARD_CTRL_SHOT       | A keyboard is used to control a launching mechanism |
| SEMIAUTO_CTRL_SHOT       | Single shot or automatic firing is controlled on an upper layer |
| **AUTO_CTRL_SHOT**       | Friction wheel rune, speed, single shot and automatic firing are fully controlled on an upper layer |



## Introduction to Protocol Data

A frame of protocol data is divided into 4 parts, including frame header, command code ID, data and frame footer check data.

### 1. Frame Header

``` c
/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;
```


| Frame Header | Number of Bytes | Detailed Description                     |
| :----------- | :-------------- | :--------------------------------------- |
| sof          | 1               | Domain ID of data, domain for main control module and Referee System: 0xA5; upper layer domain for main control module and PC: 0xA0 |
| data_length  | 2               | Data length per frame                    |
| seq          | 1               | Packet sequence number, retained in the 0xA0 domain |
| crc8         | 1               | CRC check result of frame header         |

*Note:*

Data domain ID is mainly divided into upper layer domain 0xA0 and underlying layer domain 0xA5.

The 0xA5 domain is the communication domain ID of the Referee System and main control module of a robot and is mainly filled with game data from the Referee System and user-customized data.

The data in the 0xA0 domain is the communication data from an upper layer PC and main control module, and mainly includes control data of the PC against an underlying layer and feedback data from the underlying layer, as well as the data forwarded from the main control module to the upper layer Referee System.

### 2. Command Code

| Command Code | Number of Bytes |
| :----------- | :-------------- |
| cmdid        | 2               |

A command code corresponds to a frame of data containing specific information. The following shows command codes corresponding to all existing data.

*Note:*As the main control module of a robot forwards data from the Referee System to an upper layer PC, 0xA0 contains all information in the 0xA5 domain.

``` c
typedef enum
{
  GAME_INFO_ID        = 0x0001,
  REAL_BLOOD_DATA_ID  = 0x0002,
  REAL_SHOOT_DATA_ID  = 0x0003,
  REAL_FIELD_DATA_ID  = 0x0005,
  GAME_RESULT_ID      = 0x0006,
  GAIN_BUFF_ID        = 0x0007,
  
  CHASSIS_DATA_ID     = 0x0010,
  GIMBAL_DATA_ID      = 0x0011,
  SHOOT_TASK_DATA_ID  = 0x0012,
  INFANTRY_ERR_ID     = 0x0013,
  CONFIG_RESPONSE_ID  = 0x0014,
  CALI_RESPONSE_ID    = 0x0015,
  REMOTE_CTRL_INFO_ID = 0x0016,
  BOTTOM_VERSION_ID   = 0x0017,
  
  CHASSIS_CTRL_ID     = 0x00A0,
  GIMBAL_CTRL_ID      = 0x00A1,
  SHOOT_CTRL_ID       = 0x00A2,
  ERROR_LEVEL_ID      = 0x00A3,
  INFANTRY_STRUCT_ID  = 0x00A4,
  CALI_GIMBAL_ID      = 0x00A5,
  
  STU_CUSTOM_DATA_ID  = 0x0100,
  ROBOT_TO_CLIENT_ID  = 0x0101,
  CLIENT_TO_ROBOT_ID  = 0x0102,
} command_id_e;
```


Data transmission direction and specific features of command codes are as follows:

| Command Code | Transmission Direction   | Functional Description                   | Frequency                                |
| :----------- | :----------------------- | :--------------------------------------- | :--------------------------------------- |
| 0x0001       | Main control module > PC | Robot status in competition              | Referee System 10 Hz                     |
| 0x0002       | Main control module > PC | Real-time damage data                    | Transmitted when hit                     |
| 0x0003       | Main control module > PC | Real-time launching data                 | Referee system                           |
| 0x0005       | Main control module > PC | Field interaction data                   | Transmitted when an IC card is detected  |
| 0x0006       | Main control module > PC | Competition result data                  | Transmitted when competition ends        |
| 0x0007       | Main control module > PC | Obtain buff data                         | Referee System                           |
|              |                          |                                          |                                          |
| 0x0010       | Main control module > PC | Robot chassis-related information        | Fixed 50Hz                               |
| 0x0011       | Main control module > PC | Robot gimbal-related information         | Fixed 50Hz                               |
| 0x0012       | Main control module > PC | Robot launching task information         | Fixed 50Hz                               |
| 0x0013       | Main control module > PC | Robot chassis fault information          | Fixed 50Hz                               |
| 0x0014       | Main control module > PC | Robot structure configuration status feedback | Fixed 50Hz                               |
| 0x0015       | Main control module > PC | Robot gimbal calibration feedback        | Transmitted once when valid calibration information is received |
| 0x0016       | Main control module > PC | Parsed remote control information        | Fixed 50Hz                               |
| 0x0017       | Main control module > PC | Underlying software version information  | Fixed 1Hz                                |
|              |                          |                                          |                                          |
| 0x00A0       | PC > main control module | Gimbal control information               | Fixed 50Hz                               |
| 0x00A1       | PC > main control module | Chassis control information              | Fixed 50Hz                               |
| 0x00A2       | PC > main control module | Launching mechanism control information  | Fixed 50Hz                               |
| 0x00A3       | PC > main control module | Warning level when a PC encounters a runtime error | Transmitted if an error occurs           |
| 0x00A4       | PC > main control module | Robot structure configuration information | Generally during a period of time before power on |
| 0x00A5       | PC > main control module | Gimbal calibration information           | Transmitted when Gimbal calibration is required |
|              |                          |                                          |                                          |
| 0x0100       | PC > main control module | Data that must be forwarded by a PC to be displayed on a client side | Fixed frequency (Hz)                     |
| 0x0101       | PC > main control module | Data that must be forwarded by a PC to a client side | Fixed frequency (Hz)                     |
| 0x0102       | Main control module > PC | Data from a client to a PC               | Forwarded when there is data             |

*Notes:*

Command code 0x0100/0x0101 is the data transmitted by a PC to the main control module, then forwarded by the main control module to the Referee System and finally displayed on the client interface. Command code 0x0102 is the data forwarded by the main control module to a PC after operation data from a client is transmitted to the server and Referee System.

### 3. Data
The data structure corresponding to a command code ID. Data length is the size of this structure.
| Data | Number of Bytes |
| :--- | :-------------- |
| data | data_length     |

#### Class 1

##### 0x0001 game process

Corresponds to data structure game_info_t (game process information)

```c
typedef __packed struct
{
  uint16_t   stage_remain_time;
  uint8_t    game_process;
  /* current race stage
     0 not start
     1 preparation stage
     2 self-check stage
     3 5 seconds count down
     4 fighting stage
     5 result computing stage */
  uint8_t    reserved;
  uint16_t   remain_hp;
  uint16_t   max_hp;
  position_t position;
} game_robot_state_t;
```

| Data              | Description                              |
| :---------------- | ---------------------------------------- |
| stage_remain_time | Remaining time in the current round (seconds) |
| game_process      | Current stage                            |
|                   | 0: pre-competition stage                 |
|                   | 1: preparation stage                     |
|                   | 2: initialization stage                  |
|                   | 3: 5-second countdown                    |
|                   | 4: in combat                             |
|                   | 5: calculating competition results       |
| reserved          | Bits reserved                            |
| remain_hp         | Robot's current HP                       |
| max_hp            | Robot's maximum HP                       |
| position          | Position/angle information               |

*Note:*

Position/angle control information is included in the position_t structure:

```c
typedef __packed struct
{
  uint8_t valid_flag;
  float x;
  float y;
  float z;
  float yaw;
} position_t;
```

| Data       | Description                              |
| ---------- | ---------------------------------------- |
| valid_flag | Position/angle information effective zone bit |
|            | 0: Invalid                               |
|            | 1: Valid                                 |
| x          | X value of location                      |
| y          | Y value of location                      |
| z          | Z value of location                      |
| yaw        | Barrel direction angle value             |

##### 0x0002 damage data

Corresponds to the data structure robot_hurt_data_t (damage data)

```c
typedef __packed struct
{
  uint8_t armor_type:4;
 /* 0-3bits: the attacked armor id:
    0x00: 0 front
    0x01：1 left
    0x02：2 behind
    0x03：3 right
    others reserved*/
  uint8_t hurt_type:4;
 /* 4-7bits: blood volume change type
    0x00: armor attacked
    0x01：module offline
    0x02: bullet over speed
    0x03: bullet over frequency */
} robot_hurt_data_t;
```

| Data                                     | Description                              |
| ---------------------------------------- | ---------------------------------------- |
| armor_type (ID of the armor receiving an attack) | 0-3 bits: indicate the armor ID if the type of change is armor damage |
|                                          | 0x00: armor #0 (front)                   |
|                                          | 0x01: armor #1 (left)                    |
|                                          | 0x02: armor #2 (rear)                    |
|                                          | 0x03: armor #3 (right)                   |
|                                          | Other bits reserved                      |
| hurt_type (HP deduction type)            | 4-7 bits: type of HP changes             |
|                                          | 0x0: armor damage (attack received)      |
|                                          | 0x1: module offline                      |
|                                          | 0x2: projectile exceeds launching speed limit |
|                                          | 0x3: projectile exceeds launching rate limit |

##### 0x0003 real-time launching

Corresponds to the data structure real_shoot_data_t (real-time launching information)

```c
typedef __packed struct
{
  uint8_t reserved;
  uint8_t bullet_freq;
  float   bullet_speed;
  float   reserved;
} real_shoot_data_t;
```

| Data         | Description                |
| ------------ | -------------------------- |
| reserved     | Reserved                   |
| bullet_freq  | projectile launching rate  |
| bullet_speed | projectile launching speed |
| reserved     | Reserved                   |

##### 0x0005 field interaction

Corresponds to the data structure rfid_detect_t (field interaction data)

```c
typedef __packed struct
{
  uint8_t card_type;
  uint8_t card_idx;
} rfid_detect_t;
```

| Data      | Description                              |
| --------- | ---------------------------------------- |
| card_type | Card type                                |
|           | 0: Attack buff card                      |
|           | 1: Defense buff card                     |
| card_idx  | Card index number; used to distinguish different sections |

##### 0x0006 competition result

Corresponds to the data structure game_result_t (competition result)

```c
typedef __packed struct
{
  uint8_t winner;
} game_result_t;
```

| Data   | Description        |
| ------ | ------------------ |
| winner | Competition result |
|        | 0: Draw            |
|        | 1: Red team wins   |
|        | 2: Blue team wins  |

##### 0x0007 obtained buff

Corresponds to the data structure get_buff_t (obtained buff data)

```c
typedef __packed struct
{
  uint8_t buff_type;
  uint8_t buff_addition;
} get_buff_t;
```

| Data          | Description     |
| ------------- | --------------- |
| buff_type     | Buff type       |
|               | 0: Attack buff  |
|               | 1: Defense buff |
| buff_addition | Buff percentage |

#### Class 2

##### 0x0010 chassis information

Corresponds to the data structure chassis_info_t (chassis status information)

```c
typedef __packed struct
{
  uint8_t ctrl_mode;      /* chassis control mode */
  float   gyro_palstance; /* chassis palstance(degree/s) from gyroscope */
  float   gyro_angle;     /* chassis angle(degree) relative to ground from gyroscope */
  float   ecd_palstance;  /* chassis palstance(degree/s) from chassis motor encoder calculated */
  float   ecd_calc_angle; /* chassis angle(degree) relative to ground from chassis motor encoder calculated */
  int16_t x_speed;        /* chassis x-axis move speed(mm/s) from chassis motor encoder calculated */
  int16_t y_speed;        /* chassis y-axis move speed(mm/s) from chassis motor encoder calculated */
  int32_t x_position;     /* chassis x-axis position(mm) relative to the starting point */
  int32_t y_position;     /* chassis y-axis position(mm) relative to the starting point */
} chassis_info_t;
```

| Data           | Description                              |
| -------------- | ---------------------------------------- |
| ctrl_mode      | Chassis control mode                     |
| gyro_palstance | Chassis angular velocity measured by using a single-shaft module (degree/s) |
| gyro_angle     | Chassis angle measured by using a single-shaft module (degree) |
| ecd_palstance  | Chassis angular velocity calculated by using a chassis encoder (degree/s) |
| ecd_calc_angle | Chassis angle calculated by using a chassis encoder (degree) |
| x_speed        | Chassis x-axis motion speed (mm/s)       |
| y_speed        | Chassis y-axis motion speed (mm/s)       |
| x_position     | The coordinate position of the chassis x-axis relative to the origin (mm) |
| y_position     | The coordinate position of the chassis y-axis relative to the origin (mm) |

*Note:*

> Position information completely follows a right-handed coordinate system, including all position-related data of the chassis and gimbal. The x-axis of the right-handed coordinate system is the forward direction while the y-axis is the left direction.

##### 0x0011 gimbal information

Corresponds to the data structure gimbal_info_t (gimbal status information)

```c
typedef __packed struct
{
  uint8_t ctrl_mode;          /* gimbal control mode */
  float   pit_relative_angle; /* pitch angle(degree) relative to the gimbal center */
  float   yaw_relative_angle; /* yaw angle(degree) relative to the gimbal center */
  float   pit_absolute_angle; /* pitch angle(degree) relative to ground */
  float   yaw_absolute_angle; /* yaw angle(degree) relative to ground */
  float   pit_palstance;      /* pitch axis palstance(degree/s) */
  float   yaw_palstance;      /* yaw axis palstance(degree/s) */
} gimbal_info_t;
```

| Data               | Description                              |
| ------------------ | ---------------------------------------- |
| ctrl_mode          | Gimbal control mode                      |
| pit_relative_angle | The angle of the pitch axis relative to the midpoint of the gimbal (degree) |
| yaw_relative_angle | The angle of the yaw axis relative to the midpoint of the gimbal (degree) |
| pit_absolute_angle | The angle of the pitch axis relative to the ground (degree) |
| yaw_absolute_angle | The angle of the yaw axis relative to the ground (degree) |
| pit_palstance      | Angular velocity of the pitch axis (degree/s) |
| yaw_palstance      | Angular velocity of the yaw axis (degree/s) |

##### 0x0012 launching mechanism

Corresponds to the data structure shoot_info_t (launching mechanism status information)

```c
typedef __packed struct
{
  int16_t remain_bullets;  /* the member of remain bullets */
  int16_t shot_bullets;    /* the member of bullets that have been shot */
  uint8_t fric_wheel_run; /* friction run or not */
} shoot_info_t;
```

| Data           | Description                         |
| -------------- | ----------------------------------- |
| remain_bullets | Number of remaining projectiles     |
| shot_bullets   | Number of shot projectiles          |
| fric_wheel_run | Whether a friction wheel is running |

##### 0x0013 underlying error

Corresponds to the data structure infantry_err_t (underlying rover error information)

```c
typedef __packed struct
{
  bottom_err_e err_sta;                 /* bottom error state */
  bottom_err_e err[ERROR_LIST_LENGTH];  /* device error list */
} infantry_err_t;
```

| Data                   | Description                              |
| ---------------------- | ---------------------------------------- |
| err_sta                | Global status of underlying layer devices |
| err[ERROR_LIST_LENGTH] | Operating status of all devices and mechanisms |

*Note:*

The enumeration type bottom_err_e of underlying layer error information is as follows. If an error occurs to the related device, the status bit is set to `ERROR_EXIST`.

```c
typedef enum
{
  DEVICE_NORMAL = 0,
  ERROR_EXIST   = 1,
  UNKNOWN_STATE = 2,
} bottom_err_e;
```

All classes (mainly divided into 3 classes) of the underlying error information are included in err_id_e. Class 1 are `设备_OFFLINE` related devices going offline; class 2 are mechanism runtime errors. Currently, only cartridge jams are identified; class 3 are `_CONFIG_ERR` software related configuration errors; for example, the gimbal is installed beyond the physical range of the chassis.

```c
typedef enum
{
  BOTTOM_DEVICE        = 0,
  GIMBAL_GYRO_OFFLINE  = 1,
  CHASSIS_GYRO_OFFLINE = 2,
  CHASSIS_M1_OFFLINE   = 3,
  CHASSIS_M2_OFFLINE   = 4,
  CHASSIS_M3_OFFLINE   = 5,
  CHASSIS_M4_OFFLINE   = 6,
  REMOTE_CTRL_OFFLINE  = 7,
  JUDGE_SYS_OFFLINE    = 8,
  GIMBAL_YAW_OFFLINE   = 9,
  GIMBAL_PIT_OFFLINE   = 10,
  TRIGGER_MOTO_OFFLINE = 11,
  BULLET_JAM           = 12,
  CHASSIS_CONFIG_ERR   = 13,
  GIMBAL_CONFIG_ERR    = 14,
  ERROR_LIST_LENGTH    = 15,
} err_id_e;
```

##### 0x0014 structure configuration feedback

Corresponds to the data structure config_response_t (underlying layer structure configuration feedback information)

```c
typedef __packed struct
{
  struct_config_e chassis_config;
  struct_config_e gimbal_config;
} config_response_t;
```

| Data           | Description                              |
| -------------- | ---------------------------------------- |
| chassis_config | Active structure configuration for the chassis |
| gimbal_config  | Active structure configuration for the gimbal |

*Note:*

The enumeration type struct_config_e is defined as below:

```c
typedef enum
{
  NO_CONFIG      = 0,
  DEFAULT_CONFIG = 1,
  CUSTOM_CONFIG  = 3,
} struct_config_e;
```

To configure the chassis or Gimbal related information, set the configuration status to `CUSTOM_CONFIG`, and then fill in all related data, instead of only configuring a certain piece of chassis or gimbal data. If the configuration status is `DEFAULT_CONFIG` or this frame of data has never been transmitted, the default configuration will be applied for an underlying layer.

##### 0x0015 gimbal calibration feedback

Corresponds to the data structure cali_response_t (gimbal calibration feedback information)

```c
typedef __packed struct
{
  uint8_t type;
  int16_t yaw_offset;
  int16_t pitch_offset;
} cali_response_t;
```

| Data         | Description                              |
| ------------ | ---------------------------------------- |
| type         | Configuration succeeds or not (1: Yes 0: No) |
| yaw_offset   | Encoder value in the midpoint of the yaw axis (0 ~ 8191) |
| pitch_offset | Encoder value in the midpoint of the pitch axis (0 ~ 8191) |

##### 0x0016 remote control information

Corresponds to the data structure rc_info_t (remote control information)

```c
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  __packed struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
  
    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  __packed union
  {
    uint16_t key_code;
    __packed struct 
    {
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
    } bit;
  } kb;
} rc_info_t;
```

| Data      | Description                              |
| --------- | ---------------------------------------- |
| ch1 ~ ch4 | Data from 4 rocker channels of the remote control, data range (-660 ~ 660) |
| sw1 ~ sw2 | Data of 2 rockers of the remote control (upper: 1, middle: 3, lower: 2) |
| mouse     | Mouse data, mouse x and y movement speeds, left and right key values |
| key_code  | Keyboard key value; for available keys, bit in the union kb corresponds to key_code |

##### 0x0017 underlying layer version

Corresponds to the data structure version_info_t (underlying layer software version information)

```c
typedef __packed struct
{
  uint8_t num[4];
} version_info_t;
```

| Data   | Description            |
| ------ | ---------------------- |
| num[4] | Storage version number |

#### Class 3

##### 0x00A0 chassis control

Corresponds to the data structure chassis_ctrl_t (chassis control information)

```C
typedef __packed struct
{
  uint8_t          ctrl_mode; /* chassis control mode */
  int16_t          x_speed;   /* x-axis move speed(mm/s) of chassis */
  int16_t          y_speed;   /* y-axis move speed(mm/s) of chassis */
  chassis_rotate_t w_info;    /* rotation control of chassis */
} chassis_ctrl_t;
```

| Data      | Description                              |
| --------- | ---------------------------------------- |
| ctrl_mode | To control the operating mode of the chassis |
| x_speed   | To control the chassis x-axis motion speed (mm/s) |
| y_speed   | To control the chassis y-axis motion speed (mm/s) |
| w_info    | To control the rotation of the chassis   |

*Note:*

Rotation control information of the chassis is included in the chassis_rotate_t structure:

```c
typedef __packed struct
{
  int16_t x_offset;   /* offset(mm) relative to the x-axis of the chassis center */
  int16_t y_offset;   /* offset(mm) relative to the y-axis of the chassis center */
  float   w_speed;    /* rotation speed(degree/s) of chassis */
} chassis_rotate_t;
```

Includes the center of rotation and rotation speed of the chassis. The center of rotation is the position coordinate relative to the chassis' geometric center, the x and y axes correspond to x_offset and y_offset (in mm) and the rotation speed is expressed in degree/s.

##### 0x00A1 gimbal control

Corresponds to the data structure gimbal_ctrl_t (gimbal control information)

```c
typedef __packed struct
{
  uint8_t ctrl_mode;    /* gimbal control mode */
  float   pit_ref;      /* gimbal pitch reference angle(degree) */
  float   yaw_ref;      /* gimbal yaw reference angle(degree) */
  uint8_t visual_valid; /* visual information valid or not */
} gimbal_ctrl_t;
```

| Data         | Description                              |
| ------------ | ---------------------------------------- |
| ctrl_mode    | To control the operating mode of the gimbal |
| pit_ref      | The target angle of the pitch axis relative to the midpoint |
| yaw_ref      | The target angle of the yaw axis relative to the midpoint |
| visual_valid | A significance bit of visual information used to check whether the gimbal control data at that moment is trustworthy |

##### 0x00A2 launching mechanism control

Corresponds to the data structure shoot_ctrl_t (launching mechanism control information)

```C
typedef __packed struct
{
  uint8_t shoot_cmd;      /* single shoot command */
  uint8_t c_shoot_cmd;    /* continuous shoot command */
  uint8_t fric_wheel_run; /* friction run or not */
  uint8_t fric_wheel_spd; /* fricrion wheel speed */
} shoot_ctrl_t;
```

| Data           | Description                              |
| -------------- | ---------------------------------------- |
| shoot_cmd      | Single shot command                      |
| c_shoot_cmd    | Automatic firing command                 |
| fric_wheel_run | Turn on/off a friction wheel, 0: off, 1: on |
| fric_wheel_spd | Speed of a friction wheel, ranging from 0 to 100 |

##### 0x00A3 global error

Corresponds to the data structure global_err_level_t (warning level of overall system operation)

```c
typedef __packed struct
{
  err_level_e err_level;  /* the error level is included in err_level_e enumeration */
} global_err_level_t;
```

| Data      | Description                              |
| --------- | ---------------------------------------- |
| err_level | Refer mainly to data in the err_level_e type |

*Note:*

The warning level of the overall system operation is included in the err_level_e enumeration type:

```c
typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  SHOOT_ERROR          = 4,
  CHASSIS_ERROR        = 5,
  GIMBAL_ERROR         = 6,
} err_level_e;
```

The information mentioned above can be interpreted as being sorted by priority or degree of emergency. The higher the number is, the higher the priority or the degree of emergency.

*Note:*

> The error level of the system is transmitted from an upper layer to an underlying layer. If different types of errors occur, the error of the highest priority will be transmitted. Error level data can be divided into two categories: the first are running conditions of upper layer software, and the second are errors in underlying hardware. A user can define the handling method as desired when these different levels errors occur. The current approach is that, except for `SOFTWARE_FATAL_ERROR` in the software layer, an underlying layer will not respond to any other situations. When receiving the message indicating that `SOFTWARE_FATAL_ERROR` has occurred, the underlying layer will stop output from the gimbal and chassis. Hardware errors belonging to class 2 are mainly classified according to the data error list included in the underlying data err_id_e. If an error occurs to a mechanism, the mechanism itself and devices with lower priority will stop output.

##### 0x00A4 structure configuration

Corresponds to the data structure infantry_structure_t (rover structure configuration information)

```c
typedef __packed struct
{
  struct_config_e  chassis_config;  /* chassis structure config state */
  uint16_t         wheel_perimeter; /* the perimeter(mm) of wheel */
  uint16_t         wheel_track;     /* wheel track distance(mm) */
  uint16_t         wheel_base;      /* wheelbase distance(mm) */
  struct_config_e  gimbal_config;   /* gimbal structure config state */
  int16_t          gimbal_x_offset; /* gimbal offset(mm) relative to the x-axis of the chassis center */
  int16_t          gimbal_y_offset; /* gimbal offset(mm) relative to the y-axis of the chassis center */
} infantry_structure_t;
```

| Data            | Description                              |
| --------------- | ---------------------------------------- |
| chassis_config  | Chassis structure configuration status   |
| wheel_perimeter | Wheel perimeter of the chassis (mm)      |
| wheel_track     | Wheel track of the chassis (mm)          |
| wheel_base      | Wheel base of the chassis (mm)           |
| gimbal_config   | Gimbal structure configuration status    |
| gimbal_x_offset | The distance from the position where the gimbal is installed to the x axis on the center of the chassis (mm) |
| gimbal_y_offset | The distance from the position where the gimbal is installed to the y axis on the center of the chassis (mm) |

##### 0x00A5 gimbal calibration

Corresponds to the data structure cali_cmd_t (gimbal calibration command information)

```c
typedef __packed struct
{
  uint8_t type;
  /* 0x01: calibrate gimbal center start
     0x02: calibrate gimbal center end
     0x03: calibrate camera start
     0x04: calibrate camera end
     other: invalid */
} cali_cmd_t;
```

| Data | Description                              |
| ---- | ---------------------------------------- |
| type | Calibration type                         |
|      | 0x01: start calibrating the midpoint of the gimbal |
|      | 0x02: stop calibrating the midpoint of the gimbal |
|      | 0x03: start calibrating the camera       |
|      | 0x04: stop calibrating the camera        |

#### Class 4

##### 0x0100 displays on client side

Corresponds to the data structure client_show_data_t (customized data)

```c
typedef __packed struct
{
  float data1;
  float data2;
  float data3;
} client_show_data_t;
```

| Data  | Description       |
| ----- | ----------------- |
| data1 | Customized data 1 |
| data2 | Customized data 2 |
| data3 | Customized data 3 |

##### 0x0101 forward to server

Corresponds to the data structure user_to_server_t (transparent transmission of uplink data)

```c
typedef __packed struct
{
  uint8_t data[64];
} user_to_server_t;
```

| Data     | Description               |
| -------- | ------------------------- |
| data[64] | Customized data, up to 64 |

##### 0x0102 forward to decision-making PC

Corresponds to the data structure server_to_user_t (transparent transmission of downlink data)

```c
typedef __packed struct
{
  uint8_t data[32];
} server_to_user_t;
```

| Data     | Description               |
| -------- | ------------------------- |
| data[32] | Customized data, up to 32 |

### 4. Frame footer data

The CRC16 check result of each frame of data is stored in this location.
| Frame footer | Number of Bytes |
| :----------- | :-------------- |
| CRC16        | 2               |



## Data transmitting and receiving

### 1. Data transmitting

Use the function below to pack the data to transmit:

```c
/**
  * @brief     pack data to bottom device
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  * @usage     data_pack_handle(CHASSIS_CTRL_ID, &chassis_control_data, sizeof(chassis_ctrl_t))
  */

void data_pack_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
  memset(computer_tx_buf, 0, COMPUTER_FRAME_BUFLEN);
  frame_header_t *p_header = (frame_header_t*)computer_tx_buf;
  
  p_header->sof          = UP_REG_ID;
  p_header->data_length  = len;
  
  memcpy(&computer_tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(computer_tx_buf, HEADER_LEN);
  
  memcpy(&computer_tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
  append_crc16_check_sum(computer_tx_buf, HEADER_LEN + CMD_LEN + len + CRC_LEN);

}
```

### 2. Data receiving

Use the method below to address the data adhering problem

```c
void read_and_unpack_thread(void *argu)
{
  uint8_t byte = 0;
  int32_t read_len;
  int32_t buff_read_index;
  
  uint16_t      data_len;
  unpack_step_e unpack_step;
  int32_t       index;
  uint8_t       protocol_packet[PROTOCAL_FRAME_MAX_SIZE];

  while (1)
  {
    read_len = uart_recv(uart_fd, computer_rx_buf, UART_BUFF_SIZE);
    buff_read_index = 0;
    
    while (read_len--)
    {
      byte = computer_rx_buf[buff_read_index++];
      
      switch(unpack_step)
      {
        case STEP_HEADER_SOF:
        {
          if(byte == UP_REG_ID)
          {
            unpack_step = STEP_LENGTH_LOW;
            protocol_packet[index++] = byte;
          }
          else
          {
            index = 0;
          }
        }break;
        
        case STEP_LENGTH_LOW:
        {
          data_len = byte;
          protocol_packet[index++] = byte;
          unpack_step = STEP_LENGTH_HIGH;
        }break;
        
        case STEP_LENGTH_HIGH:
        {
          data_len |= (byte << 8);
          protocol_packet[index++] = byte;

          if(data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
          {
            unpack_step = STEP_FRAME_SEQ;
          }
          else
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;
          }
        }break;
      
        case STEP_FRAME_SEQ:
        {
          protocol_packet[index++] = byte;
          unpack_step = STEP_HEADER_CRC8;
        }break;

        case STEP_HEADER_CRC8:
        {
          protocol_packet[index++] = byte;

          if (index == HEADER_LEN)
          {
            if ( verify_crc8_check_sum(protocol_packet, HEADER_LEN) )
            {
              unpack_step = STEP_DATA_CRC16;
            }
            else
            {
              unpack_step = STEP_HEADER_SOF;
              index = 0;
            }
          }
        }break;  

        case STEP_DATA_CRC16:
        {
          if (index < (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
             protocol_packet[index++] = byte;  
          }
          if (index >= (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;

            if ( verify_crc16_check_sum(protocol_packet, HEADER_LEN + CMD_LEN + data_len + CRC_LEN) )
            {
              data_handle(protocol_packet);
            }
          }
        }break;

        default:
        {
          unpack_step = STEP_HEADER_SOF;
          index = 0;
        }break;
      }
    }
  }
}

void data_handle(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
  
  switch (cmd_id)
  {
    case GAME_INFO_ID:
      memcpy(&game_information, data_addr, data_length);
    break;
    
    //............
    //............

  }
}
```



## Protocol Version

V1.3