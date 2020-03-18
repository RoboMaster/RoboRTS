#pragma once
#ifndef _CAMERA_DEFINE_H_
#define _CAMERA_DEFINE_H_

#include "CameraStatus.h"

#define MAX_CROSS_LINE 9

//相机的句柄类型定义
typedef int CameraHandle;
typedef int INT;
typedef int LONG;
typedef unsigned int UINT;
typedef unsigned long long UINT64;
typedef int BOOL;
typedef unsigned char BYTE;
typedef unsigned int DWORD;
typedef void* PVOID;
typedef void* HWND;
typedef char* LPCTSTR;
typedef unsigned short USHORT;
typedef short SHORT;
 typedef unsigned char* LPBYTE;
typedef char CHAR;
typedef char TCHAR;
typedef  unsigned short WORD;
typedef INT HANDLE;
typedef void VOID;
typedef unsigned int ULONG;
typedef void* LPVOID;
typedef unsigned char UCHAR;
typedef void* HMODULE;

#define TRUE 1
#define FALSE 0
//图像查表变换的方式
typedef enum
{
    LUTMODE_PARAM_GEN=0,//通过调节参数动态生成LUT表
    LUTMODE_PRESET,     //使用预设的LUT表
    LUTMODE_USER_DEF    //使用用户自定义的LUT表
}emSdkLutMode;

//相机的视频流控制
typedef enum
{
    RUNMODE_PLAY=0,    //正常预览，捕获到图像就显示。（如果相机处于触发模式，则会等待触发帧的到来）
    RUNMODE_PAUSE,     //暂停，会暂停相机的图像输出，同时也不会去捕获图像
    RUNMODE_STOP       //停止相机工作。反初始化后，相机就处于停止模式
}emSdkRunMode;

//SDK内部显示接口的显示方式
typedef enum
{
    DISPLAYMODE_SCALE=0, //缩放显示模式，缩放到显示控件的尺寸
    DISPLAYMODE_REAL     //1:1显示模式，当图像尺寸大于显示控件的尺寸时，只显示局部
}emSdkDisplayMode;

//录像状态
typedef enum
{
  RECORD_STOP = 0,  //停止
  RECORD_START,     //录像中
  RECORD_PAUSE      //暂停
}emSdkRecordMode;

//图像的镜像操作
typedef enum
{
    MIRROR_DIRECTION_HORIZONTAL = 0,//水平镜像
    MIRROR_DIRECTION_VERTICAL       //垂直镜像
}emSdkMirrorDirection;

/// @ingroup MV_ENUM_TYPE
/// \~chinese 图像的旋转操作
/// \~english Rotation of the image
typedef enum
{
	ROTATE_DIRECTION_0=0,		///< \~chinese 不旋转		\~english Do not rotate
	ROTATE_DIRECTION_90=1,		///< \~chinese 逆时针90度	\~english Counterclockwise 90 degrees
	ROTATE_DIRECTION_180=2,		///< \~chinese 逆时针180度	\~english Counterclockwise 180 degrees
	ROTATE_DIRECTION_270=3,		///< \~chinese 逆时针270度	\~english Counterclockwise 270 degrees
}emSdkRotateDirection;

//相机视频的帧率
typedef enum
{
    FRAME_SPEED_LOW = 0,  //低速模式
    FRAME_SPEED_NORMAL,   //普通模式
    FRAME_SPEED_HIGH,     //高速模式(需要较高的传输带宽,多设备共享传输带宽时会对帧率的稳定性有影响)
    FRAME_SPEED_SUPER     //超高速模式(需要较高的传输带宽,多设备共享传输带宽时会对帧率的稳定性有影响)
}emSdkFrameSpeed;

//保存文件的格式类型
typedef enum
{
    FILE_JPG = 1,//JPG
    FILE_BMP = 2,//BMP
    FILE_RAW = 4,//相机输出的bayer格式文件,对于不支持bayer格式输出相机，无法保存为该格式
    FILE_PNG = 8, //PNG
    FILE_BMP_8BIT = 16,	  ///< \~chinese BMP 8bit		\~english BMP 8bit
    FILE_PNG_8BIT = 32,   ///< \~chinese PNG 8bit		\~english PNG 8bit
	  FILE_RAW_16BIT = 64,	///< \~chinese RAW 16bit	\~english RAW 16bit
}emSdkFileType;

//相机中的图像传感器的工作模式
typedef enum
{
    CONTINUATION = 0,//连续采集模式
    SOFT_TRIGGER,    //软件触发模式，由软件发送指令后，传感器开始采集指定帧数的图像，采集完成后，停止输出
    EXTERNAL_TRIGGER //硬件触发模式，当接收到外部信号，传感器开始采集指定帧数的图像，采集完成后，停止输出
} emSdkSnapMode;

//自动曝光时抗频闪的频闪
typedef enum
{
    LIGHT_FREQUENCY_50HZ = 0,//50HZ,一般的灯光都是50HZ
    LIGHT_FREQUENCY_60HZ     //60HZ,主要是指显示器的
}emSdkLightFrequency;

//相机的配置参数，分为A,B,C,D 4组进行保存。
typedef enum
{
    PARAMETER_TEAM_DEFAULT = 0xff,
    PARAMETER_TEAM_A = 0,
    PARAMETER_TEAM_B = 1,
    PARAMETER_TEAM_C = 2,
    PARAMETER_TEAM_D = 3
}emSdkParameterTeam;


/*emSdkParameterMode 相机参数加载模式，参数加载分为从文件和从设备加载两种方式

PARAM_MODE_BY_MODEL:所有同型号的相机共用ABCD四组参数文件。修改
             一台相机的参数文件，会影响到整个同型号的
             相机参数加载。

PARAM_MODE_BY_NAME:所有设备名相同的相机，共用ABCD四组参数文件。
         默认情况下，当电脑上只接了某型号一台相机时，
         设备名都是一样的，而您希望某一台相机能够加载
         不同的参数文件，则可以通过修改其设备名的方式
         来让其加载指定的参数文件。

PARAM_MODE_BY_SN:相机按照自己的唯一序列号来加载ABCD四组参数文件，
         序列号在出厂时已经固化在相机内，每台相机的序列号
         都不相同，通过这种方式，每台相机的参数文件都是独立的。

您可以根据自己的使用环境，灵活使用以上几种方式加载参数。例如，以
MV-U300为例，您希望多台该型号的相机在您的 电脑上都共用4组参数，那么就
使用PARAM_MODE_BY_MODEL方式;如果您希望其中某一台或者某几台MV-U300能
使用自己参数文件而其余的MV-U300又要使用相同的参数文件，那么使用
PARAM_MODE_BY_NAME方式;如果您希望每台MV-U300都使用不同的参数文件，那么
使用PARAM_MODE_BY_SN方式。
参数文件存在安装目录的 \Camera\Configs 目录下，以config为后缀名的文件。
*/
typedef enum
{
  PARAM_MODE_BY_MODEL = 0,  //根据相机型号名从文件中加载参数，例如MV-U300
  PARAM_MODE_BY_NAME,       //根据设备昵称(tSdkCameraDevInfo.acFriendlyName)从文件中加载参数，例如MV-U300,该昵称可自定义
  PARAM_MODE_BY_SN,         //根据设备的唯一序列号从文件中加载参数，序列号在出厂时已经写入设备，每台相机拥有不同的序列号。
  PARAM_MODE_IN_DEVICE      //从设备的固态存储器中加载参数。不是所有的型号都支持从相机中读写参数组，由tSdkCameraCapbility.bParamInDevice决定
}emSdkParameterMode;


//SDK生成的相机配置页面掩码值
typedef enum
{
  PROP_SHEET_INDEX_EXPOSURE = 0,
  PROP_SHEET_INDEX_ISP_COLOR,
  PROP_SHEET_INDEX_ISP_LUT,
  PROP_SHEET_INDEX_ISP_SHAPE,
  PROP_SHEET_INDEX_VIDEO_FORMAT,
  PROP_SHEET_INDEX_RESOLUTION,
  PROP_SHEET_INDEX_IO_CTRL,
  PROP_SHEET_INDEX_TRIGGER_SET,
  PROP_SHEET_INDEX_OVERLAY,
  PROP_SHEET_INDEX_DEVICE_INFO,
  PROP_SHEET_INDEX_WDR,
  PROP_SHEET_INDEX_MULTI_EXPOSURE
}emSdkPropSheetMask;

//SDK生成的相机配置页面的回调消息类型
typedef enum
{
  SHEET_MSG_LOAD_PARAM_DEFAULT = 0, //参数被恢复成默认后，触发该消息
  SHEET_MSG_LOAD_PARAM_GROUP,       //加载指定参数组，触发该消息
  SHEET_MSG_LOAD_PARAM_FROMFILE,    //从指定文件加载参数后，触发该消息
  SHEET_MSG_SAVE_PARAM_GROUP        //当前参数组被保存时，触发该消息
}emSdkPropSheetMsg;

//可视化选择参考窗口的类型
typedef enum
{
  REF_WIN_AUTO_EXPOSURE = 0,
  REF_WIN_WHITE_BALANCE,
}emSdkRefWinType;

//可视化选择参考窗口的类型
typedef enum
{
  RES_MODE_PREVIEW = 0,
  RES_MODE_SNAPSHOT,
}emSdkResolutionMode;

//白平衡时色温模式
typedef enum
{
  CT_MODE_AUTO = 0, //自动识别色温
  CT_MODE_PRESET,   //使用指定的预设色温
  CT_MODE_USER_DEF  //自定义色温(增益和矩阵)
}emSdkClrTmpMode;

//LUT的颜色通道
typedef enum
{
  LUT_CHANNEL_ALL = 0,//R,B,G三通道同时调节
  LUT_CHANNEL_RED,    //红色通道
  LUT_CHANNEL_GREEN,  //绿色通道
  LUT_CHANNEL_BLUE,   //蓝色通道
}emSdkLutChannel;

//ISP处理单元
typedef enum
{
  ISP_PROCESSSOR_PC = 0,//使用PC的软件ISP模块
  ISP_PROCESSSOR_DEVICE //使用相机自带的硬件ISP模块
}emSdkIspProcessor;

//闪光灯信号控制方式
typedef enum
{
  STROBE_SYNC_WITH_TRIG_AUTO = 0,    //和触发信号同步，触发后，相机进行曝光时，自动生成STROBE信号。此时，有效极性可设置(CameraSetStrobePolarity)。
  STROBE_SYNC_WITH_TRIG_MANUAL,      //和触发信号同步，触发后，STROBE延时指定的时间后(CameraSetStrobeDelayTime)，再持续指定时间的脉冲(CameraSetStrobePulseWidth)，有效极性可设置(CameraSetStrobePolarity)。
  STROBE_ALWAYS_HIGH,                //始终为高，忽略STROBE信号的其他设置
  STROBE_ALWAYS_LOW                  //始终为低，忽略STROBE信号的其他设置
}emStrobeControl;

//硬件外触发的信号种类
typedef enum
{
  EXT_TRIG_LEADING_EDGE = 0,     //上升沿触发，默认为该方式
  EXT_TRIG_TRAILING_EDGE,        //下降沿触发
  EXT_TRIG_HIGH_LEVEL,           //高电平触发,电平宽度决定曝光时间，仅部分型号的相机支持电平触发方式。
  EXT_TRIG_LOW_LEVEL             //低电平触发,
}emExtTrigSignal;

//硬件外触发时的快门方式
typedef enum
{
  EXT_TRIG_EXP_STANDARD = 0,     //标准方式，默认为该方式。
  EXT_TRIG_EXP_GRR,              //全局复位方式，部分滚动快门的CMOS型号的相机支持该方式，配合外部机械快门，可以达到全局快门的效果，适合拍高速运动的物体
}emExtTrigShutterMode;

// GPIO模式
typedef enum
{
  IOMODE_TRIG_INPUT=0,		    ///< \~chinese 触发输入 \~english Trigger input
  IOMODE_STROBE_OUTPUT=1,		  ///< \~chinese 闪光灯输出 \~english Strobe output
  IOMODE_GP_INPUT=2,			    ///< \~chinese 通用型输入 \~english Universal input
  IOMODE_GP_OUTPUT=3,			    ///< \~chinese 通用型输出 \~english Universal output
  IOMODE_PWM_OUTPUT=4,		    ///< \~chinese PWM型输出 \~english PWM output
  IOMODE_ROTARYENC_INPUT=5,   ///< \~chinese 编码器输入 \~english rotary input
}emCameraGPIOMode;

/// @ingroup MV_ENUM_TYPE
/// \~chinese GPIO 格式
/// \~english GPIO Format
typedef enum 
{
	IOFORMAT_SINGLE=0,			///< \~chinese 单端  \~english single ended
	IOFORMAT_RS422=1,			  ///< \~chinese 差分RS422 \~english Differential RS422
	IOFORMAT_RS422_TERM=2,	///< \~chinese 差分RS422带终端电阻 \~english Differential RS422 and Termination Enable
}emCameraGPIOFormat;

/// @ingroup MV_ENUM_TYPE
/// \~chinese 取图优先级
/// \~english Get Image priority
typedef enum
{
	CAMERA_GET_IMAGE_PRIORITY_OLDEST=0,		///< \~chinese 获取缓存中最旧的一帧 \~english	Get the oldest frame in the cache
	CAMERA_GET_IMAGE_PRIORITY_NEWEST=1,		///< \~chinese 获取缓存中最新的一帧（比此帧旧的将全部丢弃） \~english Get the latest frame in the cache (older than this frame will be discarded)

	/// \~chinese 丢弃缓存中的所有帧，并且如果此刻相机正在曝光或传输将会被立即打断，等待接收下一帧
	/// \note 某些型号的相机不支持此功能，对于不支持此功能的相机这个标志相当于@link #CAMERA_GET_IMAGE_PRIORITY_OLDEST @endlink
	/// \~english All frames in the cache are discarded, and if the camera is now being exposed or transmitted it will be immediately interrupted, waiting to receive the next frame
	/// \note Some models do not support this feature. For cameras that do not support this feature this flag is equivalent to @link #CAMERA_GET_IMAGE_PRIORITY_OLDEST @endlink
	CAMERA_GET_IMAGE_PRIORITY_NEXT=2,
}emCameraGetImagePriority;

/// @ingroup MV_ENUM_TYPE
/// \~chinese 软触发功能标志
/// \~english Soft trigger function flag
typedef enum
{
	CAMERA_ST_CLEAR_BUFFER_BEFORE	= 0x1,	///< \~chinese 在软触发之前先清空相机已缓存的帧 \~english Empty camera-cached frames before soft triggering
}emCameraSoftTriggerExFlags;

//相机的设备信息
typedef struct
{
    char acProductSeries[32];   // 产品系列
    char acProductName[32];     // 产品名称
    char acFriendlyName[32];    // 产品昵称，用户可自定义改昵称，保存在相机内，用于区分多个相机同时使用,可以用CameraSetFriendlyName接口改变该昵称，设备重启后生效。
    char acLinkName[32];        // 内核符号连接名，内部使用
    char acDriverVersion[32];   // 驱动版本
    char acSensorType[32];      // sensor类型
    char acPortType[32];        // 接口类型
    char acSn[32];              // 产品唯一序列号
    UINT uInstance;             // 该型号相机在该电脑上的实例索引号，用于区分同型号多相机
} tSdkCameraDevInfo;

#define EXT_TRIG_MASK_GRR_SHUTTER  1	///< \~chinese 快门支持GRR模式 \~english Shutter supports GRR mode
#define EXT_TRIG_MASK_LEVEL_MODE   2	///< \~chinese 支持电平触发 \~english Support level trigger
#define EXT_TRIG_MASK_DOUBLE_EDGE  4	///< \~chinese 支持双边沿触发 \~english Supports bilateral triggering

//tSdkResolutionRange结构体中SKIP、 BIN、RESAMPLE模式的掩码值
#define MASK_2X2_HD     (1<<0)    //硬件SKIP、BIN、重采样 2X2
#define MASK_3X3_HD     (1<<1)
#define MASK_4X4_HD     (1<<2)
#define MASK_5X5_HD     (1<<3)
#define MASK_6X6_HD     (1<<4)
#define MASK_7X7_HD     (1<<5)
#define MASK_8X8_HD     (1<<6)
#define MASK_9X9_HD     (1<<7)
#define MASK_10X10_HD   (1<<8)
#define MASK_11X11_HD   (1<<9)
#define MASK_12X12_HD   (1<<10)
#define MASK_13X13_HD   (1<<11)
#define MASK_14X14_HD   (1<<12)
#define MASK_15X15_HD   (1<<13)
#define MASK_16X16_HD   (1<<14)
#define MASK_17X17_HD   (1<<15)
#define MASK_2X2_SW     (1<<16)   //硬件SKIP、BIN、重采样 2X2
#define MASK_3X3_SW     (1<<17)
#define MASK_4X4_SW     (1<<18)
#define MASK_5X5_SW     (1<<19)
#define MASK_6X6_SW     (1<<20)
#define MASK_7X7_SW     (1<<21)
#define MASK_8X8_SW     (1<<22)
#define MASK_9X9_SW     (1<<23)
#define MASK_10X10_SW   (1<<24)
#define MASK_11X11_SW   (1<<25)
#define MASK_12X12_SW   (1<<26)
#define MASK_13X13_SW   (1<<27)
#define MASK_14X14_SW   (1<<28)
#define MASK_15X15_SW   (1<<29)
#define MASK_16X16_SW   (1<<30)
#define MASK_17X17_SW   (1<<31)

//相机的分辨率设定范围，用于构件UI
typedef struct
{
  INT iHeightMax;             //图像最大高度
  INT iHeightMin;             //图像最小高度
  INT iWidthMax;              //图像最大宽度
  INT iWidthMin;              //图像最小宽度
  UINT uSkipModeMask;         //SKIP模式掩码，为0，表示不支持SKIP 。bit0为1,表示支持SKIP 2x2 ;bit1为1，表示支持SKIP 3x3....
  UINT uBinSumModeMask;       //BIN(求和)模式掩码，为0，表示不支持BIN 。bit0为1,表示支持BIN 2x2 ;bit1为1，表示支持BIN 3x3....
  UINT uBinAverageModeMask;   //BIN(求均值)模式掩码，为0，表示不支持BIN 。bit0为1,表示支持BIN 2x2 ;bit1为1，表示支持BIN 3x3....
  UINT uResampleMask;         //硬件重采样的掩码
} tSdkResolutionRange;


//相机的分辨率描述
typedef struct
{
  INT     iIndex;             // 索引号，[0,N]表示预设的分辨率(N 为预设分辨率的最大个数，一般不超过20),OXFF 表示自定义分辨率(ROI)
  char    acDescription[32];  // 该分辨率的描述信息。仅预设分辨率时该信息有效。自定义分辨率可忽略该信息
  UINT    uBinSumMode;        // BIN(求和)的模式,范围不能超过tSdkResolutionRange中uBinSumModeMask
  UINT    uBinAverageMode;    // BIN(求均值)的模式,范围不能超过tSdkResolutionRange中uBinAverageModeMask
  UINT    uSkipMode;          // 是否SKIP的尺寸，为0表示禁止SKIP模式，范围不能超过tSdkResolutionRange中uSkipModeMask
  UINT    uResampleMask;      // 硬件重采样的掩码
  INT     iHOffsetFOV;        // 采集视场相对于Sensor最大视场左上角的垂直偏移
  INT     iVOffsetFOV;        // 采集视场相对于Sensor最大视场左上角的水平偏移
  INT     iWidthFOV;          // 采集视场的宽度
  INT     iHeightFOV;         // 采集视场的高度
  INT     iWidth;             // 相机最终输出的图像的宽度
  INT     iHeight;            // 相机最终输出的图像的高度
  INT     iWidthZoomHd;       // 硬件缩放的宽度,不需要进行此操作的分辨率，此变量设置为0.
  INT     iHeightZoomHd;      // 硬件缩放的高度,不需要进行此操作的分辨率，此变量设置为0.
  INT     iWidthZoomSw;       // 软件缩放的宽度,不需要进行此操作的分辨率，此变量设置为0.
  INT     iHeightZoomSw;      // 软件缩放的高度,不需要进行此操作的分辨率，此变量设置为0.
} tSdkImageResolution;

//相机白平衡色温模式描述信息
typedef struct
{
    INT  iIndex;            // 模式索引号
    char acDescription[32]; // 描述信息
} tSdkColorTemperatureDes;

//相机帧率描述信息
typedef struct
{
    INT  iIndex;             // 帧率索引号，一般0对应于低速模式，1对应于普通模式，2对应于高速模式
    char acDescription[32];  // 描述信息
} tSdkFrameSpeed;

//相机曝光功能范围定义
typedef struct
{
    UINT  uiTargetMin;      //自动曝光亮度目标最小值
    UINT  uiTargetMax;      //自动曝光亮度目标最大值
    UINT  uiAnalogGainMin;  //模拟增益的最小值，单位为fAnalogGainStep中定义
    UINT  uiAnalogGainMax;  //模拟增益的最大值，单位为fAnalogGainStep中定义
    float fAnalogGainStep;  //模拟增益每增加1，对应的增加的放大倍数。例如，uiAnalogGainMin一般为16，fAnalogGainStep一般为0.125，那么最小放大倍数就是16*0.125 = 2倍
    UINT  uiExposeTimeMin;  //手动模式下，曝光时间的最小值，单位:行。根据CameraGetExposureLineTime可以获得一行对应的时间(微秒),从而得到整帧的曝光时间
    UINT  uiExposeTimeMax;  //手动模式下，曝光时间的最大值，单位:行
} tSdkExpose;

//触发模式描述
typedef struct
{
  INT   iIndex;            //模式索引号
  char  acDescription[32]; //该模式的描述信息
} tSdkTrigger;

//传输分包大小描述(主要是针对网络相机有效)
typedef struct
{
    INT  iIndex;              //分包大小索引号
    char acDescription[32];   //对应的描述信息
    UINT iPackSize;
} tSdkPackLength;

//预设的LUT表描述
typedef struct
{
    INT  iIndex;                //编号
    char acDescription[32];     //描述信息
} tSdkPresetLut;

//AE算法描述
typedef struct
{
    INT  iIndex;                //编号
    char acDescription[32];     //描述信息
} tSdkAeAlgorithm;

//RAW转RGB算法描述
typedef struct
{
    INT  iIndex;                //编号
    char acDescription[32];     //描述信息
} tSdkBayerDecodeAlgorithm;


//帧率统计信息
typedef struct
{
  INT iTotal;           //当前采集的总帧数（包括错误帧）
    INT iCapture;       //当前采集的有效帧的数量
    INT iLost;          //当前丢帧的数量
} tSdkFrameStatistic;

//相机输出的图像数据格式
typedef struct
{
  INT     iIndex;             //格式种类编号
  char    acDescription[32];  //描述信息
  UINT    iMediaType;         //对应的图像格式编码，如CAMERA_MEDIA_TYPE_BAYGR8，在本文件中有定义。
} tSdkMediaType;

//伽马的设定范围
typedef struct
{
  INT iMin;       //最小值
  INT iMax;       //最大值
} tGammaRange;

//对比度的设定范围
typedef struct
{
    INT iMin;   //最小值
    INT iMax;   //最大值
} tContrastRange;

//RGB三通道数字增益的设定范围
typedef struct
{
    INT iRGainMin;    //红色增益的最小值
    INT iRGainMax;    //红色增益的最大值
    INT iGGainMin;    //绿色增益的最小值
    INT iGGainMax;    //绿色增益的最大值
    INT iBGainMin;    //蓝色增益的最小值
    INT iBGainMax;    //蓝色增益的最大值
} tRgbGainRange;

//饱和度设定的范围
typedef struct
{
    INT iMin;   //最小值
    INT iMax;   //最大值
} tSaturationRange;

//锐化的设定范围
typedef struct
{
  INT iMin;   //最小值
  INT iMax;   //最大值
} tSharpnessRange;

//ISP模块的使能信息
typedef struct
{
    BOOL bMonoSensor;       //表示该型号相机是否为黑白相机,如果是黑白相机，则颜色相关的功能都无法调节
    BOOL bWbOnce;           //表示该型号相机是否支持手动白平衡功能
    BOOL bAutoWb;           //表示该型号相机是否支持自动白平衡功能
    BOOL bAutoExposure;     //表示该型号相机是否支持自动曝光功能
    BOOL bManualExposure;   //表示该型号相机是否支持手动曝光功能
    BOOL bAntiFlick;        //表示该型号相机是否支持抗频闪功能
    BOOL bDeviceIsp;        //表示该型号相机是否支持硬件ISP功能
    BOOL bForceUseDeviceIsp;//bDeviceIsp和bForceUseDeviceIsp同时为TRUE时，表示强制只用硬件ISP，不可取消。
    BOOL bZoomHD;           //相机硬件是否支持图像缩放输出(只能是缩小)。
} tSdkIspCapacity;

/* 定义整合的设备描述信息，这些信息可以用于动态构建UI */
typedef struct
{

  tSdkTrigger   *pTriggerDesc;          // 触发模式
  INT           iTriggerDesc;           // 触发模式的个数，即pTriggerDesc数组的大小

  tSdkImageResolution   *pImageSizeDesc;// 预设分辨率选择
  INT                   iImageSizeDesc; // 预设分辨率的个数，即pImageSizeDesc数组的大小

  tSdkColorTemperatureDes *pClrTempDesc;// 预设色温模式，用于白平衡
  INT                     iClrTempDesc;

  tSdkMediaType     *pMediaTypeDesc;    // 相机输出图像格式
  INT               iMediaTypdeDesc;    // 相机输出图像格式的种类个数，即pMediaTypeDesc数组的大小。

  tSdkFrameSpeed    *pFrameSpeedDesc;   // 可调节帧速类型，对应界面上普通 高速 和超级三种速度设置
  INT               iFrameSpeedDesc;    // 可调节帧速类型的个数，即pFrameSpeedDesc数组的大小。

  tSdkPackLength    *pPackLenDesc;      // 传输包长度，一般用于网络设备
  INT               iPackLenDesc;       // 可供选择的传输分包长度的个数，即pPackLenDesc数组的大小。

  INT           iOutputIoCounts;        // 可编程输出IO的个数
  INT           iInputIoCounts;         // 可编程输入IO的个数

  tSdkPresetLut  *pPresetLutDesc;       // 相机预设的LUT表
  INT            iPresetLut;            // 相机预设的LUT表的个数，即pPresetLutDesc数组的大小

  INT           iUserDataMaxLen;        // 指示该相机中用于保存用户数据区的最大长度。为0表示无。
  BOOL          bParamInDevice;         // 指示该设备是否支持从设备中读写参数组。1为支持，0不支持。

  tSdkAeAlgorithm   *pAeAlmSwDesc;      // 软件自动曝光算法描述
  int                iAeAlmSwDesc;      // 软件自动曝光算法个数

  tSdkAeAlgorithm    *pAeAlmHdDesc;     // 硬件自动曝光算法描述，为NULL表示不支持硬件自动曝光
  int                iAeAlmHdDesc;      // 硬件自动曝光算法个数，为0表示不支持硬件自动曝光

  tSdkBayerDecodeAlgorithm   *pBayerDecAlmSwDesc; // 软件Bayer转换为RGB数据的算法描述
  int                        iBayerDecAlmSwDesc;  // 软件Bayer转换为RGB数据的算法个数

  tSdkBayerDecodeAlgorithm   *pBayerDecAlmHdDesc; // 硬件Bayer转换为RGB数据的算法描述，为NULL表示不支持
  int                        iBayerDecAlmHdDesc;  // 硬件Bayer转换为RGB数据的算法个数，为0表示不支持

  /* 图像参数的调节范围定义,用于动态构建UI*/
  tSdkExpose            sExposeDesc;      // 曝光的范围值
  tSdkResolutionRange   sResolutionRange; // 分辨率范围描述
  tRgbGainRange         sRgbGainRange;    // 图像数字增益范围描述
  tSaturationRange      sSaturationRange; // 饱和度范围描述
  tGammaRange           sGammaRange;      // 伽马范围描述
  tContrastRange        sContrastRange;   // 对比度范围描述
  tSharpnessRange       sSharpnessRange;  // 锐化范围描述
  tSdkIspCapacity       sIspCapacity;     // ISP能力描述


} tSdkCameraCapbility;


//图像帧头信息
typedef struct
{
  UINT    uiMediaType;    // 图像格式,Image Format
  UINT    uBytes;         // 图像数据字节数,Total bytes
  INT     iWidth;         // 图像的宽度，调用图像处理函数后，该变量可能被动态修改，来指示处理后的图像尺寸
  INT     iHeight;        // 图像的高度，调用图像处理函数后，该变量可能被动态修改，来指示处理后的图像尺寸
  INT     iWidthZoomSw;   // 软件缩放的宽度,不需要进行软件裁剪的图像，此变量设置为0.
  INT     iHeightZoomSw;  // 软件缩放的高度,不需要进行软件裁剪的图像，此变量设置为0.
  BOOL    bIsTrigger;     // 指示是否为触发帧 is trigger
  UINT    uiTimeStamp;    // 该帧的采集时间，单位0.1毫秒
  UINT    uiExpTime;      // 当前图像的曝光值，单位为微秒us
  float   fAnalogGain;    // 当前图像的模拟增益倍数
  INT     iGamma;         // 该帧图像的伽马设定值，仅当LUT模式为动态参数生成时有效，其余模式下为-1
  INT     iContrast;      // 该帧图像的对比度设定值，仅当LUT模式为动态参数生成时有效，其余模式下为-1
  INT     iSaturation;    // 该帧图像的饱和度设定值，对于黑白相机无意义，为0
  float   fRgain;         // 该帧图像处理的红色数字增益倍数，对于黑白相机无意义，为1
  float   fGgain;         // 该帧图像处理的绿色数字增益倍数，对于黑白相机无意义，为1
  float   fBgain;         // 该帧图像处理的蓝色数字增益倍数，对于黑白相机无意义，为1
}tSdkFrameHead;

//图像帧描述
typedef struct sCameraFrame
{
  tSdkFrameHead   head;     //帧头
  BYTE *          pBuffer;  //数据区
}tSdkFrame;

//图像捕获的回调函数定义
typedef void (*CAMERA_SNAP_PROC)(CameraHandle hCamera, BYTE *pFrameBuffer, tSdkFrameHead* pFrameHead,PVOID pContext);

//SDK生成的相机配置页面的消息回调函数定义
typedef void (*CAMERA_PAGE_MSG_PROC)(CameraHandle hCamera,UINT MSG,UINT uParam,PVOID pContext);

/// @ingroup API_RECONNECT
/// \~chinese 相机连接状态回调
/// \param [in] hCamera 相机句柄
/// \param [in] MSG 消息，0: 相机连接断开    1: 相机连接恢复
/// \param [in] uParam 附加信息
/// \param [in] pContext 用户数据
/// \return 无
/// \note USB相机uParam取值：
/// \note 		未定义
/// \note 网口相机uParam取值：
/// \note		当MSG=0时：未定义
/// \note		当MSG=1时：
/// \note			0：上次掉线原因，网络通讯失败
/// \note			1：上次掉线原因，相机掉电
/// \~english Camera connection status callback
/// \param [in] hCamera Camera handle
/// \param [in] MSG message, 0: Camera disconnected 1: Camera connection restored
/// \param [in] uParam Additional Information
/// \param [in] pContext user data
/// \return None
/// \note USB camera uParam value:
/// \note       Undefined
/// \note network camera uParam value:
/// \note       When MSG=0: Undefined
/// \note       When MSG=1:
/// \note           0: The last dropped reason, network communication failed
/// \note           1: The last dropped reason, the camera lost power
typedef void (*CAMERA_CONNECTION_STATUS_CALLBACK)(CameraHandle hCamera,UINT MSG,UINT uParam,PVOID pContext);


//----------------------------IMAGE FORMAT DEFINE------------------------------------
//----------------------------图像格式定义-------------------------------------------
#define CAMERA_MEDIA_TYPE_MONO                           0x01000000
#define CAMERA_MEDIA_TYPE_RGB                            0x02000000
#define CAMERA_MEDIA_TYPE_COLOR                          0x02000000
#define CAMERA_MEDIA_TYPE_CUSTOM                         0x80000000
#define CAMERA_MEDIA_TYPE_COLOR_MASK                     0xFF000000
#define CAMERA_MEDIA_TYPE_OCCUPY1BIT                     0x00010000
#define CAMERA_MEDIA_TYPE_OCCUPY2BIT                     0x00020000
#define CAMERA_MEDIA_TYPE_OCCUPY4BIT                     0x00040000
#define CAMERA_MEDIA_TYPE_OCCUPY8BIT                     0x00080000
#define CAMERA_MEDIA_TYPE_OCCUPY10BIT                    0x000A0000
#define CAMERA_MEDIA_TYPE_OCCUPY12BIT                    0x000C0000
#define CAMERA_MEDIA_TYPE_OCCUPY16BIT                    0x00100000
#define CAMERA_MEDIA_TYPE_OCCUPY24BIT                    0x00180000
#define CAMERA_MEDIA_TYPE_OCCUPY32BIT                    0x00200000
#define CAMERA_MEDIA_TYPE_OCCUPY36BIT                    0x00240000
#define CAMERA_MEDIA_TYPE_OCCUPY48BIT                    0x00300000
#define CAMERA_MEDIA_TYPE_OCCUPY64BIT					 0x00400000

#define CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_MASK      0x00FF0000
#define CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_SHIFT     16

#define CAMERA_MEDIA_TYPE_PIXEL_SIZE(type)                 (((type) & CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_MASK)>>CAMERA_MEDIA_TYPE_EFFECTIVE_PIXEL_SIZE_SHIFT)

#define CAMERA_MEDIA_TYPE_ID_MASK                        0x0000FFFF
#define CAMERA_MEDIA_TYPE_COUNT                          0x46

/*mono*/
#define CAMERA_MEDIA_TYPE_MONO1P             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY1BIT | 0x0037)
#define CAMERA_MEDIA_TYPE_MONO2P             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY2BIT | 0x0038)
#define CAMERA_MEDIA_TYPE_MONO4P             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY4BIT | 0x0039)
#define CAMERA_MEDIA_TYPE_MONO8              (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0001)
#define CAMERA_MEDIA_TYPE_MONO8S             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0002)
#define CAMERA_MEDIA_TYPE_MONO10             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0003)
#define CAMERA_MEDIA_TYPE_MONO10_PACKED      (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0004)
#define CAMERA_MEDIA_TYPE_MONO12             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0005)
#define CAMERA_MEDIA_TYPE_MONO12_PACKED      (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0006)
#define CAMERA_MEDIA_TYPE_MONO14             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0025)
#define CAMERA_MEDIA_TYPE_MONO16             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0007)

/*Bayer */
#define CAMERA_MEDIA_TYPE_BAYGR8             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0008)
#define CAMERA_MEDIA_TYPE_BAYRG8             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x0009)
#define CAMERA_MEDIA_TYPE_BAYGB8             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x000A)
#define CAMERA_MEDIA_TYPE_BAYBG8             (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY8BIT | 0x000B)

#define CAMERA_MEDIA_TYPE_BAYGR10_MIPI       (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY10BIT | 0x0026)
#define CAMERA_MEDIA_TYPE_BAYRG10_MIPI       (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY10BIT | 0x0027)
#define CAMERA_MEDIA_TYPE_BAYGB10_MIPI       (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY10BIT | 0x0028)
#define CAMERA_MEDIA_TYPE_BAYBG10_MIPI       (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY10BIT | 0x0029)


#define CAMERA_MEDIA_TYPE_BAYGR10            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x000C)
#define CAMERA_MEDIA_TYPE_BAYRG10            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x000D)
#define CAMERA_MEDIA_TYPE_BAYGB10            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x000E)
#define CAMERA_MEDIA_TYPE_BAYBG10            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x000F)

#define CAMERA_MEDIA_TYPE_BAYGR12            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0010)
#define CAMERA_MEDIA_TYPE_BAYRG12            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0011)
#define CAMERA_MEDIA_TYPE_BAYGB12            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0012)
#define CAMERA_MEDIA_TYPE_BAYBG12            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0013)


#define CAMERA_MEDIA_TYPE_BAYGR10_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0026)
#define CAMERA_MEDIA_TYPE_BAYRG10_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0027)
#define CAMERA_MEDIA_TYPE_BAYGB10_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0028)
#define CAMERA_MEDIA_TYPE_BAYBG10_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0029)

#define CAMERA_MEDIA_TYPE_BAYGR12_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x002A)
#define CAMERA_MEDIA_TYPE_BAYRG12_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x002B)
#define CAMERA_MEDIA_TYPE_BAYGB12_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x002C)
#define CAMERA_MEDIA_TYPE_BAYBG12_PACKED     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x002D)

#define CAMERA_MEDIA_TYPE_BAYGR16            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x002E)
#define CAMERA_MEDIA_TYPE_BAYRG16            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x002F)
#define CAMERA_MEDIA_TYPE_BAYGB16            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0030)
#define CAMERA_MEDIA_TYPE_BAYBG16            (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0031)

/*RGB */
#define CAMERA_MEDIA_TYPE_RGB8               (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0014)
#define CAMERA_MEDIA_TYPE_BGR8               (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0015)
#define CAMERA_MEDIA_TYPE_RGBA8              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY32BIT | 0x0016)
#define CAMERA_MEDIA_TYPE_BGRA8              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY32BIT | 0x0017)
#define CAMERA_MEDIA_TYPE_RGB10              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0018)
#define CAMERA_MEDIA_TYPE_BGR10              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0019)
#define CAMERA_MEDIA_TYPE_RGB12              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x001A)
#define CAMERA_MEDIA_TYPE_BGR12              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x001B)
#define CAMERA_MEDIA_TYPE_RGB16              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0033)
#define CAMERA_MEDIA_TYPE_BGR16              (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x004B)
#define CAMERA_MEDIA_TYPE_RGBA16             (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY64BIT | 0x0064)
#define CAMERA_MEDIA_TYPE_BGRA16             (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY64BIT | 0x0051)
#define CAMERA_MEDIA_TYPE_RGB10V1_PACKED     (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY32BIT | 0x001C)
#define CAMERA_MEDIA_TYPE_RGB10P32           (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY32BIT | 0x001D)
#define CAMERA_MEDIA_TYPE_RGB12V1_PACKED     (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY36BIT | 0X0034)
#define CAMERA_MEDIA_TYPE_RGB565P            (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0035)
#define CAMERA_MEDIA_TYPE_BGR565P            (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0X0036)

/*YUV and YCbCr*/
#define CAMERA_MEDIA_TYPE_YUV411_8_UYYVYY    (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x001E)
#define CAMERA_MEDIA_TYPE_YUV422_8_UYVY      (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x001F)
#define CAMERA_MEDIA_TYPE_YUV422_8           (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0032)
#define CAMERA_MEDIA_TYPE_YUV8_UYV           (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0020)
#define CAMERA_MEDIA_TYPE_YCBCR8_CBYCR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x003A)
//CAMERA_MEDIA_TYPE_YCBCR422_8 : YYYYCbCrCbCr
#define CAMERA_MEDIA_TYPE_YCBCR422_8             (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x003B)
#define CAMERA_MEDIA_TYPE_YCBCR422_8_CBYCRY      (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0043)
#define CAMERA_MEDIA_TYPE_YCBCR411_8_CBYYCRYY    (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x003C)
#define CAMERA_MEDIA_TYPE_YCBCR601_8_CBYCR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x003D)
#define CAMERA_MEDIA_TYPE_YCBCR601_422_8         (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x003E)
#define CAMERA_MEDIA_TYPE_YCBCR601_422_8_CBYCRY  (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0044)
#define CAMERA_MEDIA_TYPE_YCBCR601_411_8_CBYYCRYY    (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x003F)
#define CAMERA_MEDIA_TYPE_YCBCR709_8_CBYCR           (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0040)
#define CAMERA_MEDIA_TYPE_YCBCR709_422_8             (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0041)
#define CAMERA_MEDIA_TYPE_YCBCR709_422_8_CBYCRY      (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY16BIT | 0x0045)
#define CAMERA_MEDIA_TYPE_YCBCR709_411_8_CBYYCRYY    (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0042)

/*RGB Planar */
#define CAMERA_MEDIA_TYPE_RGB8_PLANAR        (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY24BIT | 0x0021)
#define CAMERA_MEDIA_TYPE_RGB10_PLANAR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0022)
#define CAMERA_MEDIA_TYPE_RGB12_PLANAR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0023)
#define CAMERA_MEDIA_TYPE_RGB16_PLANAR       (CAMERA_MEDIA_TYPE_COLOR | CAMERA_MEDIA_TYPE_OCCUPY48BIT | 0x0024)



/*MindVision 12bit packed bayer*/
#define CAMERA_MEDIA_TYPE_BAYGR12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0060)
#define CAMERA_MEDIA_TYPE_BAYRG12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0061)
#define CAMERA_MEDIA_TYPE_BAYGB12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0062)
#define CAMERA_MEDIA_TYPE_BAYBG12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0063)

/*MindVision 12bit packed monochome*/
#define CAMERA_MEDIA_TYPE_MONO12_PACKED_MV     (CAMERA_MEDIA_TYPE_MONO | CAMERA_MEDIA_TYPE_OCCUPY12BIT | 0x0064)
#endif
