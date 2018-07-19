/**
@File      GxIAPI.h
@Brief     the interface for the GxIAPI dll module. 
@Author    Software Department
@Date      2017-8-1
@Version   1.1.1708.9011
*/

#ifndef GX_GALAXY_H
#define GX_GALAXY_H


//////////////////////////////////////////////////////////////////////////
//	类型定义，以下类型都在标准C库头文件stdint.h中有定义，但是在微软的编译平台
//	VS2010之前的版本中都不包含此文件,所以在此需要重定义
//////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
	#ifndef _STDINT_H 
		#ifdef _MSC_VER // Microsoft compiler
			#if _MSC_VER < 1600
				typedef __int8            int8_t;
				typedef __int16           int16_t;
				typedef __int32           int32_t;
				typedef __int64           int64_t;
				typedef unsigned __int8   uint8_t;
				typedef unsigned __int16  uint16_t;
				typedef unsigned __int32  uint32_t;
				typedef unsigned __int64  uint64_t;
			#else
				// In Visual Studio 2010 is stdint.h already included
				#include <stdint.h>
			#endif
		#else
			// Not a Microsoft compiler
			#include <stdint.h>
		#endif
	#endif 
#else
	// Linux
	#include <stdint.h>
#endif


//------------------------------------------------------------------------------
//  操作系统平台定义
//------------------------------------------------------------------------------

#include <stddef.h>

#ifdef WIN32
	#ifndef _WIN32
		#define _WIN32
	#endif
#endif

#ifdef _WIN32
	#include <Windows.h>
	#define GX_DLLIMPORT   __declspec(dllimport)
	#define GX_DLLEXPORT   __declspec(dllexport)

	#define GX_STDC __stdcall
	#define GX_CDEC __cdecl

	#if defined(__cplusplus)
		#define GX_EXTC extern "C"
	#else
		#define GX_EXTC
	#endif
#else
	// remove the None #define conflicting with GenApi
	#undef None
	#if __GNUC__>=4
		#define GX_DLLIMPORT   __attribute__((visibility("default")))
		#define GX_DLLEXPORT   __attribute__((visibility("default")))

		#if defined(__i386__)
			#define GX_STDC __attribute__((stdcall))
			#define GX_CDEC __attribute__((cdecl))
		#else
			#define GX_STDC 
			#define GX_CDEC 
		#endif

		#if defined(__cplusplus)
			#define GX_EXTC extern "C"
		#else
			#define GX_EXTC
		#endif
	#else
		#error Unknown compiler
	#endif
#endif

#ifdef GX_GALAXY_DLL
	#define GX_DLLENTRY GX_EXTC GX_DLLEXPORT
#else
	#define GX_DLLENTRY GX_EXTC GX_DLLIMPORT
#endif

//------------------------------------------------------------------------------
//  错误码定义
//------------------------------------------------------------------------------
typedef enum GX_STATUS_LIST
{
	GX_STATUS_SUCCESS                =  0,           ///< 成功
	GX_STATUS_ERROR                  = -1,           ///< 不期望发生的未明确指明的内部错误
	GX_STATUS_NOT_FOUND_TL           = -2,           ///< 找不到TL库
	GX_STATUS_NOT_FOUND_DEVICE       = -3,           ///< 找不到设备
	GX_STATUS_OFFLINE                = -4,           ///< 当前设备为掉线状态
	GX_STATUS_INVALID_PARAMETER      = -5,           ///< 无效参数,一般是指针为NULL或输入的IP等参数格式无效
	GX_STATUS_INVALID_HANDLE         = -6,           ///< 无效句柄
	GX_STATUS_INVALID_CALL           = -7,           ///< 无效的接口调用,专指软件接口逻辑错误
	GX_STATUS_INVALID_ACCESS         = -8,           ///< 功能当前不可访问或设备访问模式错误
	GX_STATUS_NEED_MORE_BUFFER       = -9,           ///< 用户申请的buffer不足:读操作时用户输入buffersize小于实际需要
	GX_STATUS_ERROR_TYPE             = -10,          ///< 用户使用的FeatureID类型错误，比如整型接口使用了浮点型的功能码
	GX_STATUS_OUT_OF_RANGE           = -11,          ///< 用户写入的值越界
	GX_STATUS_NOT_IMPLEMENTED        = -12,          ///< 当前不支持的功能
	GX_STATUS_NOT_INIT_API           = -13,          ///< 没有调用初始化接口
	GX_STATUS_TIMEOUT                = -14,          ///< 超时错误
}GX_STATUS_LIST;
typedef int32_t GX_STATUS;

//------------------------------------------------------------------------------
//  帧状态码定义
//------------------------------------------------------------------------------
typedef enum GX_FRAME_STATUS_LIST
{
	GX_FRAME_STATUS_SUCCESS          = 0,     ///< 正常帧
	GX_FRAME_STATUS_INCOMPLETE       = -1,    ///< 残帧
}GX_FRAME_STATUS_LIST;
typedef  int32_t  GX_FRAME_STATUS;

//------------------------------------------------------------------------------
//  设备类型码定义
//------------------------------------------------------------------------------
typedef enum GX_DEVICE_CLASS_LIST
{
	GX_DEVICE_CLASS_UNKNOWN = 0,     ///< 未知设备类型
	GX_DEVICE_CLASS_USB2    = 1,     ///< USB2.0设备
	GX_DEVICE_CLASS_GEV     = 2,     ///< 千兆网设备
	GX_DEVICE_CLASS_U3V     = 3,     ///< USB3.0设备
}GX_DEVICE_CLASS_LIST;
typedef  int32_t GX_DEVICE_CLASS;

//------------------------------------------------------------------------------
//  功能码信息
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_MASK
{
    GX_FEATURE_TYPE_MASK           =0xF0000000,         ///功能码type信息
    GX_FEATURE_LEVEL_MASK          =0x0F000000,         ///功能码层级信息
}GX_FEATURE_MASK;

//------------------------------------------------------------------------------
//  功能码类型定义
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_TYPE
{
	GX_FEATURE_INT				   =0x10000000,  ///< 整型数
	GX_FEATURE_FLOAT               =0X20000000,  ///< 浮点数
	GX_FEATURE_ENUM				   =0x30000000,  ///< 枚举
	GX_FEATURE_BOOL				   =0x40000000,  ///< 布尔
	GX_FEATURE_STRING			   =0x50000000,  ///< 字符串
	GX_FEATURE_BUFFER			   =0x60000000,  ///< buffer
	GX_FEATURE_COMMAND			   =0x70000000,  ///< 命令
}GX_FEATURE_TYPE;

//------------------------------------------------------------------------------
//  功能码所属层级定义
//------------------------------------------------------------------------------
typedef enum GX_FEATURE_LEVEL
{
	GX_FEATURE_LEVEL_REMOTE_DEV	    =0x00000000,  ///< RemoteDevice层
	GX_FEATURE_LEVEL_TL				=0x01000000,  ///< TL层
	GX_FEATURE_LEVEL_IF             =0x02000000,  ///< Interface层	
	GX_FEATURE_LEVEL_DEV		    =0x03000000,  ///< Device层
	GX_FEATURE_LEVEL_DS			    =0x04000000,  ///< DataStream层
}GX_FEATURE_LEVEL;

//------------------------------------------------------------------------------
//  设备的访问方式
//------------------------------------------------------------------------------
typedef enum GX_ACCESS_MODE
{
	GX_ACCESS_READONLY      =2,        ///< 只读方式
	GX_ACCESS_CONTROL       =3,        ///< 控制方式
	GX_ACCESS_EXCLUSIVE     =4,        ///< 独占方式
}GX_ACCESS_MODE;
typedef int32_t GX_ACCESS_MODE_CMD;

//------------------------------------------------------------------------------
//  当前设备的可访问方式
//------------------------------------------------------------------------------
typedef enum GX_ACCESS_STATUS
{
	GX_ACCESS_STATUS_UNKNOWN    = 0,   ///< 设备当前状态未知
	GX_ACCESS_STATUS_READWRITE  = 1,   ///< 设备当前可读可写
	GX_ACCESS_STATUS_READONLY   = 2,   ///< 设备当前只支持读
	GX_ACCESS_STATUS_NOACCESS   = 3,   ///< 设备当前既不支持读，又不支持写
}GX_ACCESS_STATUS;
typedef int32_t GX_ACCESS_STATUS_CMD;

//------------------------------------------------------------------------------
//  设备的打开方式
//------------------------------------------------------------------------------
typedef enum GX_OPEN_MODE
{
	GX_OPEN_SN              =0,        ///< 通过SN打开
	GX_OPEN_IP              =1,        ///< 通过IP打开
	GX_OPEN_MAC             =2,        ///< 通过MAC打开
	GX_OPEN_INDEX           =3,        ///< 通过Index打开
	GX_OPEN_USERID          =4,        ///< 通过用户自定义ID打开
}GX_OPEN_MODE;
typedef int32_t GX_OPEN_MODE_CMD;

typedef enum GX_FEATURE_ID
{
	//////////////////////////////////////////////////////////////////////////
	/// 远端设备层(Remote Device Feature)
	//////////////////////////////////////////////////////////////////////////

	//---------------DeviceInfomation Section--------------------------
	GX_STRING_DEVICE_VENDOR_NAME               = 0   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 厂商名称
	GX_STRING_DEVICE_MODEL_NAME                = 1   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备型号
	GX_STRING_DEVICE_FIRMWARE_VERSION          = 2   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备固件版本
	GX_STRING_DEVICE_VERSION                   = 3   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备版本
	GX_STRING_DEVICE_SERIAL_NUMBER             = 4   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备序列号
	GX_STRING_FACTORY_SETTING_VERSION          = 6   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 出厂参数版本
	GX_STRING_DEVICE_USERID                    = 7   | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 用户自定义名称
	GX_INT_DEVICE_LINK_SELECTOR                = 8   | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备链路选择
	GX_ENUM_DEVICE_LINK_THROUGHPUT_LIMIT_MODE  = 9   | GX_FEATURE_ENUM   | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备带宽限制模式，参考GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY
	GX_INT_DEVICE_LINK_THROUGHPUT_LIMIT        = 10  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 设备链路带宽限制
	GX_INT_DEVICE_LINK_CURRENT_THROUGHPUT      = 11  | GX_FEATURE_INT    | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 当前设备采集带宽


	//---------------ImageFormat Section--------------------------------
	GX_INT_WIDTH_MAX                  = 1002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 最大宽度
	GX_INT_HEIGHT_MAX                 = 1003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 最大高度
	GX_INT_OFFSET_X                   = 1004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 水平偏移
	GX_INT_OFFSET_Y                   = 1005 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 垂直偏移
	GX_INT_WIDTH                      = 1006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 图像宽度
	GX_INT_HEIGHT                     = 1007 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 图像高度
	GX_INT_BINNING_HORIZONTAL         = 1008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 水平像素Binning
	GX_INT_BINNING_VERTICAL           = 1009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 垂直像素Binning
	GX_INT_DECIMATION_HORIZONTAL      = 1010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 水平像素抽样
	GX_INT_DECIMATION_VERTICAL        = 1011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 垂直像素抽样
	GX_ENUM_PIXEL_SIZE                = 1012 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 像素位深,参考GX_PIXEL_SIZE_ENTRY
	GX_ENUM_PIXEL_COLOR_FILTER        = 1013 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< Bayer格式,参考GX_PIXEL_COLOR_FILTER_ENTRY
	GX_ENUM_PIXEL_FORMAT              = 1014 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 像素格式,参考GX_PIXEL_FORMAT_ENTRY
	GX_BOOL_REVERSE_X                 = 1015 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 水平翻转
	GX_BOOL_REVERSE_Y                 = 1016 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 垂直翻转
	GX_ENUM_TEST_PATTERN              = 1017 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 测试图,参考GX_TEST_PATTERN_ENTRY
	GX_ENUM_TEST_PATTERN_GENERATOR_SELECTOR = 1018 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 测试图源选择，参考GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY

	//---------------AcquisitionTrigger Section---------------------------
	GX_ENUM_ACQUISITION_MODE          = 3000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 采集模式,参考GX_ACQUISITION_MODE_ENTRY
	GX_COMMAND_ACQUISITION_START      = 3001 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 开始采集
	GX_COMMAND_ACQUISITION_STOP       = 3002 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 停止采集
	GX_INT_ACQUISITION_SPEED_LEVEL    = 3003 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 采集速度级别
	GX_INT_ACQUISITION_FRAME_COUNT    = 3004 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 多帧采集帧数
	GX_ENUM_TRIGGER_MODE              = 3005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发模式,参考GX_TRIGGER_MODE_ENTRY
	GX_COMMAND_TRIGGER_SOFTWARE       = 3006 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 软触发
	GX_ENUM_TRIGGER_ACTIVATION        = 3007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发极性,参考GX_TRIGGER_ACTIVATION_ENTRY
	GX_ENUM_TRIGGER_SWITCH            = 3008 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 外触发开关,参考GX_TRIGGER_SWITCH_ENTRY
	GX_FLOAT_EXPOSURE_TIME            = 3009 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 曝光时间
	GX_ENUM_EXPOSURE_AUTO             = 3010 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动曝光,参考GX_EXPOSURE_AUTO_ENTRY
	GX_FLOAT_TRIGGER_FILTER_RAISING   = 3011 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 上升沿触发滤波
	GX_FLOAT_TRIGGER_FILTER_FALLING   = 3012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 下降沿触发滤波
	GX_ENUM_TRIGGER_SOURCE            = 3013 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发源,参考GX_TRIGGER_SOURCE_ENTRY
	GX_ENUM_EXPOSURE_MODE             = 3014 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 曝光模式，参考GX_EXPOSURE_MODE_ENTRY
	GX_ENUM_TRIGGER_SELECTOR          = 3015 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 触发类型选择，参考GX_TRIGGER_SELECTOR_ENTRY
	GX_FLOAT_TRIGGER_DELAY            = 3016 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 触发延迟
	GX_ENUM_TRANSFER_CONTROL_MODE     = 3017 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 传输控制模式，参考GX_TRANSFER_CONTROL_MODE_ENTRY
	GX_ENUM_TRANSFER_OPERATION_MODE   = 3018 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 传输操作模式，参考GX_TRANSFER_OPERATION_MODE_ENTRY
	GX_COMMAND_TRANSFER_START         = 3019 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 开始传输
	GX_INT_TRANSFER_BLOCK_COUNT       = 3020 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,     ///< 传输帧数。当GX_ENUM_TRANSFER_OPERATION_MODE设置为GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK模式的时候此功能被激活
	GX_BOOL_FRAMESTORE_COVER_ACTIVE   = 3021 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 帧存覆盖使能


	//----------------DigitalIO Section----------------------------------
	GX_ENUM_USER_OUTPUT_SELECTOR      = 4000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 用户自定义输出选择,参考GX_USER_OUTPUT_SELECTOR_ENTRY
	GX_BOOL_USER_OUTPUT_VALUE         = 4001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 用户自定义输出值
	GX_ENUM_USER_OUTPUT_MODE          = 4002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 用户IO输出模式,参考GX_USER_OUTPUT_MODE_ENTRY
	GX_ENUM_STROBE_SWITCH             = 4003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 闪光灯开关,参考GX_STROBE_SWITCH_ENTRY
	GX_ENUM_LINE_SELECTOR             = 4004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚选择,参考GX_LINE_SELECTOR_ENTRY
	GX_ENUM_LINE_MODE                 = 4005 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚方向,参考GX_LINE_MODE_ENTRY
	GX_BOOL_LINE_INVERTER             = 4006 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚电平反转
	GX_ENUM_LINE_SOURCE               = 4007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚输出源,参考GX_LINE_SOURCE_ENTRY
	GX_BOOL_LINE_STATUS               = 4008 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 引脚状态
	GX_INT_LINE_STATUS_ALL            = 4009 | GX_FEATURE_INT  | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 所有引脚的状态

	//----------------AnalogControls Section----------------------------
	GX_ENUM_GAIN_AUTO                 = 5000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动增益,参考GX_GAIN_AUTO_ENTRY
	GX_ENUM_GAIN_SELECTOR             = 5001 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 增益通道选择,参考GX_GAIN_SELECTOR_ENTRY	
	GX_ENUM_BLACKLEVEL_AUTO           = 5003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动黑电平,参考GX_BLACKLEVEL_AUTO_ENTRY
	GX_ENUM_BLACKLEVEL_SELECTOR       = 5004 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 黑电平通道选择,参考GX_BLACKLEVEL_SELECTOR_ENTRY	
	GX_ENUM_BALANCE_WHITE_AUTO        = 5006 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动白平衡,参考GX_BALANCE_WHITE_AUTO_ENTRY
	GX_ENUM_BALANCE_RATIO_SELECTOR    = 5007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 白平衡通道选择,参考GX_BALANCE_RATIO_SELECTOR_ENTRY
	GX_FLOAT_BALANCE_RATIO            = 5008 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 白平衡系数
	GX_ENUM_COLOR_CORRECT             = 5009 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 颜色校正,参考GX_COLOR_CORRECT_ENTRY
	GX_ENUM_DEAD_PIXEL_CORRECT        = 5010 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 坏点校正,参考GX_DEAD_PIXEL_CORRECT_ENTRY
	GX_FLOAT_GAIN                     = 5011 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 增益
	GX_FLOAT_BLACKLEVEL               = 5012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 黑电平

	//---------------CustomFeature Section-------------------------
	GX_INT_ADC_LEVEL                  = 6000 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< AD转换级别
	GX_INT_H_BLANKING                 = 6001 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 水平消隐
	GX_INT_V_BLANKING                 = 6002 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 垂直消隐
	GX_STRING_USER_PASSWORD           = 6003 | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 用户加密区密码
	GX_STRING_VERIFY_PASSWORD         = 6004 | GX_FEATURE_STRING | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 用户加密区校验密码
	GX_BUFFER_USER_DATA               = 6005 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 用户加密区内容
	GX_INT_GRAY_VALUE                 = 6006 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 期望灰度值
	GX_ENUM_AA_LIGHT_ENVIRONMENT      = 6007 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动曝光、自动增益，光照环境类型,参考GX_AA_LIGHT_ENVIRMENT_ENTRY
	GX_INT_AAROI_OFFSETX              = 6008 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域X坐标
	GX_INT_AAROI_OFFSETY              = 6009 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域Y坐标
	GX_INT_AAROI_WIDTH                = 6010 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域宽度
	GX_INT_AAROI_HEIGHT               = 6011 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动调节感兴趣区域高度
	GX_FLOAT_AUTO_GAIN_MIN            = 6012 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 自动增益最小值
	GX_FLOAT_AUTO_GAIN_MAX            = 6013 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 自动增益最大值
	GX_FLOAT_AUTO_EXPOSURE_TIME_MIN   = 6014 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 自动曝光最小值
	GX_FLOAT_AUTO_EXPOSURE_TIME_MAX   = 6015 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 自动曝光最大值
	GX_BUFFER_FRAME_INFORMATION       = 6016 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 图像帧信息
	GX_INT_CONTRAST_PARAM             = 6017 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 对比度参数
	GX_FLOAT_GAMMA_PARAM              = 6018 | GX_FEATURE_FLOAT | GX_FEATURE_LEVEL_REMOTE_DEV,  ///< 伽马参数
	GX_INT_COLOR_CORRECTION_PARAM     = 6019 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 颜色校正系数
	GX_ENUM_IMAGE_GRAY_RAISE_SWITCH   = 6020 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 图像亮度拉伸开关,参考GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY
	GX_ENUM_AWB_LAMP_HOUSE            = 6021 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,   ///< 自动白平衡光源,参考GX_AWB_LAMP_HOUSE_ENTRY
	GX_INT_AWBROI_OFFSETX             = 6022 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动白平衡感兴趣区域X坐标
	GX_INT_AWBROI_OFFSETY             = 6023 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动白平衡感兴趣区域Y坐标
	GX_INT_AWBROI_WIDTH               = 6024 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动白平衡感兴趣区域宽度
	GX_INT_AWBROI_HEIGHT              = 6025 | GX_FEATURE_INT | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 自动白平衡感兴趣区域高度

	//---------------UserSetControl Section-------------------------
	GX_ENUM_USER_SET_SELECTOR         = 7000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 参数组选择,参考GX_USER_SET_SELECTOR_ENTRY
	GX_COMMAND_USER_SET_LOAD          = 7001 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 加载参数组
	GX_COMMAND_USER_SET_SAVE          = 7002 | GX_FEATURE_COMMAND | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 保存参数组
	GX_ENUM_USER_SET_DEFAULT          = 7003 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV,    ///< 启动参数组,参考GX_USER_SET_DEFAULT_ENTRY


	//---------------LUT Section-------------------------
	GX_ENUM_LUT_SELECTOR             = 9000 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 查找表选择,参考GX_LUT_SELECTOR_ENTRY
	GX_BUFFER_LUT_VALUEALL           = 9001 | GX_FEATURE_BUFFER | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 查找表内容

	//---------------ChunkData Section-------------------------
	GX_BOOL_CHUNKMODE_ACTIVE         = 10001 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 帧信息使能
	GX_ENUM_CHUNK_SELECTOR           = 10002 | GX_FEATURE_ENUM | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 帧信息项选择，参考GX_CHUNK_SELECTOR_ENTRY
	GX_BOOL_CHUNK_ENABLE             = 10003 | GX_FEATURE_BOOL | GX_FEATURE_LEVEL_REMOTE_DEV, ///< 单项帧信息使能
	

	//////////////////////////////////////////////////////////////////////////
	/// 流层(DataStream Feature)
	//////////////////////////////////////////////////////////////////////////
	GX_DS_INT_ANNOUNCED_BUFFER_COUNT          = 0 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 声明的Buffer个数
	GX_DS_INT_DELIVERED_FRAME_COUNT           = 1 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 接收帧个数(包括残帧)
	GX_DS_INT_LOST_FRAME_COUNT                = 2 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< buffer不足导致的丢帧个数
	GX_DS_INT_INCOMPLETE_FRAME_COUNT          = 3 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS, ///< 接收的残帧个数
	GX_DS_INT_STREAM_TRANSFER_SIZE            = 16 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,  ///< 传输数据块大小
	GX_DS_INT_STREAM_TRANSFER_NUMBER_URB      = 17 | GX_FEATURE_INT | GX_FEATURE_LEVEL_DS,  ///< 设置URB数量


}GX_FEATURE_ID;
typedef int32_t GX_FEATURE_ID_CMD;

//------------------------------------------------------------------------------
//  句柄定义
//------------------------------------------------------------------------------
typedef void* GX_DEV_HANDLE;               ///< 设备句柄，通过GXOpenDevice获取，通过此句柄进行控制与采集
typedef void* GX_EVENT_CALLBACK_HANDLE;    ///< 设备事件回调句柄，注册设备相关事件回调函数，比如设备掉线回调函数
typedef void* GX_FEATURE_CALLBACK_HANDLE;  ///< 设备属性更新回调句柄，注册设备属性更新回调函数的时候获取

//------------------------------------------------------------------------------------
typedef enum GX_PIXEL_SIZE_ENTRY
{
	GX_PIXEL_SIZE_BPP8  = 8,
	GX_PIXEL_SIZE_BPP10 = 10,
	GX_PIXEL_SIZE_BPP12 = 12,
	GX_PIXEL_SIZE_BPP16 = 16,
	GX_PIXEL_SIZE_BPP24 = 24,
	GX_PIXEL_SIZE_BPP30 = 30,
	GX_PIXEL_SIZE_BPP32 = 32,
	GX_PIXEL_SIZE_BPP36 = 36,
	GX_PIXEL_SIZE_BPP48 = 48,
	GX_PIXEL_SIZE_BPP64 = 64,
}GX_PIXEL_SIZE_ENTRY;

typedef enum GX_PIXEL_COLOR_FILTER_ENTRY
{
	GX_COLOR_FILTER_NONE     = 0,                        ///<无
	GX_COLOR_FILTER_BAYER_RG = 1,                        ///<RG格式
	GX_COLOR_FILTER_BAYER_GB = 2,                        ///<GB格式
	GX_COLOR_FILTER_BAYER_GR = 3,                        ///<GR格式
	GX_COLOR_FILTER_BAYER_BG = 4,                        ///<BG格式
}GX_PIXEL_COLOR_FILTER_ENTRY;

#define GX_PIXEL_MONO                  ( 0x01000000 )
#define GX_PIXEL_COLOR                 ( 0x02000000 )

#define GX_PIXEL_8BIT                  ( 0x00080000 )
#define GX_PIXEL_10BIT                 ( 0x000A0000 )
#define GX_PIXEL_12BIT                 ( 0x000C0000 )
#define GX_PIXEL_16BIT                 ( 0x00100000 )
#define GX_PIXEL_24BIT                 ( 0x00180000 )
#define GX_PIXEL_30BIT                 ( 0x001E0000 )
#define GX_PIXEL_32BIT                 ( 0x00200000 )
#define GX_PIXEL_36BIT                 ( 0x00240000 )
#define GX_PIXEL_48BIT                 ( 0x00300000 )
#define GX_PIXEL_64BIT                 ( 0x00400000 )

typedef enum GX_PIXEL_FORMAT_ENTRY
{
	GX_PIXEL_FORMAT_UNDEFINED          = (0),
	GX_PIXEL_FORMAT_MONO8              = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0001),//0x1080001,
	GX_PIXEL_FORMAT_MONO8_SIGNED       = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0002),//0x1080002,
	GX_PIXEL_FORMAT_MONO10             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0003),//0x1100003,	
	GX_PIXEL_FORMAT_MONO12             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0005),//0x1100005,	
	GX_PIXEL_FORMAT_MONO14             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0025),//0x1100025,
	GX_PIXEL_FORMAT_MONO16             = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0007),//0x1100007,
	GX_PIXEL_FORMAT_BAYER_GR8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0008),//0x1080008,               
	GX_PIXEL_FORMAT_BAYER_RG8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x0009),//0x1080009,                
	GX_PIXEL_FORMAT_BAYER_GB8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x000A),//0x108000A,
	GX_PIXEL_FORMAT_BAYER_BG8          = (GX_PIXEL_MONO  | GX_PIXEL_8BIT  | 0x000B),//0x108000B,
	GX_PIXEL_FORMAT_BAYER_GR10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000C),//0x110000C,                
	GX_PIXEL_FORMAT_BAYER_RG10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000D),//0x110000D,
	GX_PIXEL_FORMAT_BAYER_GB10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000E),//0x110000E,
	GX_PIXEL_FORMAT_BAYER_BG10         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x000F),//0x110000F,
	GX_PIXEL_FORMAT_BAYER_GR12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0010),//0x1100010,              
	GX_PIXEL_FORMAT_BAYER_RG12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0011),//0x1100011,
	GX_PIXEL_FORMAT_BAYER_GB12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0012),//0x1100012,
	GX_PIXEL_FORMAT_BAYER_BG12         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0013),//0x1100013,	
	GX_PIXEL_FORMAT_BAYER_GR16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x002E),//0x110002E,                
	GX_PIXEL_FORMAT_BAYER_RG16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x002F),//0x110002F,
	GX_PIXEL_FORMAT_BAYER_GB16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0030),//0x1100030,
	GX_PIXEL_FORMAT_BAYER_BG16         = (GX_PIXEL_MONO  | GX_PIXEL_16BIT | 0x0031),//0x1100031,	
	GX_PIXEL_FORMAT_RGB8_PLANAR        = (GX_PIXEL_COLOR | GX_PIXEL_24BIT | 0x0021),//0x2180021,
	GX_PIXEL_FORMAT_RGB10_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0022),//0x2300022,
	GX_PIXEL_FORMAT_RGB12_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0023),//0x2300023,
	GX_PIXEL_FORMAT_RGB16_PLANAR       = (GX_PIXEL_COLOR | GX_PIXEL_48BIT | 0x0024),//0x2300024,
}GX_PIXEL_FORMAT_ENTRY;

typedef enum GX_ACQUISITION_MODE_ENTRY
{
	GX_ACQ_MODE_SINGLE_FRAME = 0,                          ///<单帧模式
	GX_ACQ_MODE_MULITI_FRAME = 1,                          ///<多帧模式
	GX_ACQ_MODE_CONTINUOUS   = 2,                          ///<连续模式
}GX_ACQUISITION_MODE_ENTRY;

typedef enum GX_TRIGGER_MODE_ENTRY
{
	GX_TRIGGER_MODE_OFF = 0,                             ///< 关闭触发模式
	GX_TRIGGER_MODE_ON  = 1,                             ///< 打开触发模式
}GX_TRIGGER_MODE_ENTRY;

typedef enum GX_TRIGGER_SOURCE_ENTRY
{
	GX_TRIGGER_SOURCE_SOFTWARE = 0,                      ///< 软触发
	GX_TRIGGER_SOURCE_LINE0    = 1,                      ///< 触发源0
	GX_TRIGGER_SOURCE_LINE1    = 2,                      ///< 触发源1
	GX_TRIGGER_SOURCE_LINE2    = 3,                      ///< 触发源2
	GX_TRIGGER_SOURCE_LINE3    = 4,                      ///< 触发源3
}GX_TRIGGER_SOURCE_ENTRY;

typedef enum GX_TRIGGER_ACTIVATION_ENTRY
{
	GX_TRIGGER_ACTIVATION_FALLINGEDGE = 0,               ///< 下降沿触发
	GX_TRIGGER_ACTIVATION_RISINGEDGE  = 1,               ///< 上升沿触发
}GX_TRIGGER_ACTIVATION_ENTRY;

typedef enum GX_TRIGGER_SWITCH_ENTRY
{
	GX_TRIGGER_SWITCH_OFF = 0,                           ///< 关闭外触发
	GX_TRIGGER_SWITCH_ON  = 1,                           ///< 打开外触发
}GX_TRIGGER_SWITCH_ENTRY;

typedef enum GX_EXPOSURE_MODE_ENTRY
{
	GX_EXPOSURE_MODE_TIMED          = 1,                 ///< 曝光时间寄存器控制曝光时间
	GX_EXPOSURE_MODE_TRIGGERWIDTH   = 2,                 ///< 触发信号宽度控制曝光时间
}GX_EXPOSURE_MODE_ENTRY;

typedef enum GX_EXPOSURE_AUTO_ENTRY
{
	GX_EXPOSURE_AUTO_OFF        = 0,                     ///< 关闭自动曝光
	GX_EXPOSURE_AUTO_CONTINUOUS = 1,                     ///< 连续自动曝光
	GX_EXPOSURE_AUTO_ONCE       = 2,                     ///< 单次自动曝光
}GX_EXPOSURE_AUTO_ENTRY;

typedef enum GX_USER_OUTPUT_SELECTOR_ENTRY
{
	GX_USER_OUTPUT_SELECTOR_OUTPUT0 = 1,                   ///<输出0
	GX_USER_OUTPUT_SELECTOR_OUTPUT1 = 2,                   ///<输出1
	GX_USER_OUTPUT_SELECTOR_OUTPUT2 = 4,                   ///<输出2
}GX_USER_OUTPUT_SELECTOR_ENTRY;

typedef enum GX_USER_OUTPUT_MODE_ENTRY
{
	GX_USER_OUTPUT_MODE_STROBE      = 0,                   ///<闪光灯
	GX_USER_OUTPUT_MODE_USERDEFINED = 1,                   ///<用户自定义
}GX_USER_OUTPUT_MODE_ENTRY;

typedef enum GX_STROBE_SWITCH_ENTRY
{
	GX_STROBE_SWITCH_OFF = 0,                            ///< 关闭闪光灯开关
	GX_STROBE_SWITCH_ON  = 1,                            ///< 打开闪光灯开关
}GX_STROBE_SWITCH_ENTRY;

typedef enum GX_GAIN_AUTO_ENTRY
{
	GX_GAIN_AUTO_OFF        = 0,                         ///< 关闭自动增益
	GX_GAIN_AUTO_CONTINUOUS = 1,                         ///< 连续自动增益
	GX_GAIN_AUTO_ONCE       = 2,                         ///< 单次自动增益
}GX_GAIN_AUTO_ENTRY;

typedef enum GX_GAIN_SELECTOR_ENTRY
{
	GX_GAIN_SELECTOR_ALL   = 0,                          ///< 所有增益通道
	GX_GAIN_SELECTOR_RED   = 1,                          ///< 红通道增益
	GX_GAIN_SELECTOR_GREEN = 2,                          ///< 绿通道增益
	GX_GAIN_SELECTOR_BLUE  = 3,                          ///< 蓝通道增益
}GX_GAIN_SELECTOR_ENTRY;

typedef enum GX_BLACKLEVEL_AUTO_ENTRY
{
	GX_BLACKLEVEL_AUTO_OFF        = 0,                   ///< 关闭自动黑电平
	GX_BLACKLEVEL_AUTO_CONTINUOUS = 1,                   ///< 连续自动黑电平
	GX_BLACKLEVEL_AUTO_ONCE       = 2,                   ///< 单次自动黑电平
}GX_BLACKLEVEL_AUTO_ENTRY;

typedef enum GX_BLACKLEVEL_SELECTOR_ENTRY
{
	GX_BLACKLEVEL_SELECTOR_ALL   = 0,                    ///< 所有黑电平通道
	GX_BLACKLEVEL_SELECTOR_RED   = 1,                    ///< 红通道黑电平
	GX_BLACKLEVEL_SELECTOR_GREEN = 2,                    ///< 绿通道黑电平
	GX_BLACKLEVEL_SELECTOR_BLUE  = 3,                    ///< 蓝通道黑电平
}GX_BLACKLEVEL_SELECTOR_ENTRY;

typedef enum GX_BALANCE_WHITE_AUTO_ENTRY
{
	GX_BALANCE_WHITE_AUTO_OFF        = 0,                ///< 关闭自动白平衡
	GX_BALANCE_WHITE_AUTO_CONTINUOUS = 1,                ///< 连续自动白平衡
	GX_BALANCE_WHITE_AUTO_ONCE       = 2,                ///< 单次自动白平衡
}GX_BALANCE_WHITE_AUTO_ENTRY;

typedef enum GX_BALANCE_RATIO_SELECTOR_ENTRY
{
	GX_BALANCE_RATIO_SELECTOR_RED   = 0,                   ///<红通道
	GX_BALANCE_RATIO_SELECTOR_GREEN = 1,                   ///<绿通道
	GX_BALANCE_RATIO_SELECTOR_BLUE  = 2,                   ///<蓝通道
}GX_BALANCE_RATIO_SELECTOR_ENTRY;

typedef enum GX_COLOR_CORRECT_ENTRY
{
	GX_COLOR_CORRECT_OFF = 0,                            ///< 关闭自动颜色校正
	GX_COLOR_CORRECT_ON  = 1,                            ///< 打开自动颜色校正
}GX_COLOR_CORRECT_ENTRY;

typedef enum GX_DEAD_PIXEL_CORRECT_ENTRY
{
	GX_DEAD_PIXEL_CORRECT_OFF = 0,                       ///< 关闭自动坏点校正
	GX_DEAD_PIXEL_CORRECT_ON  = 1,                       ///< 打开自动坏点校正
}GX_DEAD_PIXEL_CORRECT_ENTRY;

typedef enum GX_AA_LIGHT_ENVIRMENT_ENTRY
{
	GX_AA_LIGHT_ENVIRMENT_NATURELIGHT = 0,                 ///<自然光
	GX_AA_LIGHT_ENVIRMENT_AC50HZ      = 1,                 ///<50赫兹日光灯
	GX_AA_LIGHT_ENVIRMENT_AC60HZ      = 2,                 ///<60赫兹日光灯
}GX_AA_LIGHT_ENVIRMENT_ENTRY;

typedef enum GX_USER_SET_SELECTOR_ENTRY
{
	GX_ENUM_USER_SET_SELECTOR_DEFAULT  = 0,                 ///<默认参数组
	GX_ENUM_USER_SET_SELECTOR_USERSET0 = 1,                 ///<用户参数组0
}GX_USER_SET_SELECTOR_ENTRY;

typedef enum GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY
{
	GX_IMAGE_GRAY_RAISE_SWITCH_OFF = 0,                     ///< 图像亮度拉伸开关
	GX_IMAGE_GRAY_RAISE_SWITCH_ON  = 1,                     ///< 图像亮度拉伸开关
}GX_IMAGE_GRAY_RAISE_SWITCH_ENTRY;

typedef enum GX_AWB_LAMP_HOUSE_ENTRY
{
	GX_AWB_LAMP_HOUSE_ADAPTIVE      = 0,                      ///< 自适应光源
	GX_AWB_LAMP_HOUSE_D65           = 1,                      ///< 指定色温6500k
	GX_AWB_LAMP_HOUSE_FLUORESCENCE  = 2,                      ///< 指定荧光灯
	GX_AWB_LAMP_HOUSE_INCANDESCENT  = 3,                      ///< 指定白炽灯
	GX_AWB_LAMP_HOUSE_D75           = 4,                      ///< 指定色温7500k
	GX_AWB_LAMP_HOUSE_D50           = 5,                      ///< 指定色温5000k
	GX_AWB_LAMP_HOUSE_D30           = 6,                      ///< 指定色温3000k
}GX_AWB_LAMP_HOUSE_ENTRY;

typedef enum GX_TEST_PATTERN_ENTRY
{
	GX_ENUM_TEST_PATTERN_OFF                    = 0,            ///<关闭
	GX_ENUM_TEST_PATTERN_GRAY_FRAME_RAMP_MOVING = 1,            ///<静止灰度递增
	GX_ENUM_TEST_PATTERN_SLANT_LINE_MOVING      = 2,            ///<滚动斜条纹
	GX_ENUM_TEST_PATTERN_VERTICAL_LINE_MOVING   = 3,            ///<滚动竖条纹
}GX_TEST_PATTERN_ENTRY;

typedef enum GX_TRIGGER_SELECTOR_ENTRY
{
	GX_ENUM_TRIGGER_SELECTOR_FRAME_START        = 1,               ///<采集一帧
} GX_TRIGGER_SELECTOR_ENTRY;

typedef enum GX_LINE_SELECTOR_ENTRY
{
	GX_ENUM_LINE_SELECTOR_LINE0           = 0,               ///<引脚0
	GX_ENUM_LINE_SELECTOR_LINE1           = 1,               ///<引脚1
	GX_ENUM_LINE_SELECTOR_LINE2           = 2,               ///<引脚2
	GX_ENUM_LINE_SELECTOR_LINE3           = 3,               ///<引脚3
} GX_LINE_SELECTOR_ENTRY;

typedef enum GX_LINE_MODE_ENTRY
{
	GX_ENUM_LINE_MODE_INPUT            = 0,               ///<输入
	GX_ENUM_LINE_MODE_OUTPUT           = 1,               ///<输出
} GX_LINE_MODE_ENTRY;

typedef enum GX_LINE_SOURCE_ENTRY
{
	GX_ENUM_LINE_SOURCE_OFF               = 0,       ///<关闭
	GX_ENUM_LINE_SOURCE_STROBE            = 1,       ///<闪光灯
	GX_ENUM_LINE_SOURCE_USEROUTPUT0       = 2,       ///<用户自定义输出0
	GX_ENUM_LINE_SOURCE_USEROUTPUT1       = 3,       ///<用户自定义输出1
	GX_ENUM_LINE_SOURCE_USEROUTPUT2       = 4,       ///<用户自定义输出2
} GX_LINE_SOURCE_ENTRY;


typedef enum GX_LUT_SELECTOR_ENTRY
{
	GX_ENUM_LUT_SELECTOR_LUMINANCE             = 0,       ///<亮度
} GX_LUT_SELECTOR_ENTRY;

typedef enum GX_TRANSFERDELAY_MODE_ENTRY
{
	GX_ENUM_TRANSFERDELAY_MODE_OFF     = 0,       ///<禁用传输延迟
	GX_ENUM_TRANSFERDELAY_MODE_ON      = 1,       ///<开启传输延迟
} GX_TRANSFERDELAY_MODE_ENTRY;

typedef enum GX_COVER_FRAMESTORE_MODE_ENTRY
{
	GX_ENUM_COVER_FRAMESTORE_MODE_OFF     = 0,       ///<禁用帧存覆盖
	GX_ENUM_COVER_FRAMESTORE_MODE_ON      = 1,       ///<开启帧存覆盖
} GX_COVER_FRAMESTORE_MODE_ENTRY;

typedef enum GX_USER_SET_DEFAULT_ENTRY
{
	GX_ENUM_USER_SET_DEFAULT_DEFAULT      = 0,       ///<出厂参数组
	GX_ENUM_USER_SET_DEFAULT_USERSET0     = 1,       ///<用户参数组0
} GX_USER_SET_DEFAULT_ENTRY;

typedef enum GX_TRANSFER_CONTROL_MODE_ENTRY
{
	GX_ENUM_TRANSFER_CONTROL_MODE_BASIC             = 0,   ///< 关闭传输控制模式
	GX_ENUM_TRANSFER_CONTROL_MODE_USERCONTROLED     = 1,   ///< 用户控制传输控制模式
} GX_TRANSFER_CONTROL_MODE_ENTRY;

typedef enum GX_TRANSFER_OPERATION_MODE_ENTRY
{
	GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK  = 0,  ///< 指定发送帧数
} GX_TRANSFER_OPERATION_MODE_ENTRY;

typedef enum GX_DS_RESEND_MODE_ENTRY
{
	GX_DS_RESEND_MODE_OFF     = 0,  ///< 关闭重传模式     
	GX_DS_RESEND_MODE_ON      = 1,  ///< 开启重传模式
} GX_DS_RESEND_MODE_ENTRY;

typedef enum GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY
{
	GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_OFF   = 0,   ///< 关闭设备带宽限制模式
	GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ON    = 1    ///< 开启设备带宽限制模式
}GX_DEVICE_LINK_THROUGHPUT_LIMIT_MODE_ENTRY;

typedef enum GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY
{
	GX_TEST_PATTERN_GENERATOR_SELECTOR_SENSOR  = 0,  ///< sensor 的测试图
	GX_TEST_PATTERN_GENERATOR_SELECTOR_REGION0 = 1   ///< FPGA的测试图
}GX_TEST_PATTERN_GENERATOR_SELECTOR_ENTRY;


typedef enum GX_CHUNK_SELECTOR_ENTRY
{
	GX_CHUNK_SELECTOR_CHUNK_FRAME_ID     = 1,    ///< 帧号
	GX_CHUNK_SELECTOR_CHUNK_TIME_STAMP   = 2     ///< 时间戳
}GX_CHUNK_SELECTOR_ENTRY;


//------------------------------------------------------------------------------
//  结构体类型定义
//------------------------------------------------------------------------------

#define GX_INFO_LENGTH_8_BYTE   (8)  ///< 8字节
#define GX_INFO_LENGTH_32_BYTE  (32) ///< 32字节
#define GX_INFO_LENGTH_64_BYTE  (64) ///< 64字节
#define GX_INFO_LENGTH_128_BYTE (128)///< 128字节


typedef struct GX_FRAME_DATA
{
	GX_FRAME_STATUS		nStatus;             ///< 图像的返回状态
	void*				pImgBuf;             ///< 图像buffer地址（开启chunkdata后，pImgBuf 包含图像数据和帧信息数据 ）
	int32_t				nWidth;              ///< 图像的宽
	int32_t				nHeight;             ///< 图像的高
	int32_t				nPixelFormat;        ///< 图像的PixFormat
	int32_t				nImgSize;            ///< 图像大小数据大小，单位字节（开启chunkdata后，nImgsize为图像数据大小+帧信息大小）
	uint64_t			nFrameID;            ///< 图像的帧号
	uint64_t			nTimestamp;          ///< 图像的时间戳
	uint64_t			nBufID;              ///< BufID
	int32_t				reserved[1];         ///< 保留
}GX_FRAME_DATA;

//------------------------------------------------------------------------------
//  标准C API功能函数定义
//------------------------------------------------------------------------------
#define GX_API GX_EXTC GX_STATUS GX_STDC

//------------------------------------------------------------------------
/**
\brief     获取程序最后的错误描述信息
\attention 当用户调用其它接口失败的时候，可以调用此接口获取关于失败信息的详细描述
\param     [out] pErrorCode    返回最后的错误码，如果用户不想获取此值，那么此参数可以传NULL
\param     [out] pszErrText    返回错误信息缓冲区地址
\param     [in,out] pSize      错误信息缓冲区地址大小，单位字节
           如果pszErrText为NULL：
           [out]pnSize返回实际需要的buffer大小
           如果pszErrText非NULL：
           [in]pnSize为实际分配的buffer大小
           [out]pnSize返回实际填充buffer大小
\return    GX_STATUS_SUCCESS                操作成功，没有发生错误
           GX_STATUS_INVALID_PARAMETER      用户输入的指针为NULL
	   GX_STATUS_NEED_MORE_BUFFER       用户分配的buffer过小
	   其它错误情况请参见GX_STATUS_LIST
*/
//------------------------------------------------------------------------
GX_API GXGetLastError             (GX_STATUS *pErrorCode, char *pszErrText, size_t *pSize);

//------------------------------------------------------------------------
/**
\brief     初始化设备库。
\attention 调用其他接口（除了GXGetLastError和GXCloseLib）之前必须先调用此接口，当用户不再使用库的时候调用GXCloseLib释放库资源。
           如果之前用户已经调用过GXInitLib，没有调用GXCloseLib，而再次调用GXInitLib接口，接口返回成功。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_FOUND_TL        找不到TL库
		   其它错误情况请参见GX_STATUS_LIST
		           
*/
//------------------------------------------------------------------------
GX_API GXInitLib();

//----------------------------------------------------------------------------------
/**
\brief     关闭设备库，释放资源
\attention 释放库资源，当用户不再使用库的时候调用此接口。
           如果用户之前没有调用GXInitLib，直接调用GXCloseLib，接口返回成功。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           其它错误情况请参见GX_STATUS_LIST   
*/
//----------------------------------------------------------------------------------
GX_API GXCloseLib();


//----------------------------------------------------------------------------------
/**
\brief     枚举所有设备并且获取设备个数,对于千兆网设备此接口仅能枚举同网段设备
\attention 此接口的作用是更新库内部设备列表，此接口会改变库内部设备列表，
           所以调用GXGetAllDeviceBaseInfo和GXOpenDevice之前需要调用此接口。
           如果在用户指定超时时间内成功枚举到设备，则立即返回；如果在用户指定超时时间内没有枚举到设备，则一直等待，直到达到用户指定的超时时间返回
\param     [out]punNumDevices 返回设备个数
\param     [in]unTimeOut      枚举的超时时间(单位ms)。
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
           GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
           其它错误情况请参见GX_STATUS_LIST
*/
//----------------------------------------------------------------------------------
GX_API GXUpdateDeviceList         (uint32_t* punNumDevices, uint32_t nTimeOut);


//----------------------------------------------------------------------------------
/**
\brief 通过序号打开设备
\param nDeviceIndex 设备序号，从1开始，例如：1、2、3、4...
\param phDevice 返回设备句柄
\return GX_STATUS,捕获底层调用产生的异常，根据异常类型返回不同的错误码
*/
//----------------------------------------------------------------------------------
GX_API GXOpenDeviceByIndex        (uint32_t nDeviceIndex, GX_DEV_HANDLE* phDevice);   // 已弃用


//----------------------------------------------------------------------------------
/**
\brief     指定设备句柄关闭设备
\attention 不能重复关闭同一个设备
\param     [in]hDevice 即将要关闭的设备句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄，或者关闭已经被关闭的设备
		   其它错误情况请参见GX_STATUS_LIST  
*/
//----------------------------------------------------------------------------------
GX_API GXCloseDevice              (GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\brief      获取Int类型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice 设备句柄
\param [in]featureID 功能码ID
\param [out]pnValue 用来返回当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_INVALID_PARAMETER   用户输入的指针为NULL
		   GX_STATUS_INVALID_ACCESS      当前不可访问
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXGetInt				      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t* pnValue);

//----------------------------------------------------------------------------------
/**
\brief      设置Int类型值的当前值
\attention  如果当前不可访问，调用此接口会返回错误GX_STATUS_INVALID_ACCESS
\param [in]hDevice   设备句柄
\param [in]featureID 功能码ID
\param [in]pnValue   用户设置的当前值
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_NOT_IMPLEMENTED     当前不支持的功能
		   GX_STATUS_ERROR_TYPE          用户传入的featureID类型错误
		   GX_STATUS_OUT_OF_RANGE        用户传入值越界
		   GX_STATUS_INVALID_ACCESS      当前不可访问
其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GXSetInt				      (GX_DEV_HANDLE hDevice, GX_FEATURE_ID_CMD featureID, int64_t nValue);

//----------------------------------------------------------------------------------
/**
\brief      用户设置采集buffer个数
\param [in]hDevice         设备句柄
\param [in]nBufferNum      用户设置的buffer个数
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			GX_STATUS_INVALID_PARAMETER    输入参数无效
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXSetAcqusitionBufferNumber(GX_DEV_HANDLE hDevice, uint64_t nBufferNum);


//----------------------------------------------------------------------------------
/**
\brief      开采
\param [in]hDevice     设备句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GxStreamOn  (GX_DEV_HANDLE hDevice);

//----------------------------------------------------------------------------------
/**
\brief      获取一幅图像，在图像处理完成后，需要调用GxQBuf接口，否则采集将无法继续
\param [in]  hDevice        设备句柄
\param [out] ppFrameData   图像数据，如果函数返回失败，图像数据是无效的
\param [in]  nTimeOut      超时时间(单位ms)
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GxDQBuf  (GX_DEV_HANDLE hDevice, GX_FRAME_DATA **ppFrameData, uint32_t nTimeOut);

//----------------------------------------------------------------------------------
/**
\brief      将图像数据Buf放回GxIAPI库，继续用于采集
\param [in]  hDevice        设备句柄
\param [out] pFrameData     图像数据Buf
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GxQBuf  (GX_DEV_HANDLE hDevice,GX_FRAME_DATA *pFrameData);

//----------------------------------------------------------------------------------
/**
\brief      停采
\param [in]hDevice     设备句柄
\return    GX_STATUS_SUCCESS             操作成功，没有发生错误
           GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
		   GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
		   GX_STATUS_INVALID_PARAMETER   用户传入指针为NULL
		   其它错误情况请参见GX_STATUS_LIST 
*/
//----------------------------------------------------------------------------------
GX_API GxStreamOff  (GX_DEV_HANDLE hDevice);


//----------------------------------------------------------------------------------
/**
\brief      读用户指定寄存器的值
\param [in]hDevice         设备句柄
\param [in]ui64Address     寄存器地址
\param [out]pBuffer        返回寄存器的值，不能为NULL
\param [in, out]piSize     [in]用户申请的Buffer大小
                           [out]成功读取寄存器的值后，返回实际大小
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXReadRemoteDevicePort(GX_DEV_HANDLE hDevice, uint64_t ui64Address, void *pBuffer, size_t *piSize);


//----------------------------------------------------------------------------------
/**
\brief      向用户指定的寄存器中写入用户给定的数据
\param [in]hDevice         设备句柄
\param [in]ui64Address     寄存器地址
\param [in]pBuffer         返回寄存器的值，不能为NULL
\param [in, out]piSize     [in]用户要写入的Buffer长度
                           [out]返回实际写入寄存器的长度
\return     GX_STATUS_SUCCESS             操作成功，没有发生错误
			GX_STATUS_NOT_INIT_API        没有调用GXInitLib初始化库
			GX_STATUS_INVALID_HANDLE      用户传入非法的句柄
			上面没有涵盖到的，不常见的错误情况请参见GX_STATUS_LIST

*/
//----------------------------------------------------------------------------------
GX_API GXWriteRemoteDevicePort(GX_DEV_HANDLE hDevice, uint64_t ui64Address, const void *pBuffer, size_t *piSize);

#endif  //GX_GALAXY_H
