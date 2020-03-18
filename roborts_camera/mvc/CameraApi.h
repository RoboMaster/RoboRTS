#ifndef _MVCAMAPI_H_
#define _MVCAMAPI_H_

#define MVSDK_API

#ifdef __cplusplus
 extern "C" {
 #endif


#include "CameraDefine.h"
#include "CameraStatus.h"



/******************************************************/
// 函数名   : CameraSdkInit
// 功能描述 : 相机SDK初始化，在调用任何SDK其他接口前，必须
//        先调用该接口进行初始化。该函数在整个进程运行
//        期间只需要调用一次。
// 参数     : iLanguageSel 用于选择SDK内部提示信息和界面的语种,
//               0:表示英文,1:表示中文。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus     CameraSdkInit(int     iLanguageSel);

/******************************************************/
// 函数名   : CameraSetDataDirectory
// 功能描述 : 设置相机数据文件的存储目录（.config .mvdat等）
//            需要在CameraInit打开相机前设置好
//            默认目录为当前目录
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraSetDataDirectory(char const* dirname);

/******************************************************/
// 函数名   : CameraUSBDeviceInit
// 功能描述 : (已废弃，无需调用)
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraUSBDeviceInit();

/******************************************************/
// 函数名   : CameraUSBDeviceUninit
// 功能描述 : (已废弃，无需调用)
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraUSBDeviceUninit();


/******************************************************/
// 函数名   : CameraEnumerateDevice
// 功能描述 : 枚举设备，并建立设备列表。在调用CameraInit
//        之前，必须调用该函数来获得设备的信息。
// 参数     : pCameraList    设备列表数组指针。
//             piNums        设备的个数指针，调用时传入pCameraList
//                            数组的元素个数，函数返回时，保存实际找到的设备个数。
//              注意，piNums指向的值必须初始化，且不超过pCameraList数组元素个数，
//              否则有可能造成内存溢出。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraEnumerateDevice(
    tSdkCameraDevInfo* pCameraList,
    INT*               piNums
);

/******************************************************/
// 函数名   : CameraIdleStateDevice
// 功能描述 : 当前系统有未使用的相机信息。
// 参数     : pCameraList    设备列表数组指针。
//             piNums        设备的个数指针，调用时传入pCameraList
//                            数组的元素个数，函数返回时，保存实际找到的设备个数。
//              注意，piNums指向的值必须初始化，且不超过pCameraList数组元素个数，
//              否则有可能造成内存溢出。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraIdleStateDevice(
    tSdkCameraDevInfo* pCameraList,
    INT*               piNums
);


/******************************************************/
// 函数名 	: CameraEnumerateDeviceEx
// 功能描述	: 枚举设备，并建立设备列表。在调用CameraInitEx
//			  之前，必须调用该函数枚举设备。
// 参数	     :
// 返回值     : 返回设备个数，0表示无。
/******************************************************/
MVSDK_API INT  CameraEnumerateDeviceEx(
);


/******************************************************/
// 函数名   : CameraIsOpened
// 功能描述 : 检测设备是否已经被其他应用程序打开。在调用CameraInit
//        之前，可以使用该函数进行检测，如果已经被打开，调用
//        CameraInit会返回设备已经被打开的错误码。
// 参数     : pCameraList 设备的枚举信息结构体指针，由CameraEnumerateDevice获得。
//            pOpened       设备的状态指针，返回设备是否被打开的状态，TRUE为打开，FALSE为空闲。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraIsOpened(
  tSdkCameraDevInfo*  pCameraList,
  BOOL*               pOpened
);


/******************************************************/
// 函数名   : CameraInit
// 功能描述 : 相机初始化。初始化成功后，才能调用任何其他
//        相机相关的操作接口。
// 参数     : pCameraInfo    该相机的设备描述信息，由CameraEnumerateDevice
//               函数获得。
//            iParamLoadMode  相机初始化时使用的参数加载方式。-1表示使用上次退出时的参数加载方式。
//            emTeam         初始化时使用的参数组。-1表示加载上次退出时的参数组。
//            pCameraHandle  相机的句柄指针，初始化成功后，该指针
//               返回该相机的有效句柄，在调用其他相机
//               相关的操作接口时，都需要传入该句柄，主要
//               用于多相机之间的区分。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraInit(
    tSdkCameraDevInfo*  pCameraInfo,
    int                 emParamLoadMode,
    int                 emTeam,
    CameraHandle*       pCameraHandle
);

/******************************************************/
// 函数名 	: CameraInitEx
// 功能描述	: 相机初始化。初始化成功后，才能调用任何其他
//			  相机相关的操作接口。
// 参数	    : iDeviceIndex    相机的索引号，CameraEnumerateDeviceEx返回相机个数。
//            iParamLoadMode  相机初始化时使用的参数加载方式。-1表示使用上次退出时的参数加载方式。
//            emTeam         初始化时使用的参数组。-1表示加载上次退出时的参数组。
//            pCameraHandle  相机的句柄指针，初始化成功后，该指针
//							 返回该相机的有效句柄，在调用其他相机
//							 相关的操作接口时，都需要传入该句柄，主要
//							 用于多相机之间的区分。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraInitEx(
    int             iDeviceIndex,
    int             iParamLoadMode,
    int             emTeam,
    CameraHandle*   pCameraHandle
);

/// @ingroup API_OPEN
/// \~chinese
/// \brief 相机初始化。初始化成功后，才能调用其他相机相关的操作接口。
/// \param [in] CameraName 相机昵称。@link #tSdkCameraDevInfo.acFriendlyName @endlink
/// \param [out] pCameraHandle 相机的句柄指针，初始化成功后，该指针返回该相机的有效句柄。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief The camera is initialized. After successful initialization, other camera-related operation interfaces can be called.
/// \param [in] CameraName Camera friendly name.@link #tSdkCameraDevInfo.acFriendlyName @endlink
/// \param [out] pCameraHandle The handle pointer of the camera, after successful initialization, returns the camera's effective handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraInitEx2(
	char* CameraName,
	CameraHandle   *pCameraHandle
);

/******************************************************/
// 函数名   : CameraSetCallbackFunction
// 功能描述 : 设置图像捕获的回调函数。当捕获到新的图像数据帧时，
//        pCallBack所指向的回调函数就会被调用。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            pCallBack 回调函数指针。
//            pContext  回调函数的附加参数，在回调函数被调用时
//            该附加参数会被传入，可以为NULL。多用于
//            多个相机时携带附加信息。
//            pCallbackOld  用于保存当前的回调函数。可以为NULL。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetCallbackFunction(
    CameraHandle        hCamera,
    CAMERA_SNAP_PROC    pCallBack,
    PVOID               pContext,
    CAMERA_SNAP_PROC*   pCallbackOld
);

/******************************************************/
// 函数名   : CameraUnInit
// 功能描述 : 相机反初始化。释放资源。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraUnInit(
    CameraHandle hCamera
);

/******************************************************/
// 函数名   : CameraGetInformation
// 功能描述 : 获得相机的描述信息
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            pbuffer 指向相机描述信息指针的指针。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetInformation(
    CameraHandle    hCamera,
    char**          pbuffer
);

/******************************************************/
// 函数名   : CameraImageProcess
// 功能描述 : 将获得的相机原始输出图像数据进行处理，叠加饱和度、
//        颜色增益和校正、降噪等处理效果，最后得到RGB888
//        格式的图像数据。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbyIn    输入图像数据的缓冲区地址，不能为NULL。
//            pbyOut   处理后图像输出的缓冲区地址，不能为NULL。
//            pFrInfo  输入图像的帧头信息，处理完成后，帧头信息
//             中的图像格式uiMediaType会随之改变。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraImageProcess(
    CameraHandle        hCamera,
    BYTE*               pbyIn,
    BYTE*               pbyOut,
    tSdkFrameHead*      pFrInfo
);

/******************************************************/
// 函数名 	: CameraImageProcessEx
// 功能描述	: 将获得的相机原始输出图像数据进行处理，叠加饱和度
//			  颜色增益和校正、降噪等处理效果，最后得到RGB888
//			  格式的图像数据。
// 参数	    : hCamera      相机的句柄，由CameraInit函数获得。
//            pbyIn	     输入图像数据的缓冲区地址，不能为NULL。
//            pbyOut        处理后图像输出的缓冲区地址，不能为NULL。
//            pFrInfo       输入图像的帧头信息，处理完成后，帧头信息
//            uOutFormat    处理完后图像的输出格式
//            uReserved     预留参数，必须设置为0
//					   中的图像格式uiMediaType会随之改变。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraImageProcessEx(
    CameraHandle hCamera,
    BYTE *pbyIn,
    BYTE *pbyOut,
    tSdkFrameHead *pFrInfo,
    UINT uOutFormat,
    UINT uReserved
);

/******************************************************/
// 函数名   : CameraDisplayInit
// 功能描述 : 初始化SDK内部的显示模块。在调用CameraDisplayRGB24
//        前必须先调用该函数初始化。如果您在二次开发中，
//        使用自己的方式进行图像显示(不调用CameraDisplayRGB24)，
//        则不需要调用本函数。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            hWndDisplay 显示窗口的句柄，一般为窗口的m_hWnd成员。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraDisplayInit(
    CameraHandle    hCamera,
    HWND            hWndDisplay
);

/******************************************************/
// 函数名   : CameraDisplayRGB24
// 功能描述 : 显示图像。必须调用过CameraDisplayInit进行
//        初始化才能调用本函数。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbyRGB24 图像的数据缓冲区，RGB888格式。
//            pFrInfo  图像的帧头信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraDisplayRGB24(
    CameraHandle        hCamera,
    BYTE*               pbyRGB24,
    tSdkFrameHead*      pFrInfo
);

/******************************************************/
// 函数名   : CameraSetDisplayMode
// 功能描述 : 设置显示的模式。必须调用过CameraDisplayInit
//        进行初始化才能调用本函数。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iMode    显示模式，DISPLAYMODE_SCALE或者
//             DISPLAYMODE_REAL,具体参见CameraDefine.h
//             中emSdkDisplayMode的定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetDisplayMode(
    CameraHandle    hCamera,
    INT             iMode
);

/******************************************************/
// 函数名   : CameraSetDisplayOffset
// 功能描述 : 设置显示的起始偏移值。仅当显示模式为DISPLAYMODE_REAL
//        时有效。例如显示控件的大小为320X240，而图像的
//        的尺寸为640X480，那么当iOffsetX = 160,iOffsetY = 120时
//        显示的区域就是图像的居中320X240的位置。必须调用过
//        CameraDisplayInit进行初始化才能调用本函数。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iOffsetX  偏移的X坐标。
//            iOffsetY  偏移的Y坐标。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetDisplayOffset(
    CameraHandle    hCamera,
    int             iOffsetX,
    int             iOffsetY
);

/******************************************************/
// 函数名   : CameraSetDisplaySize
// 功能描述 : 设置显示控件的尺寸。必须调用过
//        CameraDisplayInit进行初始化才能调用本函数。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iWidth    宽度
//            iHeight   高度
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetDisplaySize(
    CameraHandle    hCamera,
    INT             iWidth,
    INT             iHeight
);

/******************************************************/
// 函数名   : CameraGetImageBuffer
// 功能描述 : 获得一帧图像数据。为了提高效率，SDK在图像抓取时采用了零拷贝机制，
//        CameraGetImageBuffer实际获得是内核中的一个缓冲区地址，
//        该函数成功调用后，必须调用CameraReleaseImageBuffer释放由
//        CameraGetImageBuffer得到的缓冲区,以便让内核继续使用
//        该缓冲区。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            pFrameInfo  图像的帧头信息指针。
//            pbyBuffer   指向图像的数据的缓冲区指针。由于
//              采用了零拷贝机制来提高效率，因此
//              这里使用了一个指向指针的指针。
//            UINT wTimes 抓取图像的超时时间。单位毫秒。在
//              wTimes时间内还未获得图像，则该函数
//              会返回超时信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetImageBuffer(
    CameraHandle        hCamera,
    tSdkFrameHead*      pFrameInfo,
    BYTE**              pbyBuffer,
    UINT                wTimes
);

/******************************************************/
// 函数名 	: CameraGetImageBufferEx
// 功能描述	: 获得一帧图像数据。该接口获得的图像是经过处理后的RGB格式。该函数调用后，
//			  不需要调用 CameraReleaseImageBuffer 释放，也不要调用free之类的函数释放
//              来释放该函数返回的图像数据缓冲区。
// 参数	    : hCamera	  相机的句柄，由CameraInit函数获得。
//            piWidth    整形指针，返回图像的宽度
//            piHeight   整形指针，返回图像的高度
//            UINT wTimes 抓取图像的超时时间。单位毫秒。在
//						  wTimes时间内还未获得图像，则该函数
//						  会返回超时信息。
// 返回值   : 成功时，返回RGB数据缓冲区的首地址;
//            否则返回0。
/******************************************************/
MVSDK_API unsigned char*  CameraGetImageBufferEx(
    CameraHandle        hCamera,
    INT*                piWidth,
    INT*                piHeight,
    UINT                wTimes
);


/******************************************************/
// 函数名   : CameraSnapToBuffer
// 功能描述 : 抓拍一张图像到缓冲区中。相机会进入抓拍模式，并且
//        自动切换到抓拍模式的分辨率进行图像捕获。然后将
//        捕获到的数据保存到缓冲区中。
//        该函数成功调用后，必须调用CameraReleaseImageBuffer
//        释放由CameraSnapToBuffer得到的缓冲区。具体请参考
//        CameraGetImageBuffer函数的功能描述部分。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            pFrameInfo  指针，返回图像的帧头信息。
//            pbyBuffer   指向指针的指针，用来返回图像缓冲区的地址。
//            uWaitTimeMs 超时时间，单位毫秒。在该时间内，如果仍然没有
//              成功捕获的数据，则返回超时信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSnapToBuffer(
    CameraHandle        hCamera,
    tSdkFrameHead*      pFrameInfo,
    BYTE**              pbyBuffer,
    UINT                uWaitTimeMs
);

/******************************************************/
// 函数名   : CameraReleaseImageBuffer
// 功能描述 : 释放由CameraGetImageBuffer获得的缓冲区。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            pbyBuffer   由CameraGetImageBuffer获得的缓冲区地址。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraReleaseImageBuffer(
    CameraHandle    hCamera,
    BYTE*           pbyBuffer
);

/******************************************************/
// 函数名   : CameraPlay
// 功能描述 : 让SDK进入工作模式，开始接收来自相机发送的图像
//        数据。如果当前相机是触发模式，则需要接收到
//        触发帧以后才会更新图像。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraPlay(
    CameraHandle hCamera
);

/******************************************************/
// 函数名   : CameraPause
// 功能描述 : 让SDK进入暂停模式，不接收来自相机的图像数据，
//        同时也会发送命令让相机暂停输出，释放传输带宽。
//        暂停模式下，可以对相机的参数进行配置，并立即生效。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraPause(
    CameraHandle hCamera
);

/******************************************************/
// 函数名   : CameraStop
// 功能描述 : 让SDK进入停止状态，一般是反初始化时调用该函数，
//        该函数被调用，不能再对相机的参数进行配置。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraStop(
    CameraHandle hCamera
);

/******************************************************/
// 函数名   : CameraInitRecord
// 功能描述 : 初始化一次录像。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iFormat   录像的格式，当前只支持不压缩和MSCV两种方式。
//              0:不压缩；1:MSCV方式压缩。
//            pcSavePath  录像文件保存的路径。
//            b2GLimit    如果为TRUE,则文件大于2G时自动分割。
//            dwQuality   录像的质量因子，越大，则质量越好。范围1到100.
//            iFrameRate  录像的帧率。建议设定的比实际采集帧率大，
//              这样就不会漏帧。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraInitRecord(
    CameraHandle    hCamera,
    int             iFormat,
    char*           pcSavePath,
    BOOL            b2GLimit,
    DWORD           dwQuality,
    int             iFrameRate
);

/******************************************************/
// 函数名   : CameraStopRecord
// 功能描述 : 结束本次录像。当CameraInitRecord后，可以通过该函数
//        来结束一次录像，并完成文件保存操作。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraStopRecord(
    CameraHandle    hCamera
);

/******************************************************/
// 函数名   : CameraPushFrame
// 功能描述 : 将一帧数据存入录像流中。必须调用CameraInitRecord
//        才能调用该函数。CameraStopRecord调用后，不能再调用
//        该函数。由于我们的帧头信息中携带了图像采集的时间戳
//        信息，因此录像可以精准的时间同步，而不受帧率不稳定
//        的影响。
// 参数     : hCamera     相机的句柄，由CameraInit函数获得。
//            pbyImageBuffer    图像的数据缓冲区，必须是RGB格式。
//            pFrInfo           图像的帧头信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraPushFrame(
    CameraHandle    hCamera,
    BYTE*           pbyImageBuffer,
    tSdkFrameHead*  pFrInfo
);

/******************************************************/
// 函数名   : CameraSaveImage
// 功能描述 : 将图像缓冲区的数据保存成图片文件。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            lpszFileName   图片保存文件完整路径。
//            pbyImageBuffer 图像的数据缓冲区。
//            pFrInfo        图像的帧头信息。
//            byFileType     图像保存的格式。取值范围参见CameraDefine.h
//               中emSdkFileType的类型定义。目前支持
//               BMP、JPG、PNG、RAW四种格式。其中RAW表示
//               相机输出的原始数据，保存RAW格式文件要求
//               pbyImageBuffer和pFrInfo是由CameraGetImageBuffer
//               获得的数据，而且未经CameraImageProcess转换
//               成BMP格式；反之，如果要保存成BMP、JPG或者
//               PNG格式，则pbyImageBuffer和pFrInfo是由
//               CameraImageProcess处理后的RGB格式数据。
//                 具体用法可以参考Advanced的例程。
//            byQuality      图像保存的质量因子，仅当保存为JPG格式
//                 时该参数有效，范围1到100。其余格式
//                           可以写成0。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSaveImage(
    CameraHandle    hCamera,
    char*           lpszFileName,
    BYTE*           pbyImageBuffer,
    tSdkFrameHead*  pFrInfo,
    BYTE            byFileType,
    BYTE            byQuality
);

/******************************************************/
// 函数名   : CameraGetImageResolution
// 功能描述 : 获得当前预览的分辨率。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            psCurVideoSize 结构体指针，用于返回当前的分辨率。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetImageResolution(
    CameraHandle            hCamera,
    tSdkImageResolution*    psCurVideoSize
);

/******************************************************/
// 函数名   : CameraSetImageResolution
// 功能描述 : 设置预览的分辨率。
// 参数     : hCamera      相机的句柄，由CameraInit函数获得。
//            pImageResolution 结构体指针，用于返回当前的分辨率。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetImageResolution(
    CameraHandle            hCamera,
    tSdkImageResolution*    pImageResolution
);

/******************************************************/
// 函数名   : CameraGetMediaType
// 功能描述 : 获得相机当前输出原始数据的格式索引号。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            piMediaType   指针，用于返回当前格式类型的索引号。
//              由CameraGetCapability获得相机的属性，
//              在tSdkCameraCapbility结构体中的pMediaTypeDesc
//              成员中，以数组的形式保存了相机支持的格式，
//              piMediaType所指向的索引号，就是该数组的索引号。
//              pMediaTypeDesc[*piMediaType].iMediaType则表示当前格式的
//              编码。该编码请参见CameraDefine.h中[图像格式定义]部分。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetMediaType(
    CameraHandle    hCamera,
    INT*            piMediaType
);

/******************************************************/
// 函数名   : CameraSetMediaType
// 功能描述 : 设置相机的输出原始数据格式。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iMediaType  由CameraGetCapability获得相机的属性，
//              在tSdkCameraCapbility结构体中的pMediaTypeDesc
//              成员中，以数组的形式保存了相机支持的格式，
//              iMediaType就是该数组的索引号。
//              pMediaTypeDesc[iMediaType].iMediaType则表示当前格式的
//              编码。该编码请参见CameraDefine.h中[图像格式定义]部分。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetMediaType(
    CameraHandle    hCamera,
    INT             iMediaType
);

/******************************************************/
// 函数名   : CameraSetAeState
// 功能描述 : 设置相机曝光的模式。自动或者手动。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            bAeState    TRUE，使能自动曝光；FALSE，停止自动曝光。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetAeState(
    CameraHandle    hCamera,
    BOOL            bAeState
);

/******************************************************/
// 函数名   : CameraGetAeState
// 功能描述 : 获得相机当前的曝光模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pAeState   指针，用于返回自动曝光的使能状态。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetAeState(
    CameraHandle    hCamera,
    BOOL*           pAeState
);

/******************************************************/
// 函数名   : CameraSetSharpness
// 功能描述 : 设置图像的处理的锐化参数。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iSharpness 锐化参数。范围由CameraGetCapability
//               获得，一般是[0,100]，0表示关闭锐化处理。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetSharpness(
    CameraHandle    hCamera,
    int             iSharpness
);

/******************************************************/
// 函数名   : CameraGetSharpness
// 功能描述 : 获取当前锐化设定值。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            piSharpness 指针，返回当前设定的锐化的设定值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetSharpness(
    CameraHandle    hCamera,
    int*            piSharpness
);

/******************************************************/
// 函数名   : CameraSetLutMode
// 功能描述 : 设置相机的查表变换模式LUT模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            emLutMode  LUTMODE_PARAM_GEN 表示由伽马和对比度参数动态生成LUT表。
//             LUTMODE_PRESET    表示使用预设的LUT表。
//             LUTMODE_USER_DEF  表示使用用户自定的LUT表。
//             LUTMODE_PARAM_GEN的定义参考CameraDefine.h中emSdkLutMode类型。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetLutMode(
    CameraHandle    hCamera,
    int             emLutMode
);

/******************************************************/
// 函数名   : CameraGetLutMode
// 功能描述 : 获得相机的查表变换模式LUT模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pemLutMode 指针，返回当前LUT模式。意义与CameraSetLutMode
//             中emLutMode参数相同。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetLutMode(
    CameraHandle    hCamera,
    int*            pemLutMode
);

/******************************************************/
// 函数名   : CameraSelectLutPreset
// 功能描述 : 选择预设LUT模式下的LUT表。必须先使用CameraSetLutMode
//        将LUT模式设置为预设模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iSel     表的索引号。表的个数由CameraGetCapability
//             获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSelectLutPreset(
    CameraHandle    hCamera,
    int             iSel
);

/******************************************************/
// 函数名   : CameraGetLutPresetSel
// 功能描述 : 获得预设LUT模式下的LUT表索引号。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piSel      指针，返回表的索引号。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetLutPresetSel(
    CameraHandle    hCamera,
    int*            piSel
);

/******************************************************/
// 函数名   : CameraSetCustomLut
// 功能描述 : 设置自定义的LUT表。必须先使用CameraSetLutMode
//        将LUT模式设置为自定义模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//             iChannel 指定要设定的LUT颜色通道，当为LUT_CHANNEL_ALL时，
//                      三个通道的LUT将被同时替换。
//                      参考CameraDefine.h中emSdkLutChannel定义。
//            pLut     指针，指向LUT表的地址。LUT表为无符号短整形数组，数组大小为
//           4096，分别代码颜色通道从0到4096(12bit颜色精度)对应的映射值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetCustomLut(
    CameraHandle    hCamera,
    int       iChannel,
    USHORT*         pLut
);

/******************************************************/
// 函数名   : CameraGetCustomLut
// 功能描述 : 获得当前使用的自定义LUT表。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//             iChannel 指定要获得的LUT颜色通道。当为LUT_CHANNEL_ALL时，
//                      返回红色通道的LUT表。
//                      参考CameraDefine.h中emSdkLutChannel定义。
//            pLut       指针，指向LUT表的地址。LUT表为无符号短整形数组，数组大小为
//           4096，分别代码颜色通道从0到4096(12bit颜色精度)对应的映射值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetCustomLut(
    CameraHandle    hCamera,
    int       iChannel,
    USHORT*         pLut
);

/******************************************************/
// 函数名   : CameraGetCurrentLut
// 功能描述 : 获得相机当前的LUT表，在任何LUT模式下都可以调用,
//        用来直观的观察LUT曲线的变化。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//             iChannel 指定要获得的LUT颜色通道。当为LUT_CHANNEL_ALL时，
//                      返回红色通道的LUT表。
//                      参考CameraDefine.h中emSdkLutChannel定义。
//            pLut       指针，指向LUT表的地址。LUT表为无符号短整形数组，数组大小为
//           4096，分别代码颜色通道从0到4096(12bit颜色精度)对应的映射值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetCurrentLut(
    CameraHandle    hCamera,
    int       iChannel,
    USHORT*         pLut
);

/******************************************************/
// 函数名   : CameraSetWbMode
// 功能描述 : 设置相机白平衡模式。分为手动和自动两种方式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            bAuto      TRUE，则表示使能自动模式。
//             FALSE，则表示使用手动模式，通过调用
//                 CameraSetOnceWB来进行一次白平衡。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetWbMode(
    CameraHandle    hCamera,
    BOOL            bAuto
);

/******************************************************/
// 函数名   : CameraGetWbMode
// 功能描述 : 获得当前的白平衡模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbAuto   指针，返回TRUE表示自动模式，FALSE
//             为手动模式。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetWbMode(
    CameraHandle    hCamera,
    BOOL*           pbAuto
);

/******************************************************/
// 函数名   : CameraSetPresetClrTemp
// 功能描述 : 选择指定预设色温模式
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iSel 预设色温的模式索引号，从0开始
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetPresetClrTemp(
    CameraHandle    hCamera,
    int             iSel
);

/******************************************************/
// 函数名   : CameraGetPresetClrTemp
// 功能描述 : 获得当前选择的预设色温模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piSel  指针，返回选择的预设色温索引号
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetPresetClrTemp(
    CameraHandle    hCamera,
    int*            piSel
);

/******************************************************/
// 函数名   : CameraSetUserClrTempGain
// 功能描述 : 设置自定义色温模式下的数字增益
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iRgain  红色增益，范围0到400，表示0到4倍
//            iGgain  绿色增益，范围0到400，表示0到4倍
//            iBgain  蓝色增益，范围0到400，表示0到4倍
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetUserClrTempGain(
  CameraHandle  hCamera,
  int       iRgain,
  int       iGgain,
  int       iBgain
);


/******************************************************/
// 函数名   : CameraGetUserClrTempGain
// 功能描述 : 获得自定义色温模式下的数字增益
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piRgain  指针，返回红色增益，范围0到400，表示0到4倍
//            piGgain  指针，返回绿色增益，范围0到400，表示0到4倍
//            piBgain  指针，返回蓝色增益，范围0到400，表示0到4倍
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetUserClrTempGain(
  CameraHandle  hCamera,
  int*      piRgain,
  int*      piGgain,
  int*      piBgain
);

/******************************************************/
// 函数名   : CameraSetUserClrTempMatrix
// 功能描述 : 设置自定义色温模式下的颜色矩阵
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pMatrix 指向一个float[3][3]数组的首地址
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetUserClrTempMatrix(
  CameraHandle  hCamera,
  float*      pMatrix
);


/******************************************************/
// 函数名   : CameraGetUserClrTempMatrix
// 功能描述 : 获得自定义色温模式下的颜色矩阵
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pMatrix 指向一个float[3][3]数组的首地址
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetUserClrTempMatrix(
  CameraHandle  hCamera,
  float*      pMatrix
);

/******************************************************/
// 函数名   : CameraSetClrTempMode
// 功能描述 : 设置白平衡时使用的色温模式，
//              支持的模式有三种，分别是自动，预设和自定义。
//              自动模式下，会自动选择合适的色温模式
//              预设模式下，会使用用户指定的色温模式
//              自定义模式下，使用用户自定义的色温数字增益和矩阵
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iMode 模式，只能是emSdkClrTmpMode中定义的一种
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetClrTempMode(
  CameraHandle  hCamera,
  int       iMode
);

/******************************************************/
// 函数名   : CameraGetClrTempMode
// 功能描述 : 获得白平衡时使用的色温模式。参考CameraSetClrTempMode
//              中功能描述部分。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            pimode 指针，返回模式选择，参考emSdkClrTmpMode类型定义
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetClrTempMode(
  CameraHandle  hCamera,
  int*      pimode
);



/******************************************************/
// 函数名   : CameraSetOnceWB
// 功能描述 : 在手动白平衡模式下，调用该函数会进行一次白平衡。
//        生效的时间为接收到下一帧图像数据时。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetOnceWB(
    CameraHandle    hCamera
);

/******************************************************/
// 函数名   : CameraSetOnceBB
// 功能描述 : 执行一次黑平衡操作。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetOnceBB(
    CameraHandle    hCamera
);


/******************************************************/
// 函数名   : CameraSetAeTarget
// 功能描述 : 设定自动曝光的亮度目标值。设定范围由CameraGetCapability
//        函数获得。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iAeTarget  亮度目标值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetAeTarget(
    CameraHandle    hCamera,
    int             iAeTarget
);

/******************************************************/
// 函数名   : CameraGetAeTarget
// 功能描述 : 获得自动曝光的亮度目标值。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            *piAeTarget 指针，返回目标值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetAeTarget(
    CameraHandle    hCamera,
    int*            piAeTarget
);


/******************************************************/
// 函数名   : CameraSetAeExposureRange
// 功能描述 : 设定自动曝光模式的曝光时间调节范围
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//           fMinExposureTime 最小曝光时间（微秒）
//			 fMaxExposureTime 最大曝光时间（微秒）
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraSetAeExposureRange(
	CameraHandle    hCamera,
	double          fMinExposureTime,
	double			fMaxExposureTime
	);

/******************************************************/
// 函数名   : CameraGetAeExposureRange
// 功能描述 : 获得自动曝光模式的曝光时间调节范围
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//           fMinExposureTime 最小曝光时间（微秒）
//			 fMaxExposureTime 最大曝光时间（微秒）
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraGetAeExposureRange(
	CameraHandle    hCamera,
	double*         fMinExposureTime,
	double*			fMaxExposureTime
	);

/******************************************************/
// 函数名   : CameraSetAeAnalogGainRange
// 功能描述 : 设定自动曝光模式的增益调节范围
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//           iMinAnalogGain 最小增益
//			 iMaxAnalogGain 最大增益
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraSetAeAnalogGainRange(
	CameraHandle    hCamera,
	int				iMinAnalogGain,
	int				iMaxAnalogGain
	);

/******************************************************/
// 函数名   : CameraGetAeAnalogGainRange
// 功能描述 : 获得自动曝光模式的增益调节范围
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//           iMinAnalogGain 最小增益
//			 iMaxAnalogGain 最大增益
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraGetAeAnalogGainRange(
	CameraHandle    hCamera,
	int*			iMinAnalogGain,
	int*			iMaxAnalogGain
	);


/******************************************************/
// 函数名   : CameraSetExposureTime
// 功能描述 : 设置曝光时间。单位为微秒。对于CMOS传感器，其曝光
//        的单位是按照行来计算的，因此，曝光时间并不能在微秒
//        级别连续可调。而是会按照整行来取舍。在调用
//        本函数设定曝光时间后，建议再调用CameraGetExposureTime
//        来获得实际设定的值。
// 参数     : hCamera      相机的句柄，由CameraInit函数获得。
//            fExposureTime 曝光时间，单位微秒。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/

MVSDK_API CameraSdkStatus  CameraSetExposureTime(
    CameraHandle    hCamera,
    double          fExposureTime
);

/******************************************************/
// 函数名   : CameraGetExposureLineTime
// 功能描述 : 获得一行的曝光时间。对于CMOS传感器，其曝光
//        的单位是按照行来计算的，因此，曝光时间并不能在微秒
//        级别连续可调。而是会按照整行来取舍。这个函数的
//          作用就是返回CMOS相机曝光一行对应的时间。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pfLineTime 指针，返回一行的曝光时间，单位为微秒。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/

MVSDK_API CameraSdkStatus  CameraGetExposureLineTime(
    CameraHandle    hCamera,
    double*         pfLineTime
);

/******************************************************/
// 函数名   : CameraGetExposureTime
// 功能描述 : 获得相机的曝光时间。请参见CameraSetExposureTime
//        的功能描述。
// 参数     : hCamera        相机的句柄，由CameraInit函数获得。
//            pfExposureTime   指针，返回当前的曝光时间，单位微秒。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetExposureTime(
    CameraHandle    hCamera,
    double*         pfExposureTime
);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief 获得相机的曝光时间范围
/// \param [in] hCamera 相机的句柄。
/// \param [out] pfMin		指针，返回曝光时间的最小值，单位微秒。
/// \param [out] pfMax		指针，返回曝光时间的最大值，单位微秒。
/// \param [out] pfStep		指针，返回曝光时间的步进值，单位微秒。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get camera exposure time range
/// \param [in] hCamera Camera handle.
/// \param [out] pfMin Returns the minimum exposure time in microseconds.
/// \param [out] pfMax Returns the maximum exposure time in microseconds.
/// \param [out] pfStep Returns the exposure time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetExposureTimeRange(
	CameraHandle    hCamera, 
	double*         pfMin,
	double*			pfMax,
	double*			pfStep
	);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief 设置多重曝光时间。单位为微秒。(此功能仅线阵相机支持)
/// \param [in] hCamera 相机的句柄。
/// \param [in] index 曝光索引。
/// \param [in] fExposureTime 曝光时间，单位微秒。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \note 对于CMOS传感器，其曝光的单位是按照行来计算的，因此，曝光时间并不能在微秒级别连续可调。而是会按照整行来取舍。在调用本函数设定曝光时间后，建议再调用@link #CameraGetMultiExposureTime @endlink来获得实际设定的值。
/// \~english
/// \brief Set the multiple exposure time. The unit is microseconds. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [in] index Exposure index.
/// \param [in] fExposureTime Exposure time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note For CMOS sensors, the unit of exposure is calculated in rows, so the exposure time cannot be continuously adjusted in microseconds. Instead, the entire line will be chosen. After calling this function to set the exposure time, it is recommended to call @link #CameraGetMultiExposureTime @endlink to get the actual set value.
MVSDK_API CameraSdkStatus CameraSetMultiExposureTime(
	CameraHandle    hCamera, 
	int				index,
	double          fExposureTime
	);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief 获取多重曝光时间。单位为微秒。(此功能仅线阵相机支持)
/// \param [in] hCamera 相机的句柄。
/// \param [in] index 曝光索引。
/// \param [out] fExposureTime 返回曝光时间，单位微秒。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the multiple exposure time. The unit is microseconds. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [in] index Exposure index.
/// \param [out] fExposureTime Returns exposure time in microseconds.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetMultiExposureTime(
	CameraHandle    hCamera, 
	int				index,
	double*         fExposureTime
	);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief 设置多重曝光使能个数。(此功能仅线阵相机支持)
/// \param [in] hCamera 相机的句柄。
/// \param [in] count 使能个数。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the number of multiple exposure enable. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [in] count The number of exposures enabled.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetMultiExposureCount(
	CameraHandle    hCamera, 
	int				count
	);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief 获取多重曝光使能个数。(此功能仅线阵相机支持)
/// \param [in] hCamera 相机的句柄。
/// \param [out] count 使能个数。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the number of multiple exposure enable. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [out] count The number of exposures enabled.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetMultiExposureCount(
	CameraHandle    hCamera, 
	int*			count
	);

/// @ingroup API_EXPOSURE
/// \~chinese
/// \brief 获取多重曝光的最大曝光个数。(此功能仅线阵相机支持)
/// \param [in] hCamera 相机的句柄。
/// \param [out] max_count 支持的最大曝光个数。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the maximum number of exposures for multiple exposures. (This feature is only supported by line camera)
/// \param [in] hCamera Camera handle.
/// \param [out] max_count The maximum number of exposures supported.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetMultiExposureMaxCount(
	CameraHandle    hCamera, 
	int*			max_count
	);

/******************************************************/
// 函数名   : CameraSetAnalogGain
// 功能描述 : 设置相机的图像模拟增益值。该值乘以CameraGetCapability获得
//        的相机属性结构体中sExposeDesc.fAnalogGainStep，就
//        得到实际的图像信号放大倍数。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iAnalogGain 设定的模拟增益值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetAnalogGain(
    CameraHandle    hCamera,
    INT             iAnalogGain
);

/******************************************************/
// 函数名   : CameraGetAnalogGain
// 功能描述 : 获得图像信号的模拟增益值。参见CameraSetAnalogGain
//        详细说明。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            piAnalogGain 指针，返回当前的模拟增益值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetAnalogGain(
    CameraHandle    hCamera,
    INT*            piAnalogGain
);

/******************************************************/
// 函数名   : CameraSetGain
// 功能描述 : 设置图像的数字增益。设定范围由CameraGetCapability
//        获得的相机属性结构体中sRgbGainRange成员表述。
//        实际的放大倍数是设定值/100。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iRGain   红色通道的增益值。
//            iGGain   绿色通道的增益值。
//            iBGain   蓝色通道的增益值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetGain(
    CameraHandle    hCamera,
    int             iRGain,
    int             iGGain,
    int             iBGain
);


/******************************************************/
// 函数名   : CameraGetGain
// 功能描述 : 获得图像处理的数字增益。具体请参见CameraSetGain
//        的功能描述部分。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piRGain  指针，返回红色通道的数字增益值。
//            piGGain    指针，返回绿色通道的数字增益值。
//            piBGain    指针，返回蓝色通道的数字增益值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetGain(
    CameraHandle    hCamera,
    int*            piRGain,
    int*            piGGain,
    int*            piBGain
);


/******************************************************/
// 函数名   : CameraSetGamma
// 功能描述 : 设定LUT动态生成模式下的Gamma值。设定的值会
//        马上保存在SDK内部，但是只有当相机处于动态
//        参数生成的LUT模式时，才会生效。请参考CameraSetLutMode
//        的函数说明部分。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iGamma     要设定的Gamma值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetGamma(
    CameraHandle    hCamera,
    int             iGamma
);

/******************************************************/
// 函数名   : CameraGetGamma
// 功能描述 : 获得LUT动态生成模式下的Gamma值。请参考CameraSetGamma
//        函数的功能描述。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piGamma    指针，返回当前的Gamma值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetGamma(
    CameraHandle    hCamera,
    int*            piGamma
);

/******************************************************/
// 函数名   : CameraSetContrast
// 功能描述 : 设定LUT动态生成模式下的对比度值。设定的值会
//        马上保存在SDK内部，但是只有当相机处于动态
//        参数生成的LUT模式时，才会生效。请参考CameraSetLutMode
//        的函数说明部分。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iContrast  设定的对比度值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetContrast(
    CameraHandle    hCamera,
    int             iContrast
);

/******************************************************/
// 函数名   : CameraGetContrast
// 功能描述 : 获得LUT动态生成模式下的对比度值。请参考
//        CameraSetContrast函数的功能描述。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piContrast 指针，返回当前的对比度值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetContrast(
    CameraHandle    hCamera,
    int*            piContrast
);

/******************************************************/
// 函数名   : CameraSetSaturation
// 功能描述 : 设定图像处理的饱和度。对黑白相机无效。
//        设定范围由CameraGetCapability获得。100表示
//        表示原始色度，不增强。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            iSaturation  设定的饱和度值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetSaturation(
    CameraHandle    hCamera,
    int             iSaturation
);

/******************************************************/
// 函数名   : CameraGetSaturation
// 功能描述 : 获得图像处理的饱和度。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            piSaturation 指针，返回当前图像处理的饱和度值。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetSaturation(
    CameraHandle    hCamera,
    int*            piSaturation
);

/******************************************************/
// 函数名   : CameraSetMonochrome
// 功能描述 : 设置彩色转为黑白功能的使能。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            bEnable   TRUE，表示将彩色图像转为黑白。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetMonochrome(
    CameraHandle    hCamera,
    BOOL            bEnable
);

/******************************************************/
// 函数名   : CameraGetMonochrome
// 功能描述 : 获得彩色转换黑白功能的使能状况。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbEnable   指针。返回TRUE表示开启了彩色图像
//             转换为黑白图像的功能。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetMonochrome(
    CameraHandle    hCamera,
    BOOL*           pbEnable
);

/******************************************************/
// 函数名   : CameraSetInverse
// 功能描述 : 设置彩图像颜色翻转功能的使能。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            bEnable    TRUE，表示开启图像颜色翻转功能，
//             可以获得类似胶卷底片的效果。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetInverse(
    CameraHandle    hCamera,
    BOOL            bEnable
);

/******************************************************/
// 函数名   : CameraGetInverse
// 功能描述 : 获得图像颜色反转功能的使能状态。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbEnable   指针，返回该功能使能状态。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetInverse(
    CameraHandle    hCamera,
    BOOL*           pbEnable
);

/******************************************************/
// 函数名   : CameraSetAntiFlick
// 功能描述 : 设置自动曝光时抗频闪功能的使能状态。对于手动
//        曝光模式下无效。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            bEnable    TRUE，开启抗频闪功能;FALSE，关闭该功能。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetAntiFlick(
    CameraHandle    hCamera,
    BOOL            bEnable
);

/******************************************************/
// 函数名   : CameraGetAntiFlick
// 功能描述 : 获得自动曝光时抗频闪功能的使能状态。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbEnable   指针，返回该功能的使能状态。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetAntiFlick(
    CameraHandle    hCamera,
    BOOL*           pbEnable
);

/******************************************************/
// 函数名   : CameraGetLightFrequency
// 功能描述 : 获得自动曝光时，消频闪的频率选择。
// 参数     : hCamera      相机的句柄，由CameraInit函数获得。
//            piFrequencySel 指针，返回选择的索引号。0:50HZ 1:60HZ
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetLightFrequency(
    CameraHandle    hCamera,
    int*            piFrequencySel
);

/******************************************************/
// 函数名   : CameraSetLightFrequency
// 功能描述 : 设置自动曝光时消频闪的频率。
// 参数     : hCamera     相机的句柄，由CameraInit函数获得。
//            iFrequencySel 0:50HZ , 1:60HZ
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetLightFrequency(
    CameraHandle    hCamera,
    int             iFrequencySel
);

/******************************************************/
// 函数名   : CameraSetFrameSpeed
// 功能描述 : 设定相机输出图像的帧率。相机可供选择的帧率模式由
//        CameraGetCapability获得的信息结构体中iFrameSpeedDesc
//        表示最大帧率选择模式个数。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iFrameSpeed 选择的帧率模式索引号，范围从0到
//              CameraGetCapability获得的信息结构体中iFrameSpeedDesc - 1
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetFrameSpeed(
    CameraHandle    hCamera,
    int             iFrameSpeed
);

/******************************************************/
// 函数名   : CameraGetFrameSpeed
// 功能描述 : 获得相机输出图像的帧率选择索引号。具体用法参考
//        CameraSetFrameSpeed函数的功能描述部分。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            piFrameSpeed 指针，返回选择的帧率模式索引号。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetFrameSpeed(
    CameraHandle    hCamera,
    int*            piFrameSpeed
);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief 设定相机的帧频(面阵)或行频(线阵)。（仅部分网口相机支持）
/// \param [in] hCamera 相机的句柄。
/// \param [in] RateHZ 帧频或行频（<=0表示最大频率）。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the frame frequency (area) or line frequency (line scan). (only supported by some gige camera)
/// \param [in] hCamera Camera handle.
/// \param [in] RateHZ frame rate or line rate (<=0 means maximum frequency).
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetFrameRate(
	CameraHandle    hCamera, 
	int             RateHZ
	);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief 获取设定的相机帧频(面阵)或行频(线阵)
/// \param [in] hCamera 相机的句柄。
/// \param [out] RateHZ 帧频或行频（<=0表示最大频率）。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the frame frequency (area) or line frequency (line scan).
/// \param [in] hCamera Camera handle.
/// \param [out] RateHZ frame rate or line rate (<=0 means maximum frequency).
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetFrameRate(
	CameraHandle    hCamera, 
	int*            RateHZ
	);

/******************************************************/
// 函数名   : CameraSetParameterMode
// 功能描述 : 设定参数存取的目标对象。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iMode  参数存取的对象。参考CameraDefine.h
//          中emSdkParameterMode的类型定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetParameterMode(
    CameraHandle    hCamera,
    int             iMode
);

/******************************************************/
// 函数名   : CameraGetParameterMode
// 功能描述 :
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            int* piTarget
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetParameterMode(
    CameraHandle    hCamera,
    int*            piTarget
);

/******************************************************/
// 函数名   : CameraSetParameterMask
// 功能描述 : 设置参数存取的掩码。参数加载和保存时会根据该
//        掩码来决定各个模块参数的是否加载或者保存。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            uMask     掩码。参考CameraDefine.h中PROP_SHEET_INDEX
//            类型定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetParameterMask(
    CameraHandle    hCamera,
    UINT            uMask
);

/******************************************************/
// 函数名   : CameraSaveParameter
// 功能描述 : 保存当前相机参数到指定的参数组中。相机提供了A,B,C,D
//        A,B,C,D四组空间来进行参数的保存。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iTeam      PARAMETER_TEAM_A 保存到A组中,
//             PARAMETER_TEAM_B 保存到B组中,
//             PARAMETER_TEAM_C 保存到C组中,
//             PARAMETER_TEAM_D 保存到D组中
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSaveParameter(
    CameraHandle    hCamera,
    int             iTeam
);


/******************************************************/
// 函数名   : CameraSaveParameterToFile
// 功能描述 : 保存当前相机参数到指定的文件中。该文件可以复制到
//        别的电脑上供其他相机加载，也可以做参数备份用。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            sFileName  参数文件的完整路径。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSaveParameterToFile(
  CameraHandle  hCamera,
  char*       sFileName
);


/******************************************************/
// 函数名   : CameraReadParameterFromFile
// 功能描述 : 从PC上指定的参数文件中加载参数。我公司相机参数
//        保存在PC上为.config后缀的文件，位于安装下的
//        Camera\Configs文件夹中。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            *sFileName 参数文件的完整路径。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraReadParameterFromFile(
    CameraHandle    hCamera,
    char*           sFileName
);

/******************************************************/
// 函数名   : CameraLoadParameter
// 功能描述 : 加载指定组的参数到相机中。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iTeam    PARAMETER_TEAM_A 加载A组参数,
//             PARAMETER_TEAM_B 加载B组参数,
//             PARAMETER_TEAM_C 加载C组参数,
//             PARAMETER_TEAM_D 加载D组参数,
//             PARAMETER_TEAM_DEFAULT 加载默认参数。
//             类型定义参考CameraDefine.h中emSdkParameterTeam类型
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraLoadParameter(
    CameraHandle    hCamera,
    int             iTeam
);

/******************************************************/
// 函数名   : CameraGetCurrentParameterGroup
// 功能描述 : 获得当前选择的参数组。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piTeam     指针，返回当前选择的参数组。返回值
//             参考CameraLoadParameter中iTeam参数。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetCurrentParameterGroup(
    CameraHandle    hCamera,
    int*            piTeam
);

/******************************************************/
// 函数名   : CameraSetTransPackLen
// 功能描述 : 设置相机传输图像数据的分包大小。
//        目前的SDK版本中，该接口仅对GIGE接口相机有效，
//        用来控制网络传输的分包大小。对于支持巨帧的网卡，
//        我们建议选择8K的分包大小，可以有效的降低传输
//        所占用的CPU处理时间。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iPackSel   分包长度选择的索引号。分包长度可由
//             获得相机属性结构体中pPackLenDesc成员表述，
//             iPackLenDesc成员则表示最大可选的分包模式个数。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetTransPackLen(
    CameraHandle    hCamera,
    INT             iPackSel
);

/******************************************************/
// 函数名   : CameraGetTransPackLen
// 功能描述 : 获得相机当前传输分包大小的选择索引号。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piPackSel  指针，返回当前选择的分包大小索引号。
//             参见CameraSetTransPackLen中iPackSel的
//             说明。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetTransPackLen(
    CameraHandle    hCamera,
    INT*            piPackSel
);

/******************************************************/
// 函数名   : CameraIsAeWinVisible
// 功能描述 : 获得自动曝光参考窗口的显示状态。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            pbIsVisible  指针，返回TRUE，则表示当前窗口会
//               被叠加在图像内容上。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraIsAeWinVisible(
    CameraHandle    hCamera,
    BOOL*           pbIsVisible
);

/******************************************************/
// 函数名   : CameraSetAeWinVisible
// 功能描述 : 设置自动曝光参考窗口的显示状态。当设置窗口状态
//        为显示，调用CameraImageOverlay后，能够将窗口位置
//        以矩形的方式叠加在图像上。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            bIsVisible  TRUE，设置为显示；FALSE，不显示。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetAeWinVisible(
    CameraHandle    hCamera,
    BOOL            bIsVisible
);

/******************************************************/
// 函数名   : CameraGetAeWindow
// 功能描述 : 获得自动曝光参考窗口的位置。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piHOff     指针，返回窗口位置左上角横坐标值。
//            piVOff     指针，返回窗口位置左上角纵坐标值。
//            piWidth    指针，返回窗口的宽度。
//            piHeight   指针，返回窗口的高度。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetAeWindow(
    CameraHandle    hCamera,
    INT*            piHOff,
    INT*            piVOff,
    INT*            piWidth,
    INT*            piHeight
);

/******************************************************/
// 函数名   : CameraSetAeWindow
// 功能描述 : 设置自动曝光的参考窗口。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iHOff    窗口左上角的横坐标
//            iVOff      窗口左上角的纵坐标
//            iWidth     窗口的宽度
//            iHeight    窗口的高度
//        如果iHOff、iVOff、iWidth、iHeight全部为0，则
//        窗口设置为每个分辨率下的居中1/2大小。可以随着
//        分辨率的变化而跟随变化；如果iHOff、iVOff、iWidth、iHeight
//        所决定的窗口位置范围超出了当前分辨率范围内，
//          则自动使用居中1/2大小窗口。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetAeWindow(
    CameraHandle    hCamera,
    int             iHOff,
    int             iVOff,
    int             iWidth,
    int             iHeight
);

/******************************************************/
// 函数名   : CameraSetMirror
// 功能描述 : 设置图像镜像操作。镜像操作分为水平和垂直两个方向。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iDir     表示镜像的方向。0，表示水平方向；1，表示垂直方向。
//            bEnable  TRUE，使能镜像;FALSE，禁止镜像
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetMirror(
    CameraHandle    hCamera,
    int             iDir,
    BOOL            bEnable
);

/******************************************************/
// 函数名   : CameraGetMirror
// 功能描述 : 获得图像的镜像状态。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iDir     表示要获得的镜像方向。
//             0，表示水平方向；1，表示垂直方向。
//            pbEnable   指针，返回TRUE，则表示iDir所指的方向
//             镜像被使能。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetMirror(
    CameraHandle    hCamera,
    int             iDir,
    BOOL*           pbEnable
);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief 设置硬件镜像。分为水平和垂直两个方向。（仅部分网口、U3相机支持此功能）
/// \param [in] hCamera 相机的句柄。
/// \param [in] iDir     表示镜像的方向。0，表示水平方向；1，表示垂直方向。
/// \param [in] bEnable  TRUE，使能镜像;FALSE，禁止镜像
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set up the hardware mirror. Divided into two directions, horizontal and vertical. (Only some GigE and U3 cameras support this feature)
/// \param [in] hCamera Camera handle.
/// \param [in] iDir Indicates the direction of the mirror. 0 means horizontal direction; 1 means vertical direction.
/// \param [in] bEnable TRUE to enable mirroring; FALSE to disable mirroring
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetHardwareMirror(
	CameraHandle    hCamera, 
	int             iDir, 
	BOOL            bEnable
	);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief 获取设置的硬件镜像状态。
/// \param [in] hCamera 相机的句柄。
/// \param [in] iDir     表示要获得的镜像方向。0，表示水平方向；1，表示垂直方向。
/// \param [out] pbEnable   指针，返回TRUE，则表示iDir所指的方向镜像被使能。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the hardware mirrored state of the image.
/// \param [in] hCamera Camera handle.
/// \param [in] iDir Indicates the mirroring direction to be obtained. 0 means horizontal direction; 1 means vertical direction.
/// \param [out] pbEnable Returns TRUE, indicating that the direction mirror image of iDir is enabled.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetHardwareMirror(
	CameraHandle    hCamera, 
	int             iDir, 
	BOOL*           pbEnable
	);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief 设置图像旋转操作
/// \param [in] hCamera 相机的句柄。
/// \param [in] iRot    表示旋转的角度（逆时针方向）（0：不旋转 1:90度 2:180度 3:270度）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set image rotation operation
/// \param [in] hCamera Camera handle.
/// \param [in] iRot rotation angle (counterclockwise) (0: no rotation 1:90 degrees 2:180 degrees 3:270 degrees)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetRotate(
	CameraHandle    hCamera,
	int             iRot
	);

/// @ingroup API_MIRROR
/// \~chinese
/// \brief 获得图像的旋转状态。
/// \param [in] hCamera 相机的句柄。
/// \param [out] iRot     表示要获得的旋转方向。（逆时针方向）（0：不旋转 1:90度 2:180度 3:270度）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the rotation state of the image.
/// \param [in] hCamera Camera handle.
/// \param [out] iRot Indicates the direction of rotation to get. (Counterclockwise) (0: Do not rotate 1:90 degree 2: 180 degree 3: 270 degree)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetRotate(
	CameraHandle    hCamera,
	int*            iRot
	);

/******************************************************/
// 函数名   : CameraGetWbWindow
// 功能描述 : 获得白平衡参考窗口的位置。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            PiHOff   指针，返回参考窗口的左上角横坐标 。
//            PiVOff     指针，返回参考窗口的左上角纵坐标 。
//            PiWidth    指针，返回参考窗口的宽度。
//            PiHeight   指针，返回参考窗口的高度。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetWbWindow(
    CameraHandle    hCamera,
    INT*            PiHOff,
    INT*            PiVOff,
    INT*            PiWidth,
    INT*            PiHeight
);

/******************************************************/
// 函数名   : CameraSetWbWindow
// 功能描述 : 设置白平衡参考窗口的位置。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iHOff   参考窗口的左上角横坐标。
//            iVOff     参考窗口的左上角纵坐标。
//            iWidth    参考窗口的宽度。
//            iHeight   参考窗口的高度。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetWbWindow(
    CameraHandle    hCamera,
    INT             iHOff,
    INT             iVOff,
    INT             iWidth,
    INT             iHeight
);

/******************************************************/
// 函数名   : CameraIsWbWinVisible
// 功能描述 : 获得白平衡窗口的显示状态。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbShow   指针，返回TRUE，则表示窗口是可见的。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraIsWbWinVisible(
    CameraHandle    hCamera,
    BOOL*           pbShow
);

/******************************************************/
// 函数名   : CameraSetWbWinVisible
// 功能描述 : 设置白平衡窗口的显示状态。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            bShow      TRUE，则表示设置为可见。在调用
//             CameraImageOverlay后，图像内容上将以矩形
//             的方式叠加白平衡参考窗口的位置。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetWbWinVisible(
    CameraHandle    hCamera,
    BOOL            bShow
);

/******************************************************/
// 函数名   : CameraImageOverlay
// 功能描述 : 将输入的图像数据上叠加十字线、白平衡参考窗口、
//        自动曝光参考窗口等图形。只有设置为可见状态的
//        十字线和参考窗口才能被叠加上。
//        注意，该函数的输入图像必须是RGB格式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pRgbBuffer 图像数据缓冲区。
//            pFrInfo    图像的帧头信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraImageOverlay(
    CameraHandle    hCamera,
    BYTE*           pRgbBuffer,
    tSdkFrameHead*  pFrInfo
);

/******************************************************/
// 函数名   : CameraSetCrossLine
// 功能描述 : 设置指定十字线的参数。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iLine    表示要设置第几条十字线的状态。范围为[0,8]，共9条。
//            x          十字线中心位置的横坐标值。
//            y      十字线中心位置的纵坐标值。
//            uColor     十字线的颜色，格式为(R|(G<<8)|(B<<16))
//            bVisible   十字线的显示状态。TRUE，表示显示。
//             只有设置为显示状态的十字线，在调用
//             CameraImageOverlay后才会被叠加到图像上。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetCrossLine(
    CameraHandle    hCamera,
    int             iLine,
    INT             x,
    INT             y,
    UINT            uColor,
    BOOL            bVisible
);

/******************************************************/
// 函数名   : CameraGetCrossLine
// 功能描述 : 获得指定十字线的状态。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iLine    表示要获取的第几条十字线的状态。范围为[0,8]，共9条。
//            px     指针，返回该十字线中心位置的横坐标。
//            py     指针，返回该十字线中心位置的横坐标。
//            pcolor     指针，返回该十字线的颜色，格式为(R|(G<<8)|(B<<16))。
//            pbVisible  指针，返回TRUE，则表示该十字线可见。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetCrossLine(
    CameraHandle    hCamera,
    INT             iLine,
    INT*            px,
    INT*            py,
    UINT*           pcolor,
    BOOL*           pbVisible
);

/******************************************************/
// 函数名   : CameraGetCapability
// 功能描述 : 获得相机的特性描述结构体。该结构体中包含了相机
//        可设置的各种参数的范围信息。决定了相关函数的参数
//        返回，也可用于动态创建相机的配置界面。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            pCameraInfo 指针，返回该相机特性描述的结构体。
//                        tSdkCameraCapbility在CameraDefine.h中定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetCapability(
    CameraHandle            hCamera,
    tSdkCameraCapbility*    pCameraInfo
);

/******************************************************/
// 函数名   : CameraGetCapabilityEx
// 功能描述 : 获得相机的特性描述结构体。该结构体中包含了相机
//        可设置的各种参数的范围信息。决定了相关函数的参数
//        返回，也可用于动态创建相机的配置界面。
// 参数     : sDeviceModel    相机的型号，由扫描列表中获取
//            pCameraInfo 指针，返回该相机特性描述的结构体。
//                        tSdkCameraCapbility在CameraDefine.h中定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetCapabilityEx(
    char*                   sDeviceModel,
    tSdkCameraCapbility*    pCameraInfo,
    PVOID                   hCameraHandle
);


/******************************************************/
// 函数名   : CameraWriteSN
// 功能描述 : 设置相机的序列号。我公司相机序列号分为3级。
//        0级的是我公司自定义的相机序列号，出厂时已经
//        设定好，1级和2级留给二次开发使用。每级序列
//        号长度都是32个字节。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbySN    序列号的缓冲区。
//            iLevel   要设定的序列号级别，只能是1或者2。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraWriteSN(
    CameraHandle    hCamera,
    BYTE*           pbySN,
    INT             iLevel
);

/******************************************************/
// 函数名   : CameraReadSN
// 功能描述 : 读取相机指定级别的序列号。序列号的定义请参考
//          CameraWriteSN函数的功能描述部分。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            pbySN    序列号的缓冲区。
//            iLevel     要读取的序列号级别。只能是1和2。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraReadSN(
    CameraHandle        hCamera,
    BYTE*               pbySN,
    INT                 iLevel
);
/******************************************************/
// 函数名   : CameraSetTriggerDelayTime
// 功能描述 : 设置硬件触发模式下的触发延时时间，单位微秒。
//        当硬触发信号来临后，经过指定的延时，再开始采集
//        图像。仅部分型号的相机支持该功能。具体请查看
//        产品说明书。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            uDelayTimeUs 硬触发延时。单位微秒。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetTriggerDelayTime(
    CameraHandle    hCamera,
    UINT            uDelayTimeUs
);

/******************************************************/
// 函数名   : CameraGetTriggerDelayTime
// 功能描述 : 获得当前设定的硬触发延时时间。
// 参数     : hCamera     相机的句柄，由CameraInit函数获得。
//            puDelayTimeUs 指针，返回延时时间，单位微秒。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetTriggerDelayTime(
    CameraHandle    hCamera,
    UINT*           puDelayTimeUs
);

/******************************************************/
// 函数名 	: CameraSetTriggerPeriodTime
// 功能描述	: 一次触发多帧时，设置2帧之间的间隔时间。
//           仅部分型号的相机支持该功能。
// 参数	    : hCamera	   相机的句柄，由CameraInit函数获得。
//              time       间隔时间（微秒）
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraSetTriggerPeriodTime(
    CameraHandle hCamera,
    UINT time
);

/******************************************************/
// 函数名 	: CameraGetTriggerPeriodTime
// 功能描述	: 获取当前设置的间隔时间。
// 参数	    : hCamera	   相机的句柄，由CameraInit函数获得。
//              time       间隔时间（微秒）
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraGetTriggerPeriodTime(
    CameraHandle hCamera,
    UINT* time
);

/******************************************************/
// 函数名   : CameraSetTriggerCount
// 功能描述 : 设置触发模式下的触发帧数。对软件触发和硬件触发
//        模式都有效。默认为1帧，即一次触发信号采集一帧图像。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iCount    一次触发采集的帧数。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetTriggerCount(
    CameraHandle    hCamera,
    INT             iCount
);

/******************************************************/
// 函数名   : CameraGetTriggerCount
// 功能描述 : 获得一次触发的帧数。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            INT* piCount
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetTriggerCount(
    CameraHandle    hCamera,
    INT*            piCount
);

/******************************************************/
// 函数名   : CameraSoftTrigger
// 功能描述 : 执行一次软触发。执行后，会触发由CameraSetTriggerCount
//          指定的帧数。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSoftTrigger(
    CameraHandle    hCamera
);

/******************************************************/
// 函数名   : CameraSetTriggerMode
// 功能描述 : 设置相机的触发模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iModeSel   模式选择索引号。可设定的模式由
//             CameraGetCapability函数获取。请参考
//               CameraDefine.h中tSdkCameraCapbility的定义。
//             一般情况，0表示连续采集模式；1表示
//             软件触发模式；2表示硬件触发模式。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetTriggerMode(
    CameraHandle    hCamera,
    int             iModeSel
);

/******************************************************/
// 函数名   : CameraGetTriggerMode
// 功能描述 : 获得相机的触发模式。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            piModeSel  指针，返回当前选择的相机触发模式的索引号。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetTriggerMode(
    CameraHandle    hCamera,
    INT*            piModeSel
);

/******************************************************/
// 函数名 	: CameraSetStrobeMode
// 功能描述	: 设置IO引脚端子上的STROBE信号。该信号可以做闪光灯控制，也可以做外部机械快门控制。
// 参数	    : hCamera 相机的句柄，由CameraInit函数获得。
//             iMode   当为STROBE_SYNC_WITH_TRIG_AUTO      和触发信号同步，触发后，相机进行曝光时，自动生成STROBE信号。
//                                                         此时，有效极性可设置(CameraSetStrobePolarity)。
//                     当为STROBE_SYNC_WITH_TRIG_MANUAL时，和触发信号同步，触发后，STROBE延时指定的时间后(CameraSetStrobeDelayTime)，
//                                                         再持续指定时间的脉冲(CameraSetStrobePulseWidth)，
//                                                         有效极性可设置(CameraSetStrobePolarity)。
//                     当为STROBE_ALWAYS_HIGH时，STROBE信号恒为高,忽略其他设置
//                     当为STROBE_ALWAYS_LOW时，STROBE信号恒为低,忽略其他设置
//
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraSetStrobeMode(
    CameraHandle    hCamera,
    INT             iMode
);

/******************************************************/
// 函数名 	: CameraGetStrobeMode
// 功能描述	: 或者当前STROBE信号设置的模式。
// 参数	    : hCamera 相机的句柄，由CameraInit函数获得。
//             piMode  指针，返回STROBE_SYNC_WITH_TRIG_AUTO,STROBE_SYNC_WITH_TRIG_MANUAL、STROBE_ALWAYS_HIGH或者STROBE_ALWAYS_LOW。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetStrobeMode(
    CameraHandle    hCamera,
    INT*            piMode
);

/******************************************************/
// 函数名 	: CameraSetStrobeDelayTime
// 功能描述	: 当STROBE信号处于STROBE_SYNC_WITH_TRIG时，通过该函数设置其相对触发信号延时时间。
// 参数	    : hCamera       相机的句柄，由CameraInit函数获得。
//             uDelayTimeUs  相对触发信号的延时时间，单位为us。可以为0，但不能为负数。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraSetStrobeDelayTime(
    CameraHandle    hCamera,
    UINT            uDelayTimeUs
);

/******************************************************/
// 函数名 	: CameraGetStrobeDelayTime
// 功能描述	: 当STROBE信号处于STROBE_SYNC_WITH_TRIG时，通过该函数获得其相对触发信号延时时间。
// 参数	    : hCamera           相机的句柄，由CameraInit函数获得。
//             upDelayTimeUs     指针，返回延时时间，单位us。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetStrobeDelayTime(
    CameraHandle    hCamera,
    UINT*           upDelayTimeUs
);

/******************************************************/
// 函数名 	: CameraSetStrobePulseWidth
// 功能描述	: 当STROBE信号处于STROBE_SYNC_WITH_TRIG时，通过该函数设置其脉冲宽度。
// 参数	    : hCamera       相机的句柄，由CameraInit函数获得。
//             uTimeUs       脉冲的宽度，单位为时间us。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraSetStrobePulseWidth(
    CameraHandle    hCamera,
    UINT            uTimeUs
);

/******************************************************/
// 函数名 	: CameraGetStrobePulseWidth
// 功能描述	: 当STROBE信号处于STROBE_SYNC_WITH_TRIG时，通过该函数获得其脉冲宽度。
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//             upTimeUs  指针，返回脉冲宽度。单位为时间us。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetStrobePulseWidth(
    CameraHandle    hCamera,
    UINT*           upTimeUs
);

/******************************************************/
// 函数名 	: CameraSetStrobePolarity
// 功能描述	: 当STROBE信号处于STROBE_SYNC_WITH_TRIG时，通过该函数设置其有效电平的极性。默认为高有效，当触发信号到来时，STROBE信号被拉高。
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//             iPolarity STROBE信号的极性，0为低电平有效，1为高电平有效。默认为高电平有效。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraSetStrobePolarity(
    CameraHandle    hCamera,
    INT             uPolarity
);

/******************************************************/
// 函数名 	: CameraGetStrobePolarity
// 功能描述	: 获得相机当前STROBE信号的有效极性。默认为高电平有效。
// 参数	    : hCamera       相机的句柄，由CameraInit函数获得。
//             ipPolarity    指针，返回STROBE信号当前的有效极性。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetStrobePolarity(
    CameraHandle    hCamera,
    INT*            upPolarity
);

/******************************************************/
// 函数名 	: CameraSetExtTrigSignalType
// 功能描述	: 设置相机外触发信号的种类。上边沿、下边沿、或者高、低电平方式。
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//             iType     外触发信号种类，返回值参考CameraDefine.h中
//                       emExtTrigSignal类型定义。

// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraSetExtTrigSignalType(
    CameraHandle    hCamera,
    INT             iType
);

/******************************************************/
// 函数名 	: CameraGetExtTrigSignalType
// 功能描述	: 获得相机当前外触发信号的种类。
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//             ipType    指针，返回外触发信号种类，返回值参考CameraDefine.h中
//                       emExtTrigSignal类型定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetExtTrigSignalType(
    CameraHandle    hCamera,
    INT*            ipType
);

/******************************************************/
// 函数名 	: CameraSetExtTrigShutterType
// 功能描述	: 设置外触发模式下，相机快门的方式，默认为标准快门方式。
//              部分滚动快门的CMOS相机支持GRR方式。
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//             iType     外触发快门方式。参考CameraDefine.h中emExtTrigShutterMode类型。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraSetExtTrigShutterType(
    CameraHandle    hCamera,
    INT             iType
);

/******************************************************/
// 函数名 	: CameraSetExtTrigShutterType
// 功能描述	: 获得外触发模式下，相机快门的方式，默认为标准快门方式。
//              部分滚动快门的CMOS相机支持GRR方式。
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//             ipType    指针，返回当前设定的外触发快门方式。返回值参考
//                       CameraDefine.h中emExtTrigShutterMode类型。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetExtTrigShutterType(
    CameraHandle    hCamera,
    INT*            ipType
);

/******************************************************/
// 函数名 	: CameraSetExtTrigDelayTime
// 功能描述	: 设置外触发信号延时时间，默认为0，单位为微秒。
//              当设置的值uDelayTimeUs不为0时，相机接收到外触发信号后，将延时uDelayTimeUs个微秒后再进行图像捕获。
// 参数	    : hCamera       相机的句柄，由CameraInit函数获得。
//             uDelayTimeUs  延时时间，单位为微秒，默认为0.
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraSetExtTrigDelayTime(
    CameraHandle    hCamera,
    UINT            uDelayTimeUs
);

/******************************************************/
// 函数名 	: CameraGetExtTrigDelayTime
// 功能描述	: 获得设置的外触发信号延时时间，默认为0，单位为微秒。
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//            UINT* upDelayTimeUs
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetExtTrigDelayTime(
    CameraHandle    hCamera,
    UINT*           upDelayTimeUs
);

/******************************************************/
// 函数名 	: CameraSetExtTrigBufferedDelayTime
// 功能描述	: 设置外触发信号延时时间，默认为0，单位为微秒。
//           当设置的值uDelayTimeUs不为0时，相机接收到外触发信号后，将延时uDelayTimeUs个微秒后再进行图像捕获。
//           并且会把延时期间收到的触发信号缓存起来，被缓存的信号也将延时uDelayTimeUs个微秒后生效（最大缓存个数128）。
// 参数	    : hCamera       相机的句柄，由CameraInit函数获得。
//            uDelayTimeUs   延时时间，单位为微秒，默认为0.
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraSetExtTrigBufferedDelayTime(
    CameraHandle    hCamera,
    UINT            uDelayTimeUs
);

/******************************************************/
// 函数名 	: CameraGetExtTrigBufferedDelayTime
// 功能描述	: 获得设置的外触发信号延时时间，默认为0，单位为微秒。
// 参数	    : hCamera       相机的句柄，由CameraInit函数获得。
//            puDelayTimeUs   延时时间，单位为微秒，默认为0.
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraGetExtTrigBufferedDelayTime(
    CameraHandle    hCamera,
    UINT*           puDelayTimeUs
);

/******************************************************/
// 函数名 	: CameraSetExtTrigJitterTime
// 功能描述	: 设置相机外触发信号的消抖时间。默认为0，单位为微秒。
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//            UINT uTimeUs
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraSetExtTrigJitterTime(
    CameraHandle    hCamera,
    UINT            uTimeUs
);

/******************************************************/
// 函数名 	: CameraGetExtTrigJitterTime
// 功能描述	: 获得设置的相机外触发消抖时间，默认为0.单位为微妙
// 参数	    : hCamera   相机的句柄，由CameraInit函数获得。
//            UINT* upTimeUs
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetExtTrigJitterTime(
    CameraHandle    hCamera,
    UINT*           upTimeUs
);

/******************************************************/
// 函数名 	: CameraGetExtTrigCapability
// 功能描述	: 获得相机外触发的属性掩码
// 参数	    : hCamera           相机的句柄，由CameraInit函数获得。
//             puCapabilityMask  指针，返回该相机外触发特性掩码，掩码参考CameraDefine.h中
//                               EXT_TRIG_MASK_ 开头的宏定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus   CameraGetExtTrigCapability(
    CameraHandle    hCamera,
    UINT*           puCapabilityMask
);


/******************************************************/
// 函数名   : CameraGetResolutionForSnap
// 功能描述 : 获得抓拍模式下的分辨率选择索引号。
// 参数     : hCamera        相机的句柄，由CameraInit函数获得。
//            pImageResolution 指针，返回抓拍模式的分辨率。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetResolutionForSnap(
    CameraHandle            hCamera,
    tSdkImageResolution*    pImageResolution
);

/******************************************************/
// 函数名   : CameraSetResolutionForSnap
// 功能描述 : 设置抓拍模式下相机输出图像的分辨率。
// 参数     : hCamera       相机的句柄，由CameraInit函数获得。
//            pImageResolution 如果pImageResolution->iWidth
//                 和 pImageResolution->iHeight都为0，
//                         则表示设定为跟随当前预览分辨率。抓
//                         怕到的图像的分辨率会和当前设定的
//                 预览分辨率一样。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetResolutionForSnap(
    CameraHandle            hCamera,
    tSdkImageResolution*    pImageResolution
);

/******************************************************/
// 函数名   : CameraCustomizeResolution
// 功能描述 : 打开分辨率自定义面板，并通过可视化的方式
//        来配置一个自定义分辨率。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            pImageCustom 指针，返回自定义的分辨率。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraCustomizeResolution(
    CameraHandle            hCamera,
    tSdkImageResolution*    pImageCustom
);

/******************************************************/
// 函数名   : CameraCustomizeReferWin
// 功能描述 : 打开参考窗口自定义面板。并通过可视化的方式来
//        获得一个自定义窗口的位置。一般是用自定义白平衡
//        和自动曝光的参考窗口。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            iWinType   要生成的参考窗口的用途。0,自动曝光参考窗口；
//             1,白平衡参考窗口。
//            hParent    调用该函数的窗口的句柄。可以为NULL。
//            piHOff     指针，返回自定义窗口的左上角横坐标。
//            piVOff     指针，返回自定义窗口的左上角纵坐标。
//            piWidth    指针，返回自定义窗口的宽度。
//            piHeight   指针，返回自定义窗口的高度。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraCustomizeReferWin(
    CameraHandle    hCamera,
    INT             iWinType,
    HWND            hParent,
    INT*            piHOff,
    INT*            piVOff,
    INT*            piWidth,
    INT*            piHeight
);

/******************************************************/
// 函数名   : CameraShowSettingPage
// 功能描述 : 设置相机属性配置窗口显示状态。必须先调用CameraCreateSettingPage
//        成功创建相机属性配置窗口后，才能调用本函数进行
//        显示。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            bShow    TRUE，显示;FALSE，隐藏。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraShowSettingPage(
    CameraHandle    hCamera,
    BOOL            bShow
);

/******************************************************/
// 函数名   : CameraCreateSettingPage
// 功能描述 : 创建该相机的属性配置窗口。调用该函数，SDK内部会
//        帮您创建好相机的配置窗口，省去了您重新开发相机
//        配置界面的时间。强烈建议使用您使用该函数让
//        SDK为您创建好配置窗口。
// 参数     : hCamera     相机的句柄，由CameraInit函数获得。
//            hParent       应用程序主窗口的句柄。可以为NULL。
//            pWinText      字符串指针，窗口显示的标题栏。
//            pCallbackFunc 窗口消息的回调函数，当相应的事件发生时，
//              pCallbackFunc所指向的函数会被调用，
//              例如切换了参数之类的操作时，pCallbackFunc
//              被回调时，在入口参数处指明了消息类型。
//              这样可以方便您自己开发的界面和我们生成的UI
//              之间进行同步。该参数可以为NULL。
//            pCallbackCtx  回调函数的附加参数。可以为NULL。pCallbackCtx
//              会在pCallbackFunc被回调时，做为参数之一传入。
//              您可以使用该参数来做一些灵活的判断。
//            uReserved     预留。必须设置为0。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraCreateSettingPage(
    CameraHandle            hCamera,
    HWND                    hParent,
    char*                   pWinText,
    CAMERA_PAGE_MSG_PROC    pCallbackFunc,
    PVOID                   pCallbackCtx,
    UINT                    uReserved
);

/******************************************************/
// 函数名   : CameraCreateSettingPageEx
// 功能描述 : 创建该相机的属性配置窗口。调用该函数，SDK内部会
//        帮您创建好相机的配置窗口，省去了您重新开发相机
//        配置界面的时间。强烈建议使用您使用该函数让
//        SDK为您创建好配置窗口。
// 参数     : hCamera     相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraCreateSettingPageEx(
    CameraHandle            hCamera
);


/******************************************************/
// 函数名   : CameraSetActiveSettingSubPage
// 功能描述 : 设置相机配置窗口的激活页面。相机配置窗口有多个
//        子页面构成，该函数可以设定当前哪一个子页面
//        为激活状态，显示在最前端。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            index      子页面的索引号。参考CameraDefine.h中
//             PROP_SHEET_INDEX的定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetActiveSettingSubPage(
    CameraHandle    hCamera,
    INT             index
);

/******************************************************/
// 函数名   : CameraSpecialControl
// 功能描述 : 相机一些特殊配置所调用的接口，二次开发时一般不需要
//        调用。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            dwCtrlCode 控制码。
//            dwParam    控制子码，不同的dwCtrlCode时，意义不同。
//            lpData     附加参数。不同的dwCtrlCode时，意义不同。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSpecialControl(
    CameraHandle    hCamera,
    DWORD           dwCtrlCode,
    DWORD           dwParam,
    LPVOID          lpData
);

/******************************************************/
// 函数名   : CameraGetFrameStatistic
// 功能描述 : 获得相机接收帧率的统计信息，包括错误帧和丢帧的情况。
// 参数     : hCamera        相机的句柄，由CameraInit函数获得。
//            psFrameStatistic 指针，返回统计信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetFrameStatistic(
    CameraHandle            hCamera,
    tSdkFrameStatistic*     psFrameStatistic
);

/******************************************************/
// 函数名   : CameraGetStatisticResend
// 功能描述 : 获得相机统计信息之重传数量
// 参数     : hCamera        相机的句柄，由CameraInit函数获得。
//            pResendCount  指针，返回重传数量。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetStatisticResend(
    CameraHandle            hCamera,
    UINT*                   pResendCount
);

/******************************************************/
// 函数名   : CameraSetNoiseFilter
// 功能描述 : 设置图像降噪模块的使能状态。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            bEnable   TRUE，使能；FALSE，禁止。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetNoiseFilter(
    CameraHandle    hCamera,
    BOOL            bEnable
);

/******************************************************/
// 函数名   : CameraGetNoiseFilterState
// 功能描述 : 获得图像降噪模块的使能状态。
// 参数     : hCamera  相机的句柄，由CameraInit函数获得。
//            *pEnable   指针，返回状态。TRUE，为使能。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetNoiseFilterState(
    CameraHandle    hCamera,
    BOOL*           pEnable
);

/******************************************************/
// 函数名   : CameraRstTimeStamp
// 功能描述 : 复位图像采集的时间戳，从0开始。
// 参数     : CameraHandle hCamera
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraRstTimeStamp(
    CameraHandle    hCamera
);

/******************************************************/
// 函数名   : CameraSaveUserData
// 功能描述 : 将用户自定义的数据保存到相机的非易性存储器中。
//              每个型号的相机可能支持的用户数据区最大长度不一样。
//              可以从设备的特性描述中获取该长度信息。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            uStartAddr  起始地址，从0开始。
//            pbData      数据缓冲区指针
//            ilen        写入数据的长度，ilen + uStartAddr必须
//                        小于用户区最大长度
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSaveUserData(
    CameraHandle    hCamera,
    UINT            uStartAddr,
    BYTE            *pbData,
    int             ilen
);

/******************************************************/
// 函数名   : CameraLoadUserData
// 功能描述 : 从相机的非易性存储器中读取用户自定义的数据。
//              每个型号的相机可能支持的用户数据区最大长度不一样。
//              可以从设备的特性描述中获取该长度信息。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            uStartAddr  起始地址，从0开始。
//            pbData      数据缓冲区指针，返回读到的数据。
//            ilen        读取数据的长度，ilen + uStartAddr必须
//                        小于用户区最大长度
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraLoadUserData(
    CameraHandle    hCamera,
    UINT            uStartAddr,
    BYTE            *pbData,
    int             ilen
);

/******************************************************/
// 函数名   : CameraGetFriendlyName
// 功能描述 : 读取用户自定义的设备昵称。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            pName      指针，返回指向0结尾的字符串，
//             设备昵称不超过32个字节，因此该指针
//             指向的缓冲区必须大于等于32个字节空间。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetFriendlyName(
  CameraHandle  hCamera,
  char*     pName
);

/******************************************************/
// 函数名   : CameraSetFriendlyName
// 功能描述 : 设置用户自定义的设备昵称。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            pName      指针，指向0结尾的字符串，
//             设备昵称不超过32个字节，因此该指针
//             指向字符串必须小于等于32个字节空间。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetFriendlyName(
  CameraHandle  hCamera,
  char*       pName
);

/******************************************************/
// 函数名   : CameraSdkGetVersionString
// 功能描述 :
// 参数     : pVersionString 指针，返回SDK版本字符串。
//                            该指针指向的缓冲区大小必须大于
//                            32个字节
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSdkGetVersionString(
  char*       pVersionString
);

/******************************************************/
// 函数名   : CameraCheckFwUpdate
// 功能描述 : 检测固件版本，是否需要升级。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            pNeedUpdate 指针，返回固件检测状态，TRUE表示需要更新
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraCheckFwUpdate(
  CameraHandle  hCamera,
  BOOL*     pNeedUpdate
);

/******************************************************/
// 函数名   : CameraGetFirmwareVision
// 功能描述 : 获得固件版本的字符串
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            pVersion 必须指向一个大于32字节的缓冲区，
//                      返回固件的版本字符串。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetFirmwareVersion(
  CameraHandle  hCamera,
  char*     pVersion
);

/******************************************************/
// 函数名   : CameraGetEnumInfo
// 功能描述 : 获得指定设备的枚举信息
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            pCameraInfo 指针，返回设备的枚举信息。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetEnumInfo(
  CameraHandle    hCamera,
  tSdkCameraDevInfo*  pCameraInfo
);

/******************************************************/
// 函数名   : CameraGetInerfaceVersion
// 功能描述 : 获得指定设备接口的版本
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            pVersion 指向一个大于32字节的缓冲区，返回接口版本字符串。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetInerfaceVersion(
  CameraHandle    hCamera,
  char*       pVersion
);

/******************************************************/
// 函数名   : CameraSetIOState
// 功能描述 : 设置指定IO的电平状态，IO为输出型IO，相机
//              预留可编程输出IO的个数由tSdkCameraCapbility中
//              iOutputIoCounts决定。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iOutputIOIndex IO的索引号，从0开始。
//            uState 要设定的状态，1为高，0为低
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetIOState(
  CameraHandle    hCamera,
  INT         iOutputIOIndex,
  UINT        uState
);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 读取指定IO的电平状态，IO为输出型IO，相机预留可编程输出IO的个数由@link #tSdkCameraCapbility.iOutputIoCounts @endlink决定。
/// \param [in] hCamera 相机的句柄。
/// \param [in] iOutputIOIndex IO的索引号，从0开始。
/// \param [out] puState 返回IO状态，1为高，0为低
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Read the level state of the specified IO. IO is the output IO. The number of programmable output IOs for the camera is determined by @link #tSdkCameraCapbility.iOutputIoCounts @endlink.
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] puState return IO state, 1 is high, 0 is low
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetOutPutIOState(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	UINT*       puState
	);

/******************************************************/
// 函数名   : CameraGetIOState
// 功能描述 : 设置指定IO的电平状态，IO为输入型IO，相机
//              预留可编程输出IO的个数由tSdkCameraCapbility中
//              iInputIoCounts决定。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iInputIOIndex IO的索引号，从0开始。
//            puState 指针，返回IO状态,1为高，0为低
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetIOState(
  CameraHandle    hCamera,
  INT         iInputIOIndex,
  UINT*         puState
);



/******************************************************/
// 函数名   : CameraSetInPutIOMode
// 功能描述 : 设置输入IO的模式，相机
//              预留可编程输出IO的个数由tSdkCameraCapbility中
//              iInputIoCounts决定。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iInputIOIndex IO的索引号，从0开始。
//            iMode IO模式,参考CameraDefine.h中emCameraGPIOMode
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetInPutIOMode(
	CameraHandle    hCamera,
	INT         iInputIOIndex,
	INT			iMode
	);
	
/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取输入IO的模式
/// \param [in] hCamera 相机的句柄。
/// \param [in] iInputIOIndex IO的索引号，从0开始。
/// \param [out] piMode IO模式,参考@link #emCameraGPIOMode @endlink
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the input IO mode
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] piMode IO mode, reference @link #emCameraGPIOMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetInPutIOMode(
	CameraHandle    hCamera,
	INT				iInputIOIndex,
	INT*			piMode
	);

/******************************************************/
// 函数名   : CameraSetOutPutIOMode
// 功能描述 : 设置输出IO的模式，相机
//              预留可编程输出IO的个数由tSdkCameraCapbility中
//              iOutputIoCounts决定。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iOutputIOIndex IO的索引号，从0开始。
//            iMode IO模式,参考CameraDefine.h中emCameraGPIOMode
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetOutPutIOMode(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	INT			iMode
	);
	
/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取输出IO的模式
/// \param [in] hCamera 相机的句柄。
/// \param [in] iOutputIOIndex IO的索引号，从0开始。
/// \param [out] piMode IO模式,参考@link #emCameraGPIOMode @endlink
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the output IO mode
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] piMode IO mode, reference @link #emCameraGPIOMode @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetOutPutIOMode(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	INT*		piMode
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取输入IO的模式支持能力
/// \param [in] hCamera 相机的句柄。
/// \param [in] iInputIOIndex IO的索引号，从0开始。
/// \param [out] piCapbility IO模式支持位掩码
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the mode support capability of the input IO
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] piCapbility IO mode support bit mask
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetInPutIOModeCapbility(
	CameraHandle    hCamera,
	INT				iInputIOIndex,
	UINT*			piCapbility
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取输出IO的模式支持能力
/// \param [in] hCamera 相机的句柄。
/// \param [in] iOutputIOIndex IO的索引号，从0开始。
/// \param [out] piCapbility IO模式支持位掩码
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the mode support capability of the output IO
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] piCapbility IO mode support bit mask
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetOutPutIOModeCapbility(
	CameraHandle    hCamera,
	INT				iOutputIOIndex,
	UINT*			piCapbility
	);

/******************************************************/
// 函数名   : CameraSetOutPutPWM
// 功能描述 : 设置PWM型输出的参数，相机
//              预留可编程输出IO的个数由tSdkCameraCapbility中
//              iOutputIoCounts决定。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iOutputIOIndex IO的索引号，从0开始。
//            iCycle PWM的周期，单位(us)
//			  uDuty  占用比，取值1%~99%
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetOutPutPWM(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	UINT		iCycle,
	UINT		uDuty
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 设置编码器有效方向
/// \param [in] hCamera 相机的句柄。
/// \param [in] dir 有效方向（0:正反转都有效   1：顺时针（A相超前于B）   2:逆时针）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the effective direction of the rotary encoder
/// \param [in] hCamera Camera handle.
/// \param [in] dir Valid direction (0: Both positive and negative are valid    1: Clockwise (A phase leads B)    2: Counterclockwise)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetRotaryEncDir(
	CameraHandle    hCamera,
	INT				dir
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取编码器有效方向
/// \param [in] hCamera 相机的句柄。
/// \param [out] dir 有效方向（0:正反转都有效   1：顺时针（A相超前于B）   2:逆时针）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the effective direction of the rotary encoder
/// \param [in] hCamera Camera handle.
/// \param [out] dir Valid direction (0: Both positive and negative are valid    1: Clockwise (A phase leads B)    2: Counterclockwise)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetRotaryEncDir(
	CameraHandle    hCamera,
	INT*			dir
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 设置编码器频率
/// \param [in] hCamera 相机的句柄。
/// \param [in] mul 倍频
/// \param [in] div 分频
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the frequency of the rotary encoder
/// \param [in] hCamera Camera handle.
/// \param [in] mul frequency multiplier
/// \param [in] div frequency division
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetRotaryEncFreq(
	CameraHandle hCamera,
	INT			mul,
	INT			div
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取编码器频率
/// \param [in] hCamera 相机的句柄。
/// \param [out] mul 倍频
/// \param [out] div 分频
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the frequency of the rotary encoder
/// \param [in] hCamera Camera handle.
/// \param [out] mul frequency multiplier
/// \param [out] div frequency division
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetRotaryEncFreq(
	CameraHandle hCamera,
	INT*		mul,
	INT*		div
	);
	
/// @ingroup API_GPIO
/// \~chinese
/// \brief 设置输入IO的格式
/// \param [in] hCamera 相机的句柄。
/// \param [in] iInputIOIndex IO的索引号，从0开始。
/// \param [in] iFormat IO格式,参考@link #emCameraGPIOFormat @endlink
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the input IO format
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [in] iFormat IO format, reference @link #emCameraGPIOFormat @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetInPutIOFormat(
	CameraHandle    hCamera,
	INT         iInputIOIndex,
	INT			iFormat
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取输入IO的格式
/// \param [in] hCamera 相机的句柄。
/// \param [in] iInputIOIndex IO的索引号，从0开始。
/// \param [out] piFormat IO格式,参考@link #emCameraGPIOFormat @endlink
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the input IO format
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] piFormat IO format, reference @link #emCameraGPIOFormat @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetInPutIOFormat(
	CameraHandle    hCamera,
	INT				iInputIOIndex,
	INT*			piFormat
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 设置输出IO的格式
/// \param [in] hCamera 相机的句柄。
/// \param [in] iOutputIOIndex IO的索引号，从0开始。
/// \param [in] iFormat IO格式,参考@link #emCameraGPIOFormat @endlink
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the output IO format
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [in] iFormat IO format, reference @link #emCameraGPIOFormat @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetOutPutIOFormat(
	CameraHandle    hCamera,
	INT         iOutputIOIndex,
	INT			iFormat
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取输出IO的格式
/// \param [in] hCamera 相机的句柄。
/// \param [in] iOutputIOIndex IO的索引号，从0开始。
/// \param [out] piFormat IO格式,参考@link #emCameraGPIOFormat @endlink
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the output IO format
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] piFormat IO format, reference @link #emCameraGPIOFormat @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetOutPutIOFormat(
	CameraHandle    hCamera,
	INT				iOutputIOIndex,
	INT*			piFormat
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取输入IO的格式支持能力
/// \param [in] hCamera 相机的句柄。
/// \param [in] iInputIOIndex IO的索引号，从0开始。
/// \param [out] piCapbility IO格式支持位掩码
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the format support capability of the input IO
/// \param [in] hCamera Camera handle.
/// \param [in] iInputIOIndex IO index, starting from 0.
/// \param [out] piCapbility IO format support bit mask
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetInPutIOFormatCapbility(
	CameraHandle    hCamera,
	INT				iInputIOIndex,
	UINT*			piCapbility
	);

/// @ingroup API_GPIO
/// \~chinese
/// \brief 获取输出IO的格式支持能力
/// \param [in] hCamera 相机的句柄。
/// \param [in] iOutputIOIndex IO的索引号，从0开始。
/// \param [out] piCapbility IO格式支持位掩码
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the format support capability of the output IO
/// \param [in] hCamera Camera handle.
/// \param [in] iOutputIOIndex IO index, starting from 0.
/// \param [out] piCapbility IO format support bit mask
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetOutPutIOFormatCapbility(
	CameraHandle    hCamera,
	INT				iOutputIOIndex,
	UINT*			piCapbility
	);

/******************************************************/
// 函数名   : CameraSetOutPutDelayTime
// 功能描述 : 设置输出延时，相机
//              预留可编程输出IO的个数由tSdkCameraCapbility中
//              iOutputIoCounts决定。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iOutputIOIndex IO的索引号，从0开始。
//            uDelayTimeUs 延时，单位(us)
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraSetOutPutDelayTime(
	CameraHandle    hCamera,
	INT             iOutputIOIndex,
	UINT            uDelayTimeUs
	);

/******************************************************/
// 函数名   : CameraSetOutPutPulseWidth
// 功能描述 : 设置输出脉宽，相机
//              预留可编程输出IO的个数由tSdkCameraCapbility中
//              iOutputIoCounts决定。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iOutputIOIndex IO的索引号，从0开始。
//            uTimeUs 脉宽，单位(us)
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraSetOutPutPulseWidth(
	CameraHandle    hCamera,
	INT             iOutputIOIndex,
	UINT            uTimeUs
	);

/******************************************************/
// 函数名   : CameraSetOutPutPolarity
// 功能描述 : 设置输出极性，相机
//              预留可编程输出IO的个数由tSdkCameraCapbility中
//              iOutputIoCounts决定。
// 参数     : hCamera 相机的句柄，由CameraInit函数获得。
//            iOutputIOIndex IO的索引号，从0开始。
//            uPolarity    0:正向    1：反向
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraSetOutPutPolarity(
	CameraHandle    hCamera,
	INT             iOutputIOIndex,
	UINT            uPolarity
	);

/******************************************************/
// 函数名   : CameraSetAeAlgorithm
// 功能描述 : 设置自动曝光时选择的算法，不同的算法适用于
//        不同的场景。
// 参数     : hCamera       相机的句柄，由CameraInit函数获得。
//            iIspProcessor   选择执行该算法的对象，参考CameraDefine.h
//                emSdkIspProcessor的定义
//            iAeAlgorithmSel   要选择的算法编号。从0开始，最大值由tSdkCameraCapbility
//                中iAeAlmSwDesc和iAeAlmHdDesc决定。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetAeAlgorithm(
    CameraHandle    hCamera,
    INT             iIspProcessor,
    INT             iAeAlgorithmSel
);

/******************************************************/
// 函数名   : CameraGetAeAlgorithm
// 功能描述 : 获得当前自动曝光所选择的算法
// 参数     : hCamera       相机的句柄，由CameraInit函数获得。
//            iIspProcessor   选择执行该算法的对象，参考CameraDefine.h
//                emSdkIspProcessor的定义
//            piAeAlgorithmSel  返回当前选择的算法编号。从0开始，最大值由tSdkCameraCapbility
//                中iAeAlmSwDesc和iAeAlmHdDesc决定。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetAeAlgorithm(
    CameraHandle    hCamera,
    INT             iIspProcessor,
    INT*            piAlgorithmSel
);

/******************************************************/
// 函数名   : CameraSetBayerDecAlgorithm
// 功能描述 : 设置Bayer数据转彩色的算法。
// 参数     : hCamera       相机的句柄，由CameraInit函数获得。
//            iIspProcessor   选择执行该算法的对象，参考CameraDefine.h
//                emSdkIspProcessor的定义
//            iAlgorithmSel     要选择的算法编号。从0开始，最大值由tSdkCameraCapbility
//                中iBayerDecAlmSwDesc和iBayerDecAlmHdDesc决定。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetBayerDecAlgorithm(
    CameraHandle    hCamera,
    INT             iIspProcessor,
    INT             iAlgorithmSel
);

/******************************************************/
// 函数名   : CameraGetBayerDecAlgorithm
// 功能描述 : 获得Bayer数据转彩色所选择的算法。
// 参数     : hCamera       相机的句柄，由CameraInit函数获得。
//            iIspProcessor   选择执行该算法的对象，参考CameraDefine.h
//                emSdkIspProcessor的定义
//            piAlgorithmSel    返回当前选择的算法编号。从0开始，最大值由tSdkCameraCapbility
//                中iBayerDecAlmSwDesc和iBayerDecAlmHdDesc决定。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetBayerDecAlgorithm(
    CameraHandle    hCamera,
    INT             iIspProcessor,
    INT*            piAlgorithmSel
);

/******************************************************/
// 函数名   : CameraSetIspProcessor
// 功能描述 : 设置图像处理单元的算法执行对象，由PC端或者相机端
//        来执行算法，当由相机端执行时，会降低PC端的CPU占用率。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iIspProcessor 参考CameraDefine.h中
//              emSdkIspProcessor的定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetIspProcessor(
    CameraHandle    hCamera,
    INT             iIspProcessor
);

/******************************************************/
// 函数名   : CameraGetIspProcessor
// 功能描述 : 获得图像处理单元的算法执行对象。
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            piIspProcessor 返回选择的对象，返回值参考CameraDefine.h中
//               emSdkIspProcessor的定义。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetIspProcessor(
    CameraHandle    hCamera,
    INT*            piIspProcessor
);

/******************************************************/
// 函数名   : CameraSetBlackLevel
// 功能描述 : 设置图像的黑电平基准，默认值为0
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iBlackLevel 要设定的电平值。范围为0到255。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetBlackLevel(
  CameraHandle    hCamera,
  INT         iBlackLevel
);

/******************************************************/
// 函数名   : CameraGetBlackLevel
// 功能描述 : 获得图像的黑电平基准，默认值为0
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            piBlackLevel 返回当前的黑电平值。范围为0到255。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetBlackLevel(
  CameraHandle    hCamera,
  INT*        piBlackLevel
);

/******************************************************/
// 函数名   : CameraSetWhiteLevel
// 功能描述 : 设置图像的白电平基准，默认值为255
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            iWhiteLevel 要设定的电平值。范围为0到255。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetWhiteLevel(
  CameraHandle    hCamera,
  INT         iWhiteLevel
);

/******************************************************/
// 函数名   : CameraGetWhiteLevel
// 功能描述 : 获得图像的白电平基准，默认值为255
// 参数     : hCamera    相机的句柄，由CameraInit函数获得。
//            piWhiteLevel 返回当前的白电平值。范围为0到255。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetWhiteLevel(
  CameraHandle    hCamera,
  INT*        piWhiteLevel
);

/******************************************************/
// 函数名 	: CameraSetIspOutFormat
// 功能描述	: 设置CameraGetImageBuffer函数的图像处理的输出格式，支持
//              CAMERA_MEDIA_TYPE_MONO8和CAMERA_MEDIA_TYPE_RGB8
//              (在CameraDefine.h中定义)三种，分别对应8位灰度图像和24位彩色图像。
// 参数	    : hCamera		相机的句柄，由CameraInit函数获得。
//             uFormat	要设定格式。CAMERA_MEDIA_TYPE_MONO8或者CAMERA_MEDIA_TYPE_RGB8
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraSetIspOutFormat(
    CameraHandle    hCamera,
    UINT            uFormat
);

/******************************************************/
// 函数名 	: CameraGetIspOutFormat
// 功能描述	: 获得图像处理的输出格式，目前只支持
//              CAMERA_MEDIA_TYPE_MONO8和CAMERA_MEDIA_TYPE_RGB8
//              (在CameraDefine.h中定义)两种，其他的参数会返回错误。
// 参数	    : hCamera		相机的句柄，由CameraInit函数获得。
//             puFormat	返回当前设定的格式。CAMERA_MEDIA_TYPE_MONO8或者CAMERA_MEDIA_TYPE_RGB8
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraGetIspOutFormat(
    CameraHandle    hCamera,
    UINT*           puFormat
);

/******************************************************/
// 函数名 	: CameraGetErrorString
// 功能描述	: 获得错误码对应的描述字符串
// 参数	    : iStatusCode		错误码。(定义于CameraStatus.h中)
// 返回值   : 成功时，返回错误码对应的字符串首地址;
//            否则返回NULL。
/******************************************************/
MVSDK_API char*  CameraGetErrorString(
    CameraSdkStatus     iStatusCode
);


/******************************************************/
// 函数名 	: CameraReConnect
// 功能描述	: 重新连接设备，用于USB设备意外掉线后重连
// 参数	    : hCamera	   相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraReConnect(
    CameraHandle    hCamera
);


/******************************************************/
// 函数名 	: CameraConnectTest
// 功能描述	: 测试相机的连接状态，用于检测相机是否掉线
// 参数	    : hCamera	   相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0)，表示相机连接状态正常;
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraConnectTest(
    CameraHandle    hCamera
);

// 申请一段对齐的内存空间。功能和malloc类似，但是返回的内存是以align指定的字节数对齐的
MVSDK_API BYTE* CameraAlignMalloc(int size, int align);

//释放由CameraAlignMalloc 函数分配的内存空间。
MVSDK_API void CameraAlignFree(BYTE* membuffer);

/******************************************************/
// 函数名   : CameraCommonCall
// 功能描述 : 相机的一些特殊功能调用，二次开发时一般不需要调用。
// 参数     : hCamera   相机的句柄，由CameraInit函数获得。
//            pszCall   功能及参数
//            pszResult 调用结果，不同的pszCall时，意义不同。
//            uResultBufSize pszResult指向的缓冲区的字节大小
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus  CameraCommonCall(
	CameraHandle    hCamera,
	char const*		pszCall,
	char*			pszResult,
	UINT			uResultBufSize
);

/******************************************************/
// 函数名 	: CameraGetEyeCount
// 功能描述	: 获取多目相机的目数
// 参数	    : hCamera	   相机的句柄，由CameraInit函数获得。
// 返回值   : 成功时，返回CAMERA_STATUS_SUCCESS (0);
//            否则返回非0值的错误码,请参考CameraStatus.h
//            中错误码的定义。
/******************************************************/
MVSDK_API CameraSdkStatus CameraGetEyeCount(
    CameraHandle hCamera,
    int *eyecount
);

/// @ingroup API_MULTI_EYE
/// \~chinese
/// \brief 对多目相机帧内的某个单目图做ISP
/// \param [in] hCamera 相机的句柄。
/// \param [in] iEyeIndex 单目索引。
/// \param [in] pbyIn 输入图像数据的缓冲区地址，不能为NULL。 
/// \param [in] pInFrInfo 输入图像数据的帧头，不能为NULL。 
/// \param [out] pbyOut 处理后图像输出的缓冲区地址，不能为NULL。
/// \param [out] pOutFrInfo 处理后图像的帧头信息，不能为NULL。 
/// \param [in] uOutFormat 处理完后图像的输出格式。
/// \param [in] uReserved 预留参数，必须设置为0。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Do ISP for a certain monocular in the multi-camera frame.
/// \param [in] hCamera Camera handle.
/// \param [in] iEyeIndex eye index.
/// \param [in] pbyIn Input the buffer address of the image data, which cannot be NULL.
/// \param [in] pInFrInfo Input the frame header of the image data, which cannot be NULL.
/// \param [out] pbyOut The buffer address of the image output after processing, cannot be NULL.
/// \param [out] pOutFrInfo The header information of the processed image cannot be NULL.
/// \param [in] uOutFormat The output format of the image after processing.
/// \param [in] uReserved Reservation parameters must be set to 0.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraMultiEyeImageProcess(
	CameraHandle        hCamera, 
	int					iEyeIndex,
	BYTE*               pbyIn, 
	tSdkFrameHead*		pInFrInfo,
	BYTE*               pbyOut,
	tSdkFrameHead*      pOutFrInfo,
	UINT                uOutFormat,
	UINT                uReserved
	);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief 设置光源控制器的输出模式（智能相机系列且需要硬件支持）
/// \param [in] hCamera 相机的句柄。
/// \param [in] index 控制器索引
/// \param [in] mode 输出模式（0:跟随闪光灯 1:手动）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the output mode of the light controller (Smart camera series and hardware support required)
/// \param [in] hCamera Camera handle.
/// \param [in] index controller index
/// \param [in] mode output mode (0: follow strobe 1: manual)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetLightingControllerMode(
	CameraHandle        hCamera,
	int					index,
	int					mode
	);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief 设置光源控制器的输出状态（智能相机系列且需要硬件支持）
/// \param [in] hCamera 相机的句柄。
/// \param [in] index 控制器索引
/// \param [in] state 输出状态（0:关闭  1：打开）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the output status of the light controller (Smart camera series and hardware support required)
/// \param [in] hCamera Camera handle.
/// \param [in] index controller index
/// \param [in] state output state (0: off 1: on)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetLightingControllerState(
	CameraHandle        hCamera,
	int					index,
	int					state
	);

/// @ingroup API_UTIL
/// \~chinese
/// \brief 翻转帧数据
/// \param [inout] pFrameBuffer 帧数据
/// \param [in] pFrameHead 帧头
/// \param [in] Flags 1:上下   2：左右    3：上下、左右皆做一次翻转(相当于旋转180度)
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Flip frame data
/// \param [inout] pFrameBuffer frame data
/// \param [in] pFrameHead Frame Header
/// \param [in] Flags 1: Up and down 2: Around 3: Up and down, left and right are all flipped (equivalent to 180 degrees rotation)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraFlipFrameBuffer(
	BYTE *pFrameBuffer,
	tSdkFrameHead* pFrameHead,
	int Flags
	);

/// @ingroup API_UTIL
/// \~chinese
/// \brief 转换帧数据格式
/// \param [in] hCamera 相机的句柄。
/// \param [in] pInFrameBuffer 输入帧数据
/// \param [out] pOutFrameBuffer 输出帧数据
/// \param [in] outWidth 输出宽度
/// \param [in] outHeight 输出高度
/// \param [in] outMediaType 输出格式 @see CameraSetIspOutFormat
/// \param [inout] pFrameHead 帧头信息（转换成功后，里面的信息会被修改为输出帧的信息）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Conversion frame data format
/// \param [in] hCamera Camera handle.
/// \param [in] pInFrameBuffer input frame data
/// \param [out] pOutFrameBuffer output frame data
/// \param [in] outWidth output width
/// \param [in] outHeight output height
/// \param [in] outMediaType output format @see CameraSetIspOutFormat
/// \param [inout] pFrameHead frame header information (after successful conversion, the information inside will be modified to output frame information)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraConvertFrameBufferFormat(
	CameraHandle hCamera,
	BYTE *pInFrameBuffer,
	BYTE *pOutFrameBuffer,
	int outWidth,
	int outHeight,
	UINT outMediaType,
	tSdkFrameHead* pFrameHead
	);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief 获取当前帧的ID，需相机支持(网口全系列支持)，此函数返回错误代码，表示不支持。
/// \param [in] hCamera 相机的句柄。
/// \param [out] id		   帧ID
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief The ID of the current frame needs to be supported by the camera (supported by the full range of network ports). This function returns an error code indicating that it is not supported.
/// \param [in] hCamera Camera handle.
/// \param [out] id Frame ID
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetFrameID(
	CameraHandle    hCamera,
	UINT*           id
	);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief 获取当前帧的时间戳(单位微秒)
/// \param [in] hCamera 相机的句柄。
/// \param [out] TimeStampL   时间戳低32位
/// \param [out] TimeStampH   时间戳高32位
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the timestamp of the current frame (in microseconds)
/// \param [in] hCamera Camera handle.
/// \param [out] TimeStampL timestamp low 32 bits
/// \param [out] TimeStampH Timestamp high 32 bits
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGetFrameTimeStamp(
	CameraHandle    hCamera,
	UINT*           TimeStampL,
	UINT*			TimeStampH
	);

/// @ingroup API_RECONNECT
/// \~chinese
/// \brief 设置相机连接状态改变的回调通知函数。当相机掉线、重连时，pCallBack所指向的回调函数就会被调用。 
/// \param [in] hCamera 相机的句柄。
/// \param [in] pCallBack 回调函数指针。
/// \param [in] pContext  回调函数的附加参数，在回调函数被调用时该附加参数会被传入，可以为NULL。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Sets the callback notification function for camera connection state changes. When the camera is disconnected and reconnected, the callback function pointed to by pCallBack will be called.
/// \param [in] hCamera Camera handle.
/// \param [in] pCallBack callback function pointer.
/// \param [in] pContext Additional parameter of the callback function. This additional parameter will be passed in when the callback function is called. It can be NULL.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetConnectionStatusCallback(
	CameraHandle        hCamera,
	CAMERA_CONNECTION_STATUS_CALLBACK pCallBack,
	PVOID               pContext
	);

/// @ingroup API_ENUM
/// \~chinese
/// \brief 从指定IP枚举GIGE设备，并建立设备列表（适用于相机和电脑不在同一网段的情况）
/// \param [in] ppIpList 目标IP
/// \param [in] numIp 目标IP个数
/// \param [out] pCameraList 设备列表数组指针
/// \param [inout] piNums 设备的个数指针，调用时传入pCameraList数组的元素个数，函数返回时，保存实际找到的设备个数
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义
/// \warning piNums指向的值必须初始化，且不超过pCameraList数组元素个数，否则有可能造成内存溢出
/// \note 返回的相机信息列表，会根据acFriendlyName排序的。例如可以将两个相机分别改为“Camera1”和“Camera2”的名字后，名字为“Camera1”的相机会排前面，名为“Camera2”的相机排后面。
/// \~english
/// \brief Enumerates GIGE devices from the specified IP and builds a device list (applicable when the camera and computer are not on the same network segment)
/// \param [in] ppIpList target IP
/// \param [in] numIp number of target IPs
/// \param [out] pCameraList Device list array pointer
/// \param [inout] piNums The number of pointers to the device, the number of elements passed to the pCameraList array at the time of the call. When the function returns, the number of devices actually found is saved.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \warning piNums The value pointed to must be initialized and does not exceed the number of pCameraList array elements, otherwise it may cause memory overflow
/// \note The list of returned camera information will be sorted according to acFriendlyName. For example, after changing the two cameras to the names of "Camera1" and "Camera2," the camera named "Camera1" will be in front, and the camera named "Camera2" will be behind the row.
MVSDK_API CameraSdkStatus CameraGigeEnumerateDevice(
    char const**        ppIpList,
    int                 numIp,
    tSdkCameraDevInfo*  pCameraList, 
    INT*                piNums
);

/// @ingroup API_UTIL
/// \~chinese
/// \brief 配置网口相机的一些功能选项
/// \param [in] optionName 选项("NumBuffers", "3")
/// \param [in] value 值
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Configure some options for the gige camera
/// \param [in] optionName option name("NumBuffers", "3")
/// \param [in] value setting value
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGigeSetOption(
	char const* optionName,
	char const* value
	);

/// @ingroup API_UTIL
/// \~chinese
/// \brief 获取GIGE相机的IP地址
/// \param [in] pCameraInfo 相机的设备描述信息，可由@link #CameraEnumerateDevice @endlink函数获得。 
/// \param [out] CamIp 相机IP(注意：必须保证传入的缓冲区大于等于16字节)
/// \param [out] CamMask 相机子网掩码(注意：必须保证传入的缓冲区大于等于16字节)
/// \param [out] CamGateWay 相机网关(注意：必须保证传入的缓冲区大于等于16字节)
/// \param [out] EtIp 网卡IP(注意：必须保证传入的缓冲区大于等于16字节)
/// \param [out] EtMask 网卡子网掩码(注意：必须保证传入的缓冲区大于等于16字节)
/// \param [out] EtGateWay 网卡网关(注意：必须保证传入的缓冲区大于等于16字节)
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Get the GIGE camera's IP address
/// \param [in] pCameraInfo camera's device description information can be obtained by @link #CameraEnumerateDevice @endlink function.
/// \param [out] CamIp camera IP (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] CamMask camera subnet mask (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] CamGateWay camera gateway (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] EtIp network card IP (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] EtMask subnet mask (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \param [out] EtGateWay NIC Gateway (Note: must ensure that the incoming buffer is greater than or equal to 16 bytes)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGigeGetIp(
		tSdkCameraDevInfo* pCameraInfo,
		char* CamIp,
		char* CamMask,
		char* CamGateWay,
		char* EtIp,
		char* EtMask,
		char* EtGateWay
	);

/// @ingroup API_UTIL
/// \~chinese
/// \brief 设置GIGE相机的IP地址
/// \param [in] pCameraInfo 相机的设备描述信息，可由@link #CameraEnumerateDevice @endlink函数获得。 
/// \param [in] Ip 相机IP(如：192.168.1.100)
/// \param [in] SubMask 相机子网掩码(如：255.255.255.0)
/// \param [in] GateWay 相机网关(如：192.168.1.1)
/// \param [in] bPersistent TRUE: 设置相机为固定IP，FALSE：设置相机自动分配IP（忽略参数Ip, SubMask, GateWay）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Set the GIGE camera's IP address
/// \param [in] pCameraInfo camera's device description information can be obtained by @link #CameraEnumerateDevice @endlink function.
/// \param [in] Ip camera IP (eg 192.168.1.100)
/// \param [in] SubMask camera subnet mask (eg 255.255.255.0)
/// \param [in] GateWay Camera Gateway (eg 192.168.1.1)
/// \param [in] bPersistent TRUE: Set camera to fixed IP, FALSE: Set camera to assign IP automatically (ignoring parameters Ip, SubMask, GateWay)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGigeSetIp(
	tSdkCameraDevInfo* pCameraInfo,
	char const* Ip,
	char const* SubMask,
	char const* GateWay,
	BOOL bPersistent
	);

/// @ingroup API_UTIL
/// \~chinese
/// \brief 获取GIGE相机的MAC地址
/// \param [in] pCameraInfo 相机的设备描述信息，可由@link #CameraEnumerateDevice @endlink函数获得。 
/// \param [out] CamMac 相机MAC(注意：必须保证传入的缓冲区大于等于18字节)
/// \param [out] EtMac 网卡MAC(注意：必须保证传入的缓冲区大于等于18字节)
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Select the LUT table in the preset LUT mode.
/// \param [in] pCameraInfo camera's device description information can be obtained by @link #CameraEnumerateDevice @endlink function.
/// \param [out] CamMac camera MAC (Note: must ensure that the incoming buffer is greater than or equal to 18 bytes)
/// \param [out] EtMac network card MAC (Note: must ensure that the incoming buffer is greater than or equal to 18 bytes)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraGigeGetMac(
	tSdkCameraDevInfo* pCameraInfo,
	char* CamMac,
	char* EtMac
	);

/// @ingroup API_GRAB
/// \~chinese
/// \brief 清空相机内已缓存的所有帧
/// \param [in] hCamera 相机的句柄。
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Clear all cached frames in the camera
/// \param [in] hCamera Camera handle.
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraClearBuffer(
	CameraHandle hCamera
	);

/// @ingroup API_GRAB
/// \~chinese
/// \brief 获得一帧图像数据。
/// \param [in] hCamera 相机的句柄。
/// \param [out] pFrameInfo  图像的帧头信息指针。
/// \param [out] pbyBuffer   指向图像的数据的缓冲区指针。
/// \param [in] wTimes 抓取图像的超时时间。
/// \param [in] Priority 取图优先级 详见：@link #emCameraGetImagePriority @endlink
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \note 除了多一个优先级参数外与@link #CameraGetImageBuffer @endlink相同
/// \~english
/// \brief Get a frame of image data.
/// \param [in] hCamera Camera handle.
/// \param [out] pFrameInfo Frame header information pointer
/// \param [out] pbyBuffer Pointer to the buffer of data for the image.
/// \param [in] wTimes The time-out time for capturing images.
/// \param [in] Priority map priority See: @link #emCameraGetImagePriority @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \note Same as @link #CameraGetImageBuffer @endlink except one more priority parameter
MVSDK_API CameraSdkStatus CameraGetImageBufferPriority(
	CameraHandle        hCamera, 
	tSdkFrameHead*      pFrameInfo, 
	BYTE**              pbyBuffer,
	UINT                wTimes,
	UINT				Priority
	);

/// @ingroup API_GRAB
/// \~chinese
/// \brief 获得一帧图像数据。该接口获得的图像是经过处理后的RGB格式。
/// \param [in] hCamera 相机的句柄。
/// \param [out] piWidth    整形指针，返回图像的宽度
/// \param [out] piHeight   整形指针，返回图像的高度
/// \param [in] wTimes 抓取图像的超时时间。单位毫秒。
/// \param [in] Priority 取图优先级 详见：@link #emCameraGetImagePriority @endlink
/// \return 成功时，返回RGB数据缓冲区的首地址;否则返回0。
/// \note 除了多一个优先级参数外与@link #CameraGetImageBufferEx @endlink相同
/// \~english
/// \brief Get a frame of image data. The image obtained by this interface is the processed RGB format.
/// \param [in] hCamera Camera handle.
/// \param [out] piWidth Returns the width of the image
/// \param [out] piHeight Returns the height of the image
/// \param [in] wTimes The time-out time for capturing images. The unit is milliseconds.
/// \param [in] Priority map priority See: @link #emCameraGetImagePriority @endlink
/// \return Returns the first address of the RGB data buffer when successful; otherwise returns 0.
/// \note Same as @link #CameraGetImageBufferEx @endlink except one more priority parameter
MVSDK_API unsigned char* CameraGetImageBufferPriorityEx(
	CameraHandle        hCamera, 
	INT*                piWidth,
	INT*                piHeight,
	UINT                wTimes,
	UINT				Priority
	);

/// @ingroup API_TRIGGER
/// \~chinese
/// \brief 执行软触发。
/// \param [in] hCamera 相机的句柄。
/// \param [in] uFlags 功能标志,详见@link #emCameraSoftTriggerExFlags @endlink中的定义
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \see CameraSoftTrigger
/// \~english
/// \brief Perform a soft trigger.
/// \param [in] hCamera Camera handle.
/// \param [in] uFlags function flags, as defined in @link #emCameraSoftTriggerExFlags @endlink
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
/// \see CameraSoftTrigger
MVSDK_API CameraSdkStatus CameraSoftTriggerEx(
	CameraHandle hCamera,
	UINT uFlags
	);

/// @ingroup API_ADVANCE
/// \~chinese
/// \brief 当相机处于触发模式（软触发或硬触发）时，相机发送一帧到PC，如相机未收到PC端的接收确认，相机可以把帧重发几次。用本函数设置相机重发次数。（仅网口相机支持）
/// \param [in] hCamera 相机的句柄。
/// \param [in] count 重发次数（<=0表示禁用重发功能）
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief When the camera is in the trigger mode (soft trigger or hard trigger), the camera sends a frame to the PC. If the camera does not receive the reception confirmation from the PC, the camera can retransmit the frame several times. Use this function to set the number of camera resends. (only supported by Gige camera)
/// \param [in] hCamera Camera handle.
/// \param [in] count number of resends (<=0 means disable resends)
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetFrameResendCount(
	CameraHandle        hCamera,
	int					count
	);

/// @ingroup API_BASIC
/// \~chinese
/// \brief 配置系统选项（通常需要在CameraInit打开相机之前配置好）
/// \param [in] optionName 选项("NumBuffers", "3")
/// \param [in] value 值
/// \return 成功返回 CAMERA_STATUS_SUCCESS(0)。否则返回非0值的错误码, 请参考 CameraStatus.h 中错误码的定义。
/// \~english
/// \brief Configure system options (usually required before CameraInit turns on the camera)
/// \param [in] optionName option name("NumBuffers", "3")
/// \param [in] value setting value
/// \return Returns CAMERA_STATUS_SUCCESS(0) successfully. Otherwise, it returns a non-zero error code. Please refer to the definition of the error code in CameraStatus.h.
MVSDK_API CameraSdkStatus CameraSetSysOption(
	char const* optionName,
	char const* value
	);

#ifdef __cplusplus
}
#endif
#endif

