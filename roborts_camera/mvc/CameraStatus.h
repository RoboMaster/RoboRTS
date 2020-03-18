
#ifndef __CAMERA_STATUS_DEF__
#define __CAMERA_STATUS_DEF__

typedef int CameraSdkStatus;


/*常用的宏*/
#define SDK_SUCCESS(_FUC_)              ((_FUC_) == CAMERA_STATUS_SUCCESS)

#define SDK_UNSUCCESS(_FUC_)            ((_FUC_) != CAMERA_STATUS_SUCCESS)

#define SDK_UNSUCCESS_RETURN(_FUC_,RET) if((RET = (_FUC_)) != CAMERA_STATUS_SUCCESS)\
                                        {\
                                            return RET;\
                                        }

#define SDK_UNSUCCESS_BREAK(_FUC_)      if((_FUC_) != CAMERA_STATUS_SUCCESS)\
                                        {\
                                            break;\
                                        }


/* 常用错误  */

#define CAMERA_STATUS_SUCCESS                    0   // 操作成功
#define CAMERA_STATUS_FAILED                    -1   // 操作失败
#define CAMERA_STATUS_INTERNAL_ERROR            -2   // 内部错误
#define CAMERA_STATUS_UNKNOW                    -3   // 未知错误
#define CAMERA_STATUS_NOT_SUPPORTED             -4   // 不支持该功能
#define CAMERA_STATUS_NOT_INITIALIZED           -5   // 初始化未完成
#define CAMERA_STATUS_PARAMETER_INVALID         -6   // 参数无效
#define CAMERA_STATUS_PARAMETER_OUT_OF_BOUND    -7   // 参数越界
#define CAMERA_STATUS_UNENABLED                 -8   // 未使能
#define CAMERA_STATUS_USER_CANCEL               -9   // 用户手动取消了，比如roi面板点击取消，返回
#define CAMERA_STATUS_PATH_NOT_FOUND            -10  // 注册表中没有找到对应的路径
#define CAMERA_STATUS_SIZE_DISMATCH             -11  // 获得图像数据长度和定义的尺寸不匹配
#define CAMERA_STATUS_TIME_OUT                  -12  // 超时错误
#define CAMERA_STATUS_IO_ERROR                  -13  // 硬件IO错误
#define CAMERA_STATUS_COMM_ERROR                -14  // 通讯错误
#define CAMERA_STATUS_BUS_ERROR                 -15  // 总线错误
#define CAMERA_STATUS_NO_DEVICE_FOUND           -16  // 没有发现设备
#define CAMERA_STATUS_NO_LOGIC_DEVICE_FOUND     -17  // 未找到逻辑设备
#define CAMERA_STATUS_DEVICE_IS_OPENED          -18  // 设备已经打开
#define CAMERA_STATUS_DEVICE_IS_CLOSED          -19  // 设备已经关闭
#define CAMERA_STATUS_DEVICE_VEDIO_CLOSED       -20  // 没有打开设备视频，调用录像相关的函数时，如果相机视频没有打开，则回返回该错误。
#define CAMERA_STATUS_NO_MEMORY                 -21  // 没有足够系统内存
#define CAMERA_STATUS_FILE_CREATE_FAILED        -22  // 创建文件失败
#define CAMERA_STATUS_FILE_INVALID              -23  // 文件格式无效
#define CAMERA_STATUS_WRITE_PROTECTED           -24  // 写保护，不可写
#define CAMERA_STATUS_GRAB_FAILED               -25  // 数据采集失败
#define CAMERA_STATUS_LOST_DATA                 -26  // 数据丢失，不完整
#define CAMERA_STATUS_EOF_ERROR                 -27  // 未接收到帧结束符
#define CAMERA_STATUS_BUSY                      -28  // 正忙(上一次操作还在进行中)，此次操作不能进行
#define CAMERA_STATUS_WAIT                      -29  // 需要等待(进行操作的条件不成立)，可以再次尝试trf
#define CAMERA_STATUS_IN_PROCESS                -30  // 正在进行，已经被操作过
#define CAMERA_STATUS_IIC_ERROR                 -31  // IIC传输错误
#define CAMERA_STATUS_SPI_ERROR                 -32  // SPI传输错误
#define CAMERA_STATUS_USB_CONTROL_ERROR         -33  // USB控制传输错误
#define CAMERA_STATUS_USB_BULK_ERROR            -34  // USB BULK传输错误
#define CAMERA_STATUS_SOCKET_INIT_ERROR         -35  // 网络传输套件初始化失败
#define CAMERA_STATUS_GIGE_FILTER_INIT_ERROR    -36  // 网络相机内核过滤驱动初始化失败，请检查是否正确安装了驱动，或者重新安装。
#define CAMERA_STATUS_NET_SEND_ERROR            -37  // 网络数据发送错误
#define CAMERA_STATUS_DEVICE_LOST               -38  // 与网络相机失去连接，心跳检测超时
#define CAMERA_STATUS_DATA_RECV_LESS            -39  // 接收到的字节数比请求的少
#define CAMERA_STATUS_FUNCTION_LOAD_FAILED      -40  // 从文件中加载程序失败
#define CAMERA_STATUS_CRITICAL_FILE_LOST        -41  // 程序运行所必须的文件丢失。
#define CAMERA_STATUS_SENSOR_ID_DISMATCH        -42  // 固件和程序不匹配，原因是下载了错误的固件。
#define CAMERA_STATUS_OUT_OF_RANGE              -43  // 参数超出有效范围。
#define CAMERA_STATUS_REGISTRY_ERROR            -44  // 安装程序注册错误。请重新安装程序，或者运行安装目录Setup/Installer.exe
#define CAMERA_STATUS_ACCESS_DENY               -45  // 禁止访问。指定相机已经被其他程序占用时，再申请访问该相机，会返回该状态。(一个相机不能被多个程序同时访问)
#define CAMERA_STATUS_CAMERA_NEED_RESET         -46  // 表示相机需要复位后才能正常使用，此时请让相机断电重启，或者重启操作系统后，便可正常使用。
#define CAMERA_STATUS_ISP_MOUDLE_NOT_INITIALIZED -47 // ISP模块未初始化
#define CAMERA_STATUS_ISP_DATA_CRC_ERROR        -48  // 数据校验错误
#define CAMERA_STATUS_MV_TEST_FAILED            -49  // 数据测试失败
#define CAMERA_STATUS_INTERNAL_ERR1             -50	 // 内部错误1
#define CAMERA_STATUS_U3V_NO_CONTROL_EP			-51	 // U3V控制端点未找到
#define CAMERA_STATUS_U3V_CONTROL_ERROR			-52	 // U3V控制通讯错误




//和AIA制定的标准相同
/*#define CAMERA_AIA_SUCCESS                        0x0000 */
#define CAMERA_AIA_PACKET_RESEND                          0x0100 //该帧需要重传
#define CAMERA_AIA_NOT_IMPLEMENTED                        0x8001 //设备不支持的命令
#define CAMERA_AIA_INVALID_PARAMETER                      0x8002 //命令参数非法
#define CAMERA_AIA_INVALID_ADDRESS                        0x8003 //不可访问的地址
#define CAMERA_AIA_WRITE_PROTECT                          0x8004 //访问的对象不可写
#define CAMERA_AIA_BAD_ALIGNMENT                          0x8005 //访问的地址没有按照要求对齐
#define CAMERA_AIA_ACCESS_DENIED                          0x8006 //没有访问权限
#define CAMERA_AIA_BUSY                                   0x8007 //命令正在处理中
#define CAMERA_AIA_DEPRECATED                             0x8008 //0x8008-0x0800B  0x800F  该指令已经废弃
#define CAMERA_AIA_PACKET_UNAVAILABLE                     0x800C //包无效
#define CAMERA_AIA_DATA_OVERRUN                           0x800D //数据溢出，通常是收到的数据比需要的多
#define CAMERA_AIA_INVALID_HEADER                         0x800E //数据包头部中某些区域与协议不匹配
#define CAMERA_AIA_PACKET_NOT_YET_AVAILABLE               0x8010 //图像分包数据还未准备好，多用于触发模式，应用程序访问超时
#define CAMERA_AIA_PACKET_AND_PREV_REMOVED_FROM_MEMORY    0x8011 //需要访问的分包已经不存在。多用于重传时数据已经不在缓冲区中
#define CAMERA_AIA_PACKET_REMOVED_FROM_MEMORY             0x8012 //CAMERA_AIA_PACKET_AND_PREV_REMOVED_FROM_MEMORY
#define CAMERA_AIA_NO_REF_TIME                            0x0813 //没有参考时钟源。多用于时间同步的命令执行时
#define CAMERA_AIA_PACKET_TEMPORARILY_UNAVAILABLE         0x0814 //由于信道带宽问题，当前分包暂时不可用，需稍后进行访问
#define CAMERA_AIA_OVERFLOW                               0x0815 //设备端数据溢出，通常是队列已满
#define CAMERA_AIA_ACTION_LATE                            0x0816 //命令执行已经超过有效的指定时间
#define CAMERA_AIA_ERROR                                  0x8FFF //错误





#endif
