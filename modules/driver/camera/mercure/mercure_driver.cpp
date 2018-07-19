#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>

#include "common/timer.h"
#include "modules/driver/camera/mercure/mercure_driver.h"

namespace rrts{
namespace driver {
namespace camera {

MercureDriver::MercureDriver() {
  frame_data_ = nullptr;
  device_ = nullptr;
  //API接口函数返回值
  status_ = GX_STATUS_SUCCESS;
  camera_initialized_ = false;
  frame_count_ = 0;
  frame_rate_ = 0;
  LoadParam();
}

void MercureDriver::LoadParam() {
  uint32_t device_num = 0;
  uint8_t  ui8DevInfo[64] = {0};
  size_t   iSize = 0;

  //初始化库
  status_ = GXInitLib();
  if(status_ != GX_STATUS_SUCCESS) {
    return;
  }

  printf("GXUpdateDeviceList(&device_num, 1000)\n");
  //更新设备列表
  status_ = GXUpdateDeviceList(&device_num, 1000);

  //获取枚举设备个数
  if(status_ != GX_STATUS_SUCCESS) {
    status_ = GXCloseLib();
    return;
  }

  if(device_num <= 0) {
    printf("<No device>\n");
    status_ = GXCloseLib();
    return;
  } else {
    printf("DeviceNum is: %d\n", device_num);
  }

  status_ = GXOpenDeviceByIndex(0, &device_);
  printf("Open Device status_ = %d\n",status_);

  //加载默认参数组
  iSize = sizeof(int32_t);
  uint32_t ui32DefaultParameterGroup = 1;
  status_ = GXWriteRemoteDevicePort(device_, 0x00800030, &ui32DefaultParameterGroup, &iSize);

  //get info
  //get dev model
  iSize = sizeof(ui8DevInfo);
  status_ = GXReadRemoteDevicePort(device_, 0x00000044, &ui8DevInfo, &iSize);
  printf("DeviceModelName: %s\n", ui8DevInfo);
  //get SN
  status_ = GXReadRemoteDevicePort(device_, 0x00000144, &ui8DevInfo, &iSize);
  printf("DeviceSerialNum: %s\n", ui8DevInfo);
  //get dev version
  status_ = GXReadRemoteDevicePort(device_, 0x000000C4, &ui8DevInfo, &iSize);
  printf("DeviceVersion: %s\n", ui8DevInfo);
  //get dev firmware version
  status_ = GXReadRemoteDevicePort(device_, 0x00300000, &ui8DevInfo, &iSize);
  printf("DeviceFirmwareVersion: %s\n", ui8DevInfo);
  //get dev Factory Setting version
  status_ = GXReadRemoteDevicePort(device_, 0x0080007C, &ui8DevInfo, &iSize);
  printf("FactorySettingVersion: %s\n", ui8DevInfo);
  //get VendorName
  status_ = GXReadRemoteDevicePort(device_, 0x00000004, &ui8DevInfo, &iSize);
  printf("DeviceVendorName: %s\n", ui8DevInfo);

  //设置采集队列Buffer个数
  uint64_t nBufferNum = 1;
  GXSetAcqusitionBufferNumber(device_, nBufferNum);

  //设置URB大小和个数
  GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, (64*1024));
  GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, 64);

  // set trigger Mode
  int32_t i32SetTriggerMode = 0;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x00900020, &i32SetTriggerMode, &iSize);

  // Get trigger Mode
  int32_t i32GetTriggerMode = 0;
  iSize = sizeof(int32_t);
  status_ = GXReadRemoteDevicePort(device_, 0x00900020, &i32GetTriggerMode, &iSize);
  printf("TriggerMode: %d\n", i32GetTriggerMode);

  // Get Sensor MAX Resolution
  int32_t i32SensorWidth = 0;
  int32_t i32SensorHeight = 0;
  iSize = sizeof(int32_t);
  status_ = GXReadRemoteDevicePort(device_, 0x00800050, &i32SensorWidth, &iSize);
  status_ = GXReadRemoteDevicePort(device_, 0x00800054, &i32SensorHeight, &iSize);
  printf("SensorWidth: %d  SensorHeight: %d\n", i32SensorWidth, i32SensorHeight);

  // set ThroughputLimitMode
  int32_t i32ThroughputLimitMode = 0;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x009000D0, &i32ThroughputLimitMode, &iSize);

  // set ThroughputLimitValue
  int32_t i32ThroughputLimitVal = 400000000;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x009000D4, &i32ThroughputLimitVal, &iSize);

  // 设置帧率控制开关，1为开启，如果不开启，U3V相机接在U2口上可能会丢帧
  int32_t i32ControlFrameRate = 0;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x009000E8, &i32ControlFrameRate, &iSize);

  // 设置帧率值，设置值=帧率×10，例如设置值为100，相机输出帧率为10.0fps
  int32_t i32FrameRate = camera_param_.GetCameraParam()[0].fps * 10;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x009000EC, &i32FrameRate, &iSize);

  //　打开自动曝光，1为开启，0为关闭
  int32_t i32AutoExposure = camera_param_.GetCameraParam()[0].auto_exposure;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x0090003C, &i32AutoExposure, &iSize);

  // 设置曝光, 单位是us
  int32_t i32ExposureTime = camera_param_.GetCameraParam()[0].exposure_time;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x00900038, &i32ExposureTime, &iSize);
  if(status_ != GX_STATUS_SUCCESS)
  {
    printf("Set Exposure time Failed ret= %d\n",status_);
  }

  // 读取曝光时间
  int32_t i32GetExpTime = 0;
  iSize = sizeof(int32_t);
  status_ = GXReadRemoteDevicePort(device_, 0x00900038, &i32GetExpTime, &iSize);

  // 读取曝光时间范围
  int32_t i32GetExpTimeMin = 0;
  iSize = sizeof(int32_t);
  status_ = GXReadRemoteDevicePort(device_, 0x0060003C, &i32GetExpTimeMin, &iSize);

  int32_t i32GetExpTimeMax = 0;
  iSize = sizeof(int32_t);
  status_ = GXReadRemoteDevicePort(device_, 0x00600038, &i32GetExpTimeMax, &iSize);

  int32_t i32GetExpTimePrecision = 0;
  iSize = sizeof(int32_t);
  status_ = GXReadRemoteDevicePort(device_, 0x0070001C, &i32GetExpTimePrecision, &iSize);
  printf("ExposureTime: %d ExposureTimeMax: %d ExposureTimeMin: %d ExposureTimePrecision: %d\n", \
            i32GetExpTime, i32GetExpTimeMax, i32GetExpTimeMin, i32GetExpTimePrecision);

  //打开自动白平衡，1为开启，0为关闭
  int32_t i32value = camera_param_.GetCameraParam()[0].auto_white_balance;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x009000B4, &i32value, &iSize);

//  //设置红分量系数，范围256~2047
//   iSize = sizeof(int32_t);
//   int32_t i32RedRatio = 256;
//   status_ = GXWriteRemoteDevicePort(device_, 0x009000A8, &i32RedRatio, &iSize);
//  //设置绿分量系数，范围256~2047
//   iSize = sizeof(int32_t);
//   int32_t i32GreenRatio = 256;
//   status_ = GXWriteRemoteDevicePort(device_, 0x009000AC, &i32GreenRatio, &iSize);
//  //设置蓝分量系数，范围256~2047
//   iSize = sizeof(int32_t);
//   int32_t i32BlueRatio = 256;
//   status_ = GXWriteRemoteDevicePort(device_, 0x009000B0, &i32BlueRatio, &iSize);

  //打开自动增益，1为开启，0为关闭
  int32_t i32AutoGain = camera_param_.GetCameraParam()[0].auto_gain;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x0090009C, &i32AutoGain, &iSize);

  //设置增益，必须关闭自动增益
  //设置增益，范围0~160
  iSize = sizeof(int32_t);
  int32_t i32Gain = 0;
  status_ = GXWriteRemoteDevicePort(device_, 0x00900098, &i32Gain, &iSize);

  //设置图像格式
  i32value = GX_PIXEL_FORMAT_BAYER_RG8;
  iSize = sizeof(int32_t);
  status_ = GXWriteRemoteDevicePort(device_, 0x00900018, &i32value, &iSize);

  uint32_t *width, *height, *h_offset, *w_offset;
  uint32_t width_value    = camera_param_.GetCameraParam()[0].resolution_width;
  uint32_t height_value   = camera_param_.GetCameraParam()[0].resolution_height;

  uint32_t w_offset_value = camera_param_.GetCameraParam()[0].width_offset;
  uint32_t h_offset_value = camera_param_.GetCameraParam()[0].height_offset;

  width = &width_value;
  height= &height_value;
  w_offset = &w_offset_value;
  h_offset = &h_offset_value;
  //video_writer_.open("test_video.avi", 
  //        CV_FOURCC('P','I','M','1'), 
  //        30, 
  //        cv::Size(960, 600));

  //Set camera size
  //width
  GXWriteRemoteDevicePort(device_, 0x00900008, width, &iSize);

  //height
  GXWriteRemoteDevicePort(device_, 0x0090000C, height, &iSize);

  //w_offset
  GXWriteRemoteDevicePort(device_, 0x00900010, w_offset, &iSize);

  //h_offset
  GXWriteRemoteDevicePort(device_, 0x00900014, h_offset, &iSize);
}

void MercureDriver::Init(){
  status_ = GxStreamOn(device_);
  if (status_ != GX_STATUS_SUCCESS){
    printf("AcqusitionStart Failed ret= %d\n",status_);
  }
  pu8_image_buf_ = new uint8_t[10*1024*1024];
  camera_initialized_ = true;
}

void MercureDriver::StartReadCamera(unsigned int camera_num, cv::Mat &img){
  if(camera_initialized_) {
    //TIMER_START(mercure)
    GxDQBuf(device_, &frame_data_, 1000);
    //sleep(1);
    if (frame_data_ != nullptr) {
      // 丢帧统计
      if (frame_data_->nFrameID == 0) {
        last_frame_id_ = frame_data_->nFrameID;
      } else {
        if (frame_data_->nFrameID != last_frame_id_ + 1) {
          lost_count_ = frame_data_->nFrameID - last_frame_id_;
        }

        last_frame_id_ = frame_data_->nFrameID;
      }

      // 统计fps
      fps_.IncreaseFrameNum();
      fps_.UpdateFps();
      frame_rate_ = fps_.GetFps();

      //static cv::Mat test(frame_data_->nHeight, frame_data_->nWidth, CV_8UC1);
      //memcpy(test.data, frame_data_->pImgBuf, frame_data_->nHeight * frame_data_->nWidth);
      //cv::cvtColor(test, img, CV_BayerBG2BGR);

      DxRaw8toRGB24(frame_data_->pImgBuf,
                    pu8_image_buf_,
                    frame_data_->nWidth,
                    frame_data_->nHeight,
                    RAW2RGB_NEIGHBOUR,
                    DX_PIXEL_COLOR_FILTER(4),
                    false);
      
      static cv::Mat test(frame_data_->nHeight, frame_data_->nWidth, CV_8UC3);
      memcpy(test.data, pu8_image_buf_, frame_data_->nHeight * frame_data_->nWidth * 3);
      img = test.clone();
      //cv::resize(test, img, cv::Size(960, 600), 0, 0, CV_INTER_AREA);
      //video_writer_.write(img);
      //cv::waitKey(1);
      //if(i%2 == 0) {
      //  cv::imshow("test", test);
      //  cv::waitKey(1);
      //}
      //间隔100帧保存一幅图像，用户可自行开启
      //if (i % 10 == 0)
      //{
      //    SaveRawFile(pu8_image_buf_, frame_data_->nWidth, frame_data_->nHeight);
      //}
      //printf("imageSize = %d Width=%d,height=%d,PixelFormat=%x\n\n",
      //       frame_data_->nImgSize,
      //       frame_data_->nWidth,
      //       frame_data_->nHeight,
      //       frame_data_->nPixelFormat);
      //printf("AcqFrameRate= %d FrameID=%llu Timestamp=%llu nLostFrameCount = %d\n\n",
      //       frame_rate_,
      //       frame_data_->nFrameID,
      //       frame_data_->nTimestamp,
      //       lost_count_);

      int64_t i64RecvFrameCount = 0;
      int64_t i64NoBufLostFrameCount = 0;
      int64_t i64IncompleteFrameCount = 0;

      GXGetInt(device_, GX_DS_INT_DELIVERED_FRAME_COUNT, &i64RecvFrameCount);
      GXGetInt(device_, GX_DS_INT_LOST_FRAME_COUNT, &i64NoBufLostFrameCount);
      GXGetInt(device_, GX_DS_INT_INCOMPLETE_FRAME_COUNT, &i64IncompleteFrameCount);
      //printf("libgxiapi Statistic RecvFrameCount = %llu NoBufLostFrameCount=%llu,IncompleteFrameCount=%llu\n\n",
      //       i64RecvFrameCount,
      //       i64NoBufLostFrameCount,
      //       i64IncompleteFrameCount);

      //将图像buffer放回库中
      GxQBuf(device_, frame_data_);
      //TIMER_END(mercure)
    } else {
      //can not open camera
    }
  } else {
    Init();
  }
}

void MercureDriver::StopReadCamera() {
  //停止采集
  GxStreamOff(device_);
  //关闭设备
  GXCloseDevice(device_);
  //释放库
  status_ = GXCloseLib();
  printf("App Exit!\n");
  delete[] pu8_image_buf_;
}
//-------------------------------------------------
/**
\brief 保存raw数据文件
\param pImgBuffer raw图像数据
\param nWidth 图像宽
\param nHeight 图像高
\return void
*/
//-------------------------------------------------
void MercureDriver::SaveRawFile(void *pImgBuffer, size_t nWidth, size_t nHeight)
{
  char name[64] = {0};

  static int nRawFileIndex = 1;
  FILE* ff = nullptr;

  sprintf(name, "RAW%d.ppm", nRawFileIndex++);
  ff=fopen(name,"wb");
  if(ff)
  {
    fprintf(ff, "P6\n" "%u %u 255\n", nWidth, nHeight);
    fwrite(pImgBuffer, 1, nWidth * nHeight * 4, ff);
    fclose(ff);
    printf("保存%s成功\n", name);
  }
}

MercureDriver::~MercureDriver() {
  StopReadCamera();
}
} //namespace camera
} //namespace driver
} //namespace rrts
