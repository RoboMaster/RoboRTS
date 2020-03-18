#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <image_transport/image_transport.h>

#include <opencv2/core/hpp>
#include "mvc_driver.h"
#include "CameraApi.h"

namespace roborts_camera{
    MVCDriver::MVCDriver(CameraInfo camera_info): // The purpose of using Camera Info is just format work
        CameraBase(CameraInfo){ // Invoke Father class's constructor
            CameraSdkInit(1);
            iStatus =  CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
            std::cout<<"state = "<<iStatus<<std::endl;
            std::cout<<"count = "<<iCameraCounts<<std::endl;
            if(iCameraCounts == 0)
            {
                std::cout<<"No Camera is detected"<<std::endl;
                camera_initialized_ = false;
                return;
            }

            iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
            std::cout<<"state = "<<iStatus<<std::endl;
            if(iStatus != CAMERA_STATUS_SUCCESS)
            {
                std::cout<<"Camera init Failed"<<std::endl;
                camera_initialized_ = false;
                return;
            }
            CameraGetCapability(hCamera,&tCapability);
            g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

            CameraPlay(hCamera);
            CameraSetIspOutFormat(hCamera.CAMERA_MEDIA_TYPE_BGR8);
        }
    void MVCDriver::StartReadCamera(cv::Mat &img)
    {
        if(CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            CameraimageProcess(jCameram pbyBuffer, g_pRgbBuffer, $sFrameInfo);
            cv::Mat matImage(
                cv::cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8? CV_8UC1: CV_8UC3,
                g_pRgbBuffer
            );
            img = matImage;
            CameraReleaseImageBuffer(hCamera,pbyBuffer);
        }
    }

    void MVCDriver::StopReadCamera() // Not complete
    {
        CameraUnit(hCamera);
        free(g_pRgbBuffer);
    }

    MVCDriver::~MVCDriver(){}
}

