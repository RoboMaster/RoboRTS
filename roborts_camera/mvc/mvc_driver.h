#ifndef ROBORTS_CAMERA_MVC_DRIVER_H
#define ROBORTS_CAMERA_MVC_DRIVER_H

#include <thread>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "actionlib/server/simple_action_server.h"

#include "../camera_param.h"
#include "../camera_base.h"
#include "alg_factory/algorithm_factory.h"

#include "io/io.h"

namespace roborts_camera {
    class MVCDriver: public CameraBase{
        public:
            explicit MVCDriver(CameraInfo camera_info);
            void StartReadCamera(cv::Mat &img) override;
            void StopReadCamera();
            ~MVCDriver() override;
        private:
            bool                                  read_camera_initialized_;
            unsigned char              *g_pRgbBuffer;
            int                                       iCameraCounts = 1;
            int                                       iStatus = -1;
            tSdkCameraDevInfo   tCameraEnumList;
            int                                       hCamera;
            tSdkCameraCapbility tCapability;
            tSdkFrameHead            sFrameInfo;
            BYTE*                                 pbyBuffer;
            int                                        iDisplayFrames = 10000;
            IplImage                           *iplImage   = NULL;
            int                                        channel = 3;
    };
    roborts_common::REGISTER_ALGORITHM(CameraBase,"mvc",MVCDriver,CameraInfo);
}
#endif