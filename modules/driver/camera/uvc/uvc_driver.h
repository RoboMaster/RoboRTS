/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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

#ifndef DRIVER_CAMERA_UVC_H
#define DRIVER_CAMERA_UVC_H

#include <thread>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "actionlib/server/simple_action_server.h"

#include "modules/driver/camera/camera_param.h"
#include "modules/driver/camera/camera_base.h"
#include "common/algorithm_factory.h"
#include "common/rrts.h"
#include "common/io.h"
#include "common/log.h"

namespace rrts {
namespace driver {
namespace camera {

class UVCDriver: public CameraBase {
 public:
  UVCDriver();
  void LoadParam() override;
  void Init(unsigned int camera_num);
  /**
   * Initializing the image buffer and creating a new thread to read camera.
   */
  void StartReadCamera(unsigned int camera_num, cv::Mat &img) override;
  void StopReadCamera();
  void SetCameraExposure(std::string id, int val);
  ~UVCDriver() override;
 private:
  CameraParam camera_param_;
  std::vector<CameraInfo> cameras_;
  bool read_camera_initialized_;
};

rrts::common::REGISTER_ALGORITHM(CameraBase, "uvc", UVCDriver);

} //namespace camera
} //namespace driver
} //namespace rrts
#endif //DRIVER_CAMERA_UVC_H
