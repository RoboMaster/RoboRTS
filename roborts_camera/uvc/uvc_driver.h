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

#ifndef ROBORTS_CAMERA_UVC_DRIVER_H
#define ROBORTS_CAMERA_UVC_DRIVER_H

#include <thread>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "actionlib/server/simple_action_server.h"

#include "../camera_param.h"
#include "../camera_base.h"
#include "alg_factory/algorithm_factory.h"
#include "io/io.h"

namespace roborts_camera {
/**
 * @brief UVC Camera class, product of the camera factory inherited from CameraBase
 */
class UVCDriver: public CameraBase {
 public:
  /**
   * @brief Constructor of UVCDriver
   * @param camera_info  Information and parameters of camera
   */
  explicit UVCDriver(CameraInfo camera_info);
  /**
   * @brief Start to read uvc camera
   * @param img Image data in form of cv::Mat to be read
   */
  void StartReadCamera(cv::Mat &img) override;
  /**
   * @brief Stop to read uvc camera
   */
  void StopReadCamera();
  ~UVCDriver() override;
 private:
  /**
   * @brief Set camera exposure
   * @param id Camera path
   * @param val Camera exposure value
   */
  void SetCameraExposure(std::string id, int val);
  //! Initialization of camera read
  bool read_camera_initialized_;
};

roborts_common::REGISTER_ALGORITHM(CameraBase, "uvc", UVCDriver, CameraInfo);

} //namespace roborts_camera
#endif //ROBORTS_CAMERA_UVC_DRIVER_H
