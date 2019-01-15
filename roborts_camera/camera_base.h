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

#ifndef ROBORTS_CAMERA_CAMERA_BASE_H
#define ROBORTS_CAMERA_CAMERA_BASE_H

#include <vector>
#include <opencv2/opencv.hpp>

#include "state/error_code.h"

namespace roborts_camera
{
/**
 * @brief Camera base class for the camera factory
 */
class CameraBase {
 public:
  /**
   * @brief Constructor of CameraBase
   * @param camera_info  Information and parameters of camera
   */
  explicit CameraBase(CameraInfo camera_info):camera_info_(camera_info),camera_initialized_(false){};
  virtual ~CameraBase() = default;
  /**
   * @brief Start to read camera
   * @param img Image data in form of cv::Mat to be read
   */
  virtual void StartReadCamera(cv::Mat &img) = 0;

 protected:
  //! flag for camera initialization
  bool camera_initialized_;
  //! information and parameters of camera
  CameraInfo camera_info_;
};
} //namespace roborts_camera


#endif //ROBORTS_CAMERA_CAMERA_BASE_H